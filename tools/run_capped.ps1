# Run the router inside a Windows job object with a hard memory limit.
#
# The engine's --memory-budget-mb bounds its own batch/scratch accounting, but
# it is not a hard RSS cap. This wrapper enforces a real limit at the OS level:
# allocations beyond it fail, so a runaway route cannot starve the machine.
#
# Usage (PowerShell):
#   powershell -NoProfile -ExecutionPolicy Bypass -File tools/run_capped.ps1 `
#     -LimitMB 1024 -Exe build/router.exe route board.json --json `
#     --memory-budget-mb 1024 --timeout 900
#
# It prints: CAPPED_RUN exit=<code> peak_rss_mb=<n> wall_s=<t> limit_mb=<n>

param(
    [int]$LimitMB = 1024,
    [string]$Exe = "build\router.exe",
    [Parameter(ValueFromRemainingArguments = $true)][string[]]$RouterArgs
)

Add-Type -TypeDefinition @"
using System;
using System.Runtime.InteropServices;
public static class CopperlineJobMem {
  [StructLayout(LayoutKind.Sequential)] public struct BASIC {
    public long PerProcessUserTimeLimit;
    public long PerJobUserTimeLimit;
    public uint LimitFlags;
    public UIntPtr MinimumWorkingSetSize;
    public UIntPtr MaximumWorkingSetSize;
    public uint ActiveProcessLimit;
    public UIntPtr Affinity;
    public uint PriorityClass;
    public uint SchedulingClass;
  }
  [StructLayout(LayoutKind.Sequential)] public struct IO {
    public ulong a, b, c, d, e, f;
  }
  [StructLayout(LayoutKind.Sequential)] public struct EXT {
    public BASIC Basic;
    public IO Io;
    public UIntPtr ProcessMemoryLimit;
    public UIntPtr JobMemoryLimit;
    public UIntPtr PeakProcessMemoryUsed;
    public UIntPtr PeakJobMemoryUsed;
  }
  [DllImport("kernel32.dll", SetLastError = true, CharSet = CharSet.Unicode)]
  public static extern IntPtr CreateJobObjectW(IntPtr a, string name);
  [DllImport("kernel32.dll", SetLastError = true)]
  public static extern bool SetInformationJobObject(IntPtr h, int cls, IntPtr lp, uint len);
  [DllImport("kernel32.dll", SetLastError = true)]
  public static extern bool AssignProcessToJobObject(IntPtr h, IntPtr p);
  [DllImport("kernel32.dll")]
  public static extern IntPtr GetCurrentProcess();
  [DllImport("kernel32.dll", SetLastError = true)]
  public static extern bool IsProcessInJob(IntPtr p, IntPtr job, out bool result);
}
"@

$job = [CopperlineJobMem]::CreateJobObjectW([IntPtr]::Zero, $null)
if ($job -eq [IntPtr]::Zero) {
    Write-Error "CreateJobObject failed"
    exit 3
}
$ext = New-Object CopperlineJobMem+EXT
# JOB_OBJECT_LIMIT_PROCESS_MEMORY (0x100) | JOB_OBJECT_LIMIT_JOB_MEMORY (0x200)
$ext.Basic.LimitFlags = 0x300
$lim = [UIntPtr]::new([int64]$LimitMB * 1MB)
$ext.ProcessMemoryLimit = $lim
$ext.JobMemoryLimit = $lim
$len = [Runtime.InteropServices.Marshal]::SizeOf($ext)
$buf = [Runtime.InteropServices.Marshal]::AllocHGlobal($len)
[Runtime.InteropServices.Marshal]::StructureToPtr($ext, $buf, $false)
# JobObjectExtendedLimitInformation = 9
if (-not [CopperlineJobMem]::SetInformationJobObject($job, 9, $buf, [uint32]$len)) {
    $err = [Runtime.InteropServices.Marshal]::GetLastWin32Error()
    Write-Error "SetInformationJobObject failed: $([ComponentModel.Win32Exception]::new($err).Message)"
    exit 3
}

$sw = [Diagnostics.Stopwatch]::StartNew()
# Put THIS process in the job before spawning: child processes inherit job
# membership, so the limit binds from the first allocation. Assigning after
# Start-Process silently failed before (the router reached 4.4GB under a 1GB
# limit) because the returned handle lacked PROCESS_SET_QUOTA.
$self = [CopperlineJobMem]::GetCurrentProcess()
if (-not [CopperlineJobMem]::AssignProcessToJobObject($job, $self)) {
    $err = [Runtime.InteropServices.Marshal]::GetLastWin32Error()
    Write-Error "AssignProcessToJobObject(self) failed: $([ComponentModel.Win32Exception]::new($err).Message); refusing to run uncapped"
    exit 3
}
$inJob = $false
if (-not [CopperlineJobMem]::IsProcessInJob($self, $job, [ref]$inJob) -or -not $inJob) {
    Write-Error "job membership not established; refusing to run uncapped"
    exit 3
}
# Start-Process joins -ArgumentList with spaces, so quote any argument that
# contains whitespace (board/report paths often live under "C:\Users\...\").
$argv = @()
foreach ($a in $RouterArgs) {
    if ($a -match '\s') { $argv += ('"' + $a + '"') } else { $argv += $a }
}
$p = Start-Process -FilePath $Exe -ArgumentList $argv -PassThru -NoNewWindow
$peak = 0
while (-not $p.HasExited) {
    try {
        $p.Refresh()
        if ($p.WorkingSet64 -gt $peak) { $peak = $p.WorkingSet64 }
    } catch {}
    Start-Sleep -Milliseconds 100
}
$p.WaitForExit()
$sw.Stop()
Write-Host ("CAPPED_RUN exit={0} peak_rss_mb={1} wall_s={2} limit_mb={3}" -f $p.ExitCode,
    [math]::Round($peak / 1MB, 1),
    [math]::Round($sw.Elapsed.TotalSeconds, 2), $LimitMB)
exit $p.ExitCode
