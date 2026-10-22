On Error Resume Next

SET obj = CreateObject("RouteService.RouterObject")

IF Err.Number = 0 THEN
	' try call the GetVersion method that returns a string
	a = MsgBox("RouteService version: " & obj.GetVersion & vbNewLine & "Unregister server now?", vbInformation + vbYesNo, "RouteService Checker")
	IF a = vbYes THEN RegSrv(0)
ELSE
	a = MsgBox("Server not registered." & vbNewLine & "Register server now?", vbCritical + vbYesNo, "RouteService Checker")
	IF a = vbYes THEN RegSrv(1)
	Err.Clear
END IF

SET obj = Nothing
WScript.Quit

' *********************************************************
SUB RegSrv(reg)
SET UAC = CreateObject("Shell.Application")
IF reg = 1 THEN
	'UAC.ShellExecute "_reg_srv.bat", "", "", "runas", 1
	UAC.ShellExecute "RouteService.exe", "/RegServer", "", "runas", 1
ELSE
	'UAC.ShellExecute "_unreg_srv.bat", "", "", "runas", 1
	UAC.ShellExecute "RouteService.exe", "/UnregServer", "", "runas", 1
END IF
SET UAC = Nothing
END SUB
