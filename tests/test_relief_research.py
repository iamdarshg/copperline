#!/usr/bin/env python3
"""Focused standalone regression checks for relief research calculations."""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(__file__)), "em"))
import research_relief as rr

def main():
    a=rr.run_case(2,100e6,2,.25,.5,1)
    b=rr.run_case(2,100e6,8,.25,.5,1)
    assert b["relief_current_derating"] > a["relief_current_derating"]
    assert a["relief"]["junction_c"] > a["solid"]["junction_c"]
    low=rr.run_case(1,0,4,.25,.5,1)
    high=rr.run_case(10,0,4,.25,.5,1)
    assert high["relief"]["rise_c"] > low["relief"]["rise_c"]
    assert len(rr.sweep()) == 4*4*4*4*3*3
    assert all("unacceptable" in row for row in rr.sweep()[:8])
    assert rr.connection_resistance("relief",1,.5,.25,4,25,100e6) > 0
    assert rr.connection_resistance("solid",1,.5,.25,4,25,0) > 0
    print("relief research: 7 assertions passed")
if __name__ == "__main__": main()
