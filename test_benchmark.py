"""End-to-end checks for solution.py; public maps run in fresh processes.

Run: python -m unittest test_benchmark -v
The solver has 120 seconds/map. A 150-second outer watchdog includes startup
and validation. This file is a test harness, not part of the submitted solver.
"""
import copy
import json
from pathlib import Path
import subprocess
import sys
import time
import unittest
from unittest.mock import patch

import solution as solver
from sokoban import PROBLEMS
from validation import signature, validate_solution

ROOT = Path(__file__).resolve().parent
BUDGET = 120.0


def run_public_case(case):
    initial = copy.deepcopy(PROBLEMS[case-1])
    reference = copy.deepcopy(initial)
    before = (signature(initial), initial.action, initial.gval, initial.parent)
    start = time.perf_counter()
    final = solver.solve(initial, BUDGET)
    elapsed = time.perf_counter()-start
    valid, detail = (False,'Returned False') if final is False else validate_solution(reference,final)
    unchanged = before == (signature(initial),initial.action,initial.gval,initial.parent)
    return dict(case=case,valid=valid,detail=detail,unchanged=unchanged,
                steps=final.gval if valid else None,seconds=elapsed)


class SolverBenchmarkTests(unittest.TestCase):
    def test_public_maps_in_fresh_processes(self):
        for case in range(1,len(PROBLEMS)+1):
            with self.subTest(map=case):
                run = subprocess.run([sys.executable,str(Path(__file__).resolve()),
                    '--case',str(case)],cwd=ROOT,capture_output=True,text=True,timeout=BUDGET+30)
                self.assertEqual(run.returncode,0,run.stderr)
                result=json.loads(run.stdout.strip().splitlines()[-1])
                self.assertTrue(result['valid'],result['detail'])
                self.assertTrue(result['unchanged'],'Input was modified')
                self.assertLessEqual(result['seconds'],BUDGET,'solve exceeded its budget')
                print(f"Map {case:02}: PASS, steps={result['steps']}, solve={result['seconds']:.3f}s",flush=True)

    def test_improvement_interruption_keeps_first_solution(self):
        original=solver._search
        for error in [solver._Timeout,solver._SearchLimit]:
            interruptions=[]
            def search(*args,**kwargs):
                if len(args)>5:
                    interruptions.append(error)
                    raise error
                return original(*args,**kwargs)
            state=copy.deepcopy(PROBLEMS[0]); reference=copy.deepcopy(state)
            with self.subTest(error=error),patch.object(solver,'_search',side_effect=search):
                result=solver.solve(state,1)
            self.assertTrue(interruptions, 'Improvement interruption was not exercised')
            self.assertIsNot(result,False)
            self.assertTrue(validate_solution(reference,result)[0])

    def test_interrupted_reconstruction_keeps_previous_path(self):
        original=solver._reconstruct
        calls=[]
        def reconstruct(*args):
            calls.append(1)
            if len(calls)>1:
                raise solver._Timeout
            return original(*args)
        state=copy.deepcopy(PROBLEMS[0]);reference=copy.deepcopy(state)
        with patch.object(solver,'_reconstruct',side_effect=reconstruct):
            result=solver.solve(state,1)
        self.assertGreater(len(calls),1)
        self.assertTrue(validate_solution(reference,result)[0])

    def test_zero_budget(self):
        self.assertIs(solver.solve(copy.deepcopy(PROBLEMS[0]),0),False)


if __name__ == '__main__':
    if len(sys.argv)==3 and sys.argv[1]=='--case':
        print(json.dumps(run_public_case(int(sys.argv[2]))),flush=True)
    else:
        unittest.main(verbosity=2)
