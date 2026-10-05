"""
Benchmark script — do Thanh vien 4 phu trach.
Chay solver tren 20 map trong PROBLEMS, do thoi gian va so buoc.
File nay KHONG duoc dua vao solution.py khi nop bai.

Cach chay: python benchmark.py
Cach chay nhanh (timebound nho hon): python benchmark.py --time 10
"""
import sys
import time
from sokoban import PROBLEMS
from solution2 import solve
from validation import validate_solution



# --- Cau hinh ---
TIMEBOUND = int(sys.argv[2]) if len(sys.argv) > 2 and sys.argv[1] == "--time" else 120



def run_benchmark():
    results = []

    print(f"Benchmark Sokoban Solver — timebound={TIMEBOUND}s / map")
    print("=" * 65)
    print(f"{'Map':<5} {'Ket qua':<14} {'Buoc':<8} {'Thoi gian':<12} {'Hop le?'}")
    print("-" * 65)

    for i, initial_state in enumerate(PROBLEMS):
        start = time.time()

        try:
            final = solve(initial_state, timebound=TIMEBOUND)
            elapsed = time.time() - start

            if final is False:
                # Kiem tra xem la timeout hay vo nghiem thuc su
                if elapsed >= TIMEBOUND - 1:
                    status = "TIMEOUT"
                else:
                    status = "NO_SOL"
                steps = "-"
                valid_str = "-"

            else:
                elapsed_solve = time.time() - start
                ok, msg = validate_solution(initial_state, final)
                elapsed = time.time() - start

                if ok:
                    status = "PASS"
                    steps = str(final.gval)
                    valid_str = "OK"
                else:
                    status = "INVALID"
                    steps = str(final.gval)
                    valid_str = msg[:30]

        except Exception as e:
            elapsed = time.time() - start
            status = "CRASH"
            steps = "-"
            valid_str = str(e)[:30]

        print(f"{i+1:<5} {status:<14} {steps:<8} {elapsed:.3f}s       {valid_str}")
        results.append({
            "map": i + 1,
            "status": status,
            "steps": steps,
            "time_s": round(elapsed, 3),
            "valid": valid_str,
        })

    # --- Tong ket ---
    print("=" * 65)
    passed  = sum(1 for r in results if r["status"] == "PASS")
    timeout = sum(1 for r in results if r["status"] == "TIMEOUT")
    crashed = sum(1 for r in results if r["status"] == "CRASH")
    invalid = sum(1 for r in results if r["status"] == "INVALID")
    total_t = sum(r["time_s"] for r in results)

    print(f"PASS:    {passed}/{len(PROBLEMS)}")
    print(f"TIMEOUT: {timeout}")
    print(f"NO_SOL:  {sum(1 for r in results if r['status'] == 'NO_SOL')}")
    print(f"CRASH:   {crashed}")
    print(f"INVALID: {invalid}")
    print(f"Tong thoi gian: {total_t:.2f}s")

    # Trung binh so buoc cac map giai duoc
    solved_steps = [int(r["steps"]) for r in results if r["steps"].isdigit()]
    if solved_steps:
        print(f"So buoc trung binh: {sum(solved_steps)/len(solved_steps):.1f}")
        print(f"So buoc max:        {max(solved_steps)}")

    return results


if __name__ == "__main__":
    run_benchmark()
