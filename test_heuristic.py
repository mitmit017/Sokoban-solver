"""
TEST TAY DANH CHO THANH VIEN 2 — Kiem tra ham heuristic().

Cach chay: python test_heuristic.py
Ket qua mong doi: tat ca PASS.

Nguyen tac kiem tra admissible:
    h(state) <= so buoc toi uu thuc te
    => h(state) <= 0 neu da o dich
    => h(state) > 0 neu chua o dich
"""
import sys
from sokoban import SokobanState
from solution import heuristic, precompute


def make_state(robots, boxes, storage, width=6, height=6, obstacles=()):
    return SokobanState(
        "TEST", 0, None, width, height,
        tuple(robots), frozenset(boxes), frozenset(storage), frozenset(obstacles)
    )


def check(desc, state, expected_h, relation="=="):
    """
    Kiem tra gia tri heuristic voi ky vong.
    relation: '==' (chinh xac), '<=' (khong qua lon), '>0' (duong)
    """
    precomputed = precompute(state)
    h = heuristic(state, precomputed["dist_table"])
    if relation == "==":
        ok = (h == expected_h)
    elif relation == "<=":
        ok = (h <= expected_h)
    elif relation == ">0":
        ok = (h > 0)
    else:
        ok = False

    status = "PASS" if ok else "FAIL"
    print(f"[{status}] {desc}")
    print(f"        h = {h}, ky vong {relation} {expected_h}")
    if not ok:
        sys.exit(1)


print("=" * 60)
print("KIEM TRA HEURISTIC — Thanh vien 2")
print("=" * 60)

# ----------------------------------------------------------
# TC1: Thung da o dich => h phai = 0
# ----------------------------------------------------------
s1 = make_state(
    robots=[(0, 0)],
    boxes=[(2, 2)],
    storage=[(2, 2)],  # Thung va dich trung nhau
)
check("TC1: Thung da o dich → h = 0", s1, 0, "==")

# ----------------------------------------------------------
# TC2: 1 thung, 1 dich, khoang cach Manhattan = 1
# ----------------------------------------------------------
s2 = make_state(
    robots=[(0, 0)],
    boxes=[(1, 0)],
    storage=[(2, 0)],  # Cach 1 buoc sang phai
)
check("TC2: 1 thung, khoang cach 1 → h = 1", s2, 1, "==")

# ----------------------------------------------------------
# TC3: 1 thung, 1 dich, khoang cach Manhattan = 3
# ----------------------------------------------------------
s3 = make_state(
    robots=[(0, 0)],
    boxes=[(0, 0)],
    storage=[(2, 1)],  # Cach 2 ngang + 1 doc = 3
)
check("TC3: 1 thung, khoang cach 3 → h = 3", s3, 3, "==")

# ----------------------------------------------------------
# TC4: 2 thung, moi thung gan 1 dich khac nhau
# ----------------------------------------------------------
s4 = make_state(
    robots=[(0, 0)],
    boxes=[(1, 0), (3, 0)],
    storage=[(2, 0), (4, 0)],  # Moi thung cach dich gan nhat 1 buoc
)
check("TC4: 2 thung × 1 buoc = tong h = 2", s4, 2, "==")

# ----------------------------------------------------------
# TC5: Admissibility — h phai <= so buoc thuc te (= 1)
# Truong hop don gian: thung (1,0) -> dich (2,0), robot day sang phai 1 buoc
# ----------------------------------------------------------
s5 = make_state(
    robots=[(0, 0)],
    boxes=[(1, 0)],
    storage=[(2, 0)],
)
check("TC5: Admissible — h <= 1 (so buoc thuc te)", s5, 1, "<=")

# ----------------------------------------------------------
# TC6: Nhieu dich hon thung — h chi tinh thung chua o dich
# ----------------------------------------------------------
s6 = make_state(
    robots=[(0, 0)],
    boxes=[(1, 1)],
    storage=[(1, 1), (3, 3), (5, 5)],  # 3 dich, 1 thung da o dich
)
check("TC6: Thung da o dich (co nhieu dich hon thung) → h = 0", s6, 0, "==")

# ----------------------------------------------------------
# TC7: Co tuong chan giua thung va dich — test admissible voi obstacles
#
# Ban co 5x3:
#   . . . . .
#   $ # . . *   <- thung (0,1), tuong (1,1), dich (4,1)
#   . . . . .
#
# Manhattan tu thung (0,1) -> dich (4,1) = 4
# Thuc te phai di vong: len (0,0), sang phai 4 buoc, xuong (4,1) = 6 buoc
# Admissible: h <= 6 thuc te. Manhattan tra 4 <= 6 → VAN admissible.
# Nhung neu dung BFS muc 2: se tra 6 (chinh xac hon).
# ----------------------------------------------------------
s7 = make_state(
    robots=[(3, 0)],
    boxes=[(0, 1)],
    storage=[(4, 1)],
    obstacles=[(1, 1)],   # Tuong chan ngang, buoc phai di vong
    width=5, height=3,
)
check("TC7: Tuong chan giua, h_manhattan <= h_thuc_te=6 (admissible)", s7, 6, "<=")

# ----------------------------------------------------------
# TC8: Kiem tra h > 0 khi thung CHUA o dich — du co obstacles
# Bat buoc: ham khong duoc tra 0 (gia su da xong) khi thung van con xa dich
# ----------------------------------------------------------
s8 = make_state(
    robots=[(0, 0)],
    boxes=[(0, 2)],
    storage=[(4, 2)],
    obstacles=[(2, 1), (2, 2), (2, 3)],  # Tuong doc chan ngang ban co
    width=5, height=5,
)
check("TC8: Thung chua o dich (du co tuong) → h > 0", s8, 0, ">0")

# ----------------------------------------------------------
# TC_LEVEL: Test phan biet Muc 1 (Manhattan) vs Muc 2 (BFS thuc)
#
# Map 5x5, robot (0,0), storage (4,0)
# Thung tai (2,0), tuong chan truc tiep: obstacles=[(3,0)]
#
#   R . $ # *     (R=robot, $=thung, #=tuong, *=dich)
#   . . . . .
#   . . . . .
#
# Manhattan(2,0 -> 4,0) = 2  (khong biet tuong)
# BFS thuc: phai di vong qua (2,1)->(3,1)->(4,1)->(4,0) = 4
#
# MUC 1 (hien tai): h = 2  → admissible nhung uoc luong kem
# MUC 2 (sau nang cap): h = 4 → chinh xac hon, A* chay nhanh hon nhieu
# ----------------------------------------------------------
s_level = make_state(
    robots=[(0, 0)],
    boxes=[(2, 0)],
    storage=[(4, 0)],
    obstacles=[(3, 0)],   # Tuong chan thang giua thung va dich
    width=5, height=3,
)
precomputed_level = precompute(s_level)
h_level = heuristic(s_level, precomputed_level["dist_table"])

print()
print("[PHAN BIET MUC 1 vs MUC 2] Map co tuong chan giua thung va dich:")
print(f"  Manhattan thuc = 2  (khong tinh tuong)")
print(f"  BFS thuc       = 4  (phai di vong qua tuong)")
print(f"  heuristic() dang tra = {h_level}")
if h_level == 2:
    print("  → Dang o MUC 1 (Manhattan). Nang len Muc 2 de dat h=4.")
elif h_level == 4:
    print("  → Da o MUC 2 (BFS thuc)! PASS.")
else:
    print(f"  → Ket qua la {h_level} — kiem tra lai logic.")

# Bat buoc: h phai admissible (<= khoang cach thuc te la 4)
check("TC_LEVEL: Admissible du o muc nao (h <= 4 = khoang cach thuc)", s_level, 4, "<=")

print()
print("Tat ca test PASS! Ham heuristic() hoat dong dung.")


