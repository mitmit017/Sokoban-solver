"""
TEST TAY DANH CHO THANH VIEN 3 — Kiem tra is_deadlock() va _compute_dead_cells().

Cach chay: python test_deadlock.py
Ket qua mong doi: tat ca PASS.

Nguyen tac kiem tra:
    - Deadlock THAT (true positive) : is_deadlock() phai tra True
    - Khong deadlock (true negative): is_deadlock() phai tra False
    TUYET DOI tranh false positive: trang thai giai duoc ma bi danh dau deadlock!
"""
import sys
from sokoban import SokobanState, sokoban_goal_state
from solution import is_deadlock, precompute, _compute_dead_cells, solve
from validation import validate_solution


def make_state(robots, boxes, storage, width=6, height=6, obstacles=()):
    return SokobanState(
        "TEST", 0, None, width, height,
        tuple(robots), frozenset(boxes), frozenset(storage), frozenset(obstacles)
    )


def check(desc, state, expect_deadlock):
    precomputed = precompute(state)
    dead_cells = precomputed["dead_cells"]
    result = is_deadlock(state, dead_cells)
    ok = (result == expect_deadlock)
    status = "PASS" if ok else "FAIL"
    print(f"[{status}] {desc}")
    print(f"        is_deadlock = {result}, ky vong = {expect_deadlock}")
    if not ok:
        sys.exit(1)


print("=" * 60)
print("KIEM TRA DEADLOCK DETECTION — Thanh vien 3")
print("=" * 60)

# ----------------------------------------------------------
# TC1: Thung o goc (0,0) khong phai dich → DEADLOCK THAT
# Ban 4x4, goc tren-trai la o chet (2 canh tuong)
# ----------------------------------------------------------
# Ban co:
#   ####
#   #$  #
#   #   #
#   ####
# Thung ($) o (0,0) bi ket goc → deadlock
s1 = make_state(
    robots=[(2, 1)],
    boxes=[(0, 0)],
    storage=[(3, 3)],
    width=4, height=4,
)
check("TC1: Thung o goc (0,0), khong phai dich → DEADLOCK", s1, True)

# ----------------------------------------------------------
# TC2: Thung o goc (0,0) VA la dich → KHONG deadlock
# ----------------------------------------------------------
s2 = make_state(
    robots=[(2, 1)],
    boxes=[(0, 0)],
    storage=[(0, 0)],  # Goc la dich → da thang
    width=4, height=4,
)
check("TC2: Thung o goc (0,0) nhung la dich → KHONG deadlock", s2, False)

# ----------------------------------------------------------
# TC3: Thung o giua ban co → KHONG deadlock
# ----------------------------------------------------------
s3 = make_state(
    robots=[(0, 0)],
    boxes=[(2, 2)],
    storage=[(0, 0)],
    width=5, height=5,
)
check("TC3: Thung o giua ban co → KHONG deadlock", s3, False)

# ----------------------------------------------------------
# TC4: Thung sat tuong tren (y=0), khong co dich nao tren hang y=0 → DEADLOCK
# Ban 5x5, storage o hang y=4 (day ban) → thung bi day len tren la mat
# ----------------------------------------------------------
s4 = make_state(
    robots=[(2, 1)],
    boxes=[(2, 0)],    # Thung sat mep tren, khong the day len nua
    storage=[(0, 4), (4, 4)],  # Dich o day ban, hoan toan khac hang
    width=5, height=5,
)
check("TC4: Thung sat mep tren, mep tren khong co dich → DEADLOCK", s4, True)

# ----------------------------------------------------------
# TC5: Kiem tra dam bao khong false positive — map tu public_tests
# Trang thai dau: robot (0,1), thung (1,1), dich (2,1) — CO LỜI GIAI
# is_deadlock PHAI tra False
# ----------------------------------------------------------
s5 = make_state(
    robots=[(0, 1)],
    boxes=[(1, 1)],
    storage=[(2, 1)],
    width=4, height=3,
)
check("TC5: Map tu public_test (co loi giai) → KHONG deadlock", s5, False)

# ----------------------------------------------------------
# TC6: Kiem tra end-to-end — dam bao bat deadlock KHONG lam mat loi giai
# Giai map tu public_tests, ket qua phai la Valid solution
# ----------------------------------------------------------
print()
print("[KIEM TRA END-TO-END] Giai map tu public_tests voi deadlock bat...")
s6 = make_state(
    robots=[(0, 1)],
    boxes=[(1, 1)],
    storage=[(2, 1)],
    width=4, height=3,
)
result = solve(s6, timebound=10)
if result is False:
    print("[FAIL] solve() tra False tren map co loi giai!")
    sys.exit(1)
valid, msg = validate_solution(s6, result)
if not valid:
    print(f"[FAIL] Loi giai khong hop le: {msg}")
    sys.exit(1)
print(f"[PASS] Giai duoc, so buoc = {result.gval}, loi giai hop le.")

# ----------------------------------------------------------
# TC7: Hien thi tap o chet cua ban co sample de TV3 kiem tra bang mat
# ----------------------------------------------------------
print()
print("[THONG TIN] Tap o chet cua map 5x5 (storage tai (0,0) va (4,4)):")
sample = make_state(
    robots=[(2, 2)],
    boxes=[(1, 1)],
    storage=[(0, 0), (4, 4)],
    width=5, height=5,
)
dead = _compute_dead_cells(sample)
all_cells = [(x, y) for x in range(5) for y in range(5)]
print("  Ban co (D=o-chet, .=binh-thuong, *=dich):")
for y in range(5):
    row = ""
    for x in range(5):
        if (x, y) in sample.storage:
            row += "* "
        elif (x, y) in dead:
            row += "D "
        else:
            row += ". "
    print("  " + row)
print(f"  Tong so o chet: {len(dead)} / {5*5} o")

# ----------------------------------------------------------
# TC8: Co vat can (obstacles) — kiem tra dead_cells tinh voi ban co co tuong
#
# Ban co 5x3, tuong doc o x=2 (chan doi):
#   . . # . .
#   . . # . .
#   . . # . .
# Storage tai (4,1). Thung tai (0,1).
# Vi co tuong chan (x=2), thung (0,1) KHONG THE duoc day sang phai qua tuong
# => (0,1) phai la o chet (khong the toi duoc (4,1))
# ----------------------------------------------------------
s8 = make_state(
    robots=[(1, 0)],
    boxes=[(0, 1)],
    storage=[(4, 1)],
    obstacles=[(2, 0), (2, 1), (2, 2)],  # Tuong doc chan doi
    width=5, height=3,
)
check("TC8: Tuong chan hoan toan, thung bi cat khoi dich → DEADLOCK", s8, True)

# ----------------------------------------------------------
# TC9: Deadlock dong — 2 thung tao mau 2x2 (thuan thung, khong co tuong)
#
# Ban co 5x5:
#   . . . . .
#   . $ $ . .   <- thung (1,1) va (2,1)
#   . $ $ . .   <- thung (1,2) va (2,2)
#   . . . . .
#   . . . . *   <- dich (4,4)
#
# 4 thung tao hinh vuong 2x2, khong the di chuyen ra ngoai → deadlock dong
# ----------------------------------------------------------
s9 = make_state(
    robots=[(0, 0)],
    boxes=[(1, 1), (2, 1), (1, 2), (2, 2)],  # 4 thung hinh vuong 2x2
    storage=[(4, 4)],
    width=5, height=5,
)
check("TC9: 4 thung tao mau 2x2 → DEADLOCK dong", s9, True)

# ----------------------------------------------------------
# TC10: Deadlock dong — 1 thung + 3 tuong tao mau 2x2
#
# Ban co 4x4 (tuong bao ngoai tu dong):
#   # # # #
#   # $ # .    <- thung (1,1), tuong (2,1)
#   # # . .    <- tuong (1,2)
#   # . . *
#
# O (1,1) bi vay 3 phia la tuong (trai=tuong bien, tren=tuong bien, phai=(2,1))
# va phia duoi la tuong (1,2) => thung ket trong goc 2x2 voi tuong
# ----------------------------------------------------------
s10 = make_state(
    robots=[(3, 3)],
    boxes=[(1, 1)],
    storage=[(3, 3)],
    obstacles=[(2, 1), (1, 2)],   # Tao mau 2x2: (0,0)=tuong bien, (1,0)=tuong bien
    width=4, height=4,
)
# Luu y: goc tren-trai o (0,0) la tuong bien (khong phai obstacles trong state)
# Deadlock 2x2 tai goc (0,0): (0,0)=tuong, (1,0)=tuong, (0,1)=tuong, (1,1)=thung
# Nhung trong framework, 'obstacles' la tuong BEN TRONG ban co, tuong bien
# duoc xu ly o sokoban.py khi sinh successor (kiem tra x,y out of range).
# => Ham _is_2x2_deadlock chi biet ve obstacles ben trong, khong biet tuong bien.
# => TC10 kiem tra mau 2x2: thung(1,1) + obstacles(2,1) + obstacles(1,2) + o(2,2)
#    o(2,2) = trong => 4 o KHONG phai tat ca bi chiem => KHONG deadlock 2x2
# => Muon ket: can them obstacles(2,2) de tao mau 2x2 hoan chinh
s10b = make_state(
    robots=[(3, 3)],
    boxes=[(1, 1)],
    storage=[(3, 3)],
    obstacles=[(2, 1), (1, 2), (2, 2)],  # Tao mau 2x2 day du: (1,1),(2,1),(1,2),(2,2)
    width=4, height=4,
)
check("TC10: Thung + 3 tuong tao mau 2x2 → DEADLOCK dong", s10b, True)

# ----------------------------------------------------------
# TC11: Mau 2x2 nhung TẤT CA thung da o dich → KHONG deadlock
# (Dat ca 4 thung len 4 dich trung nhau voi 4 o 2x2)
# ----------------------------------------------------------
s11 = make_state(
    robots=[(0, 0)],
    boxes=[(1, 1), (2, 1), (1, 2), (2, 2)],
    storage=[(1, 1), (2, 1), (1, 2), (2, 2)],  # Dich trung voi thung
    width=5, height=5,
)
check("TC11: Mau 2x2 nhung tat ca thung da o dich → KHONG deadlock", s11, False)

# ==============================================================================
# TEST CASES MOI — Thuat toan FREEZE DE QUY (TC12 tro di)
# ==============================================================================

# ----------------------------------------------------------
# TC12: Day 3 thung lien tiep sat mep tren — DEADLOCK
# (Tong quat hoa TC4: truoc chi test 1 thung)
#
# Ban co 6x5:
#   $ $ $ . .   <- 3 thung sat mep tren (y=0)
#   . . . . .
#   . . . . *   <- dich (4,2)
#
# Moi thung: dong cung doc (y=0 la mep tren + bi thung ben canh chan ngang)
# Dong cung ngang: thung (0,0) bi chan phai boi (1,0), (1,0) bi chan trai boi (0,0)
#                  va chan phai boi (2,0), ... tat ca bi dong cung ngang lan nhau
# Dong cung doc: mep tren (y=-1) va phia duoi con trong -> CHUA dong cung doc
# => Nhung thung SAT MEP tren thi chi dong cung doc khi phia duoi cung bi chan
# => Voi y=0, frozen_v chi True neu (x, 1) cung la tuong/thung dong cung
# => 3 thung ngang y=0: frozen_v=True (mep tren + mep duoi khong, nhung...
#    frozen_h: thung giua (1,0) bi (0,0) va (2,0) chan; (0,0) bi mep va (1,0) chan
#    frozen_v: mep tren (y=-1) la tuong; phia duoi (x,1) la trong -> NOT frozen_v
# => TC nay KHONG deadlock theo freeze (chi deadlock tinh qua dead_cells)
# Kiem tra qua dead_cells thay the:
# ----------------------------------------------------------
s12 = make_state(
    robots=[(3, 2)],
    boxes=[(0, 0), (1, 0), (2, 0)],  # 3 thung sat mep tren
    storage=[(4, 2)],
    width=6, height=5,
)
precomp12 = precompute(s12)
# Deadlock tinh (dead_cells) se bat vi mep tren khong co storage
check("TC12: 3 thung sat mep tren, dich o noi khac → DEADLOCK (tinh)", s12, True)

# ----------------------------------------------------------
# TC13: Day 3 thung sat tuong ben trong — DEADLOCK dong (freeze)
#
# Ban co 6x5, co tuong ngang y=0 (toan bo): obstacles y=0
# Tuong o y=1 (ben trong), 3 thung o y=1 sat tuong nay -> freeze
#
#   # # # # # #   y=0: tuong
#   $ $ $ . . .   y=1: 3 thung sat tuong
#   . . . . . .
#   . . . . . *   dich
#
# frozen_v: mep y=-1 la tuong; phia duoi (x,2) la trong -> KHONG frozen_v
# => Dung dead_cells de bat truong hop nay
# (Freeze chi bat khi ca 2 truc bi dong cung)
# ----------------------------------------------------------

# ----------------------------------------------------------
# TC14: Frozen ngang nhung KHONG frozen doc → KHONG deadlock
#
# Ban co 3x3:
#   . . .
#   # $ #   <- thung (1,1), tuong (0,1) va (2,1)
#   . . .
#   . * .   <- dich (1,3)
#
# frozen_h: trai=(0,1)=tuong, phai=(2,1)=tuong → frozen_h=True
# frozen_v: can_push xuong: dest=(1,2) trong, src=(1,0) trong → True → KHONG frozen_v
# Thung co the day XUONG qua (1,2) roi den dich (1,3) → KHONG deadlock
# Dead cells: tu dich (1,3) BFS nguoc co the dat (1,2) roi (1,1) → KHONG chet
# ----------------------------------------------------------
s14 = make_state(
    robots=[(0, 0)],
    boxes=[(1, 1)],
    storage=[(1, 3)],
    obstacles=[(0, 1), (2, 1)],
    width=3, height=4,
)
check("TC14: Frozen ngang nhung con thoat doc (xuong) → KHONG deadlock", s14, False)

# ----------------------------------------------------------
# TC15: 1 thung bi chan doc boi 2 tuong nhung con thoat ngang → KHONG deadlock
#
# Ban co 5x5:
#   . # . . .
#   . $ . . *   <- thung (1,1), tuong (1,0) va (1,2), dich (4,1)
#   . # . . .
#
# frozen_v: tren=(1,0)=tuong, duoi=(1,2)=tuong → frozen_v=True
# frozen_h: can_push phai: dest=(2,1) trong, src=(0,1) trong → True → KHONG frozen_h
# Thung co the day PHAI sang (2,1),(3,1),(4,1)=dich → KHONG deadlock
# (Giong TC16 nhung dat lai la TC15 de doc ro hon)
# ----------------------------------------------------------
s15 = make_state(
    robots=[(0, 1)],
    boxes=[(1, 1)],
    storage=[(4, 1)],
    obstacles=[(1, 0), (1, 2)],
    width=5, height=5,
)
check("TC15: Frozen doc nhung con thoat ngang (phai) → KHONG deadlock", s15, False)

# ----------------------------------------------------------
# TC16: Deadlock dong bang freeze — thung kep ca 2 truc (goc tuong + tuong)
#
# Ban co 5x5:
#   . # . . .
#   # $ . . *   <- thung (1,1), tuong (1,0) tren, tuong (0,1) trai
#   . # . . .
#
# frozen_v: tren=(1,0)=tuong; duoi=(1,2)=tuong → frozen_v=True
# frozen_h: trai=(0,1)=tuong → phai robot tu (2,1) de day trai nhung dest=(0,1)=tuong → False
#            phai=(2,1) trong → robot tu (0,1)=tuong → False (src la tuong)
#   can_push_right (dest=(2,1), src=(0,1)): is_wall(0,1)=True → False
#   can_push_left  (dest=(0,1), src=(2,1)): dest (0,1)=tuong → False
# => frozen_h=True
# DEADLOCK: ca 2 truc bi dong cung!
# ----------------------------------------------------------
s16 = make_state(
    robots=[(3, 3)],
    boxes=[(1, 1)],
    storage=[(4, 1)],
    obstacles=[(1, 0), (0, 1), (1, 2)],
    width=5, height=5,
)
check("TC16: Thung kep tren+trai+duoi (frozen ca 2 truc) → DEADLOCK", s16, True)

# ----------------------------------------------------------
# TC17: FALSE POSITIVE quan trong — thung co 2 tuong doc nhung co the day ngang
#
# Ban co 5x5:
#   . # . . .
#   . $ . . *   <- thung (1,1), chi co tuong (1,0) phia tren
#   . . . . .
#
# frozen_v: tren=(1,0)=tuong; duoi=(1,2)=trong → KHONG frozen_v (co the day xuong)
# frozen_h: trai=(0,1) trong → can day trai. KHONG frozen_h.
# KHONG deadlock: thung co the day xuong (1,2) roi day phai toi (4,2)... nhung
# storage o (4,1) nen phai day (1,1) phai truc tiep: (2,1),(3,1),(4,1)
# Deadlock tinh: tu (4,1) BFS nguoc co the dat (1,1) → KHONG dead cell
# => KHONG deadlock
# ----------------------------------------------------------
s17 = make_state(
    robots=[(0, 2)],
    boxes=[(1, 1)],
    storage=[(4, 1)],
    obstacles=[(1, 0)],
    width=5, height=5,
)
check("TC17: Tuong phia tren nhung con 3 huong thoat → KHONG deadlock", s17, False)

# Thung bi kep ca 4 phia boi tuong ben trong
s17b = make_state(
    robots=[(3, 3)],
    boxes=[(1, 1)],
    storage=[(4, 1)],
    obstacles=[(0, 1), (1, 0), (2, 1), (1, 2)],  # 4 tuong quanh thung
    width=5, height=5,
)
check("TC17b: Thung bao vay 4 phia boi tuong → DEADLOCK", s17b, True)

# ----------------------------------------------------------
# TC18: FALSE POSITIVE nghiem trong — 3 thung lien tiep ngang (L-B-R),
# thung giua (B) bi ket DOC that, nhung 2 thung ben canh (L, R) van con
# duong thoat DOC rieng cua chung -> KHONG phai deadlock that.
#
# Ban co 7x5, hang y=2:
#   # L B R #     <- (0,2)=tuong, L=(1,2), B=(2,2), R=(3,2), (4,2)=tuong
# B: (2,1)=tuong, (2,3)=tuong -> B ket doc THAT (truc tiep)
# L: (1,1) va (1,3) deu TRONG -> L con duong thoat doc
# R: (3,1) va (3,3) deu TRONG -> R con duong thoat doc
#
# Loi SAI truoc day: code coi L, R "dong cung CUNG truc ngang" (vi L co
# tuong (0,2) o xa ben trai, R co tuong (4,2) o xa ben phai) la DU de
# ket luan B bi chan vinh vien ca 2 ben -> SAI, vi L/R van doi duoc theo
# truc doc, roi B moi day ngang duoc vao cho trong.
#
# Dich: storage=(1,3) cho L, (3,1) cho R, (3,2) cho B (o R vua bo trong)
# Loi giai that: day L xuong, day R len, roi day B sang phai vao (3,2).
# ----------------------------------------------------------
s18 = make_state(
    robots=[(6, 4)],
    boxes=[(1, 2), (2, 2), (3, 2)],
    storage=[(1, 3), (3, 1), (3, 2)],
    obstacles=[(0, 2), (4, 2), (2, 1), (2, 3)],
    width=7, height=5,
)
check("TC18: 3 thung lien tiep ngang, 2 thung ngoai con thoat doc → KHONG deadlock", s18, False)

print()
print("[KIEM TRA END-TO-END TC18] Giai thuc te trang thai L-B-R...")
result18 = solve(s18, timebound=15)
if result18 is False:
    print("[FAIL] solve() tra False cho TC18 trong khi co loi giai!")
    sys.exit(1)
valid18, msg18 = validate_solution(s18, result18)
if not valid18:
    print(f"[FAIL] Loi giai TC18 khong hop le: {msg18}")
    sys.exit(1)
print(f"[PASS] TC18 giai duoc, so buoc = {result18.gval}, loi giai hop le.")

print()
print("Tat ca test PASS! Deadlock detection (freeze de quy) hoat dong dung.")