"""
KIỂM THỬ DEADLOCK CỦA TV3 — bổ sung cho test_deadlock.py (test tay của TV4).

File này chạy được ở MỌI giai đoạn của dự án: chỉ bắt buộc có _compute_dead_cells và
is_deadlock trong solution.py. Các lớp nâng cao (_is_2x2_deadlock, _is_freeze_deadlock) và
precompute là tùy chọn; test nào cần chúng sẽ tự động bị bỏ qua (skip) kèm lý do nếu chưa có.

Cách chạy (từ thư mục gốc của repo):
    python -m unittest test_deadlock_oracle -v          # chạy nhanh (khoảng vài chục giây)
    SOKOBAN_SLOW=1 python -m unittest test_deadlock_oracle -v   # thêm map lớn hơn (chậm hơn)


Điểm mấu chốt cần kiểm chứng: is_deadlock() TUYỆT ĐỐI không được báo nhầm. Do đó ngoài
các test tình huống cụ thể, tệp này có "oracle": duyệt TOÀN BỘ không gian trạng thái của
các map nhỏ bằng chính successors() của framework, tính chính xác trạng thái nào còn giải
được, rồi khẳng định: mọi trạng thái bị đánh dấu deadlock đều thật sự không giải được.

Các nhóm test:
    1. TestDeadCells       — ô chết tĩnh (Lớp 1), có đối chiếu với cách tính độc lập.
    2. TestDeadlockRules   — các tình huống cụ thể cho Lớp 1, 2, 3 và các "bẫy" báo nhầm.
    3. TestInvariants      — tính chất bất biến (không sửa state, trạng thái đích, ...).
    4. TestOracle          — kiểm chứng không báo nhầm trên toàn bộ không gian trạng thái.
    5. TestPerformance     — is_deadlock phải đủ nhanh để gọi trên mỗi successor.
"""
import os
import random
import time
import unittest
from collections import defaultdict, deque
from itertools import combinations

from sokoban import PROBLEMS, SokobanState, sokoban_goal_state
import solution as _solution
from solution import _compute_dead_cells, is_deadlock
from validation import signature

# Phần tùy chọn: chưa có thì test tương ứng được bỏ qua chứ không báo lỗi import.
_is_2x2_deadlock = getattr(_solution, "_is_2x2_deadlock", None)
_is_freeze_deadlock = getattr(_solution, "_is_freeze_deadlock", None)
_precompute = getattr(_solution, "precompute", None)
CO_2X2 = _is_2x2_deadlock is not None
CO_FREEZE = _is_freeze_deadlock is not None


def _precompute_co_dead_cells():
    """precompute có tồn tại và đã trả về khóa "dead_cells" chưa (việc tích hợp)?"""
    if _precompute is None:
        return False
    try:
        return "dead_cells" in _precompute(PROBLEMS[0])
    except Exception:
        return False


CO_PRECOMPUTE_DEAD = _precompute_co_dead_cells()
LY_DO_2X2 = "chưa có _is_2x2_deadlock (lớp của Giai đoạn 4)"
LY_DO_FREEZE = "chưa có _is_freeze_deadlock (lớp của Giai đoạn 4)"
LY_DO_PRECOMPUTE = "precompute chưa có khóa dead_cells (tích hợp ở Giai đoạn 3)"


# Số map ngẫu nhiên mà oracle duyệt hết không gian trạng thái (hạt giống cố định nên
# kết quả lặp lại được; tăng số này nếu muốn kiểm tra kỹ hơn trước khi nộp bài).
SO_MAP_NGAU_NHIEN = 150


# =============================================================================
# HÀM TIỆN ÍCH DÙNG CHUNG CHO CÁC TEST
# =============================================================================
def make_state(robots, boxes, storage, width=6, height=6, obstacles=()):
    """Tạo SokobanState từ các danh sách tọa độ (cùng quy ước với test của TV4)."""
    return SokobanState(
        "TEST", 0, None, width, height,
        tuple(robots), frozenset(boxes), frozenset(storage), frozenset(obstacles))


def with_boxes(state, boxes):
    """Trả về trạng thái mới cùng map nhưng đổi tập thùng (không sửa state gốc)."""
    return SokobanState(
        "TEST", 0, None, state.width, state.height,
        state.robots, frozenset(boxes), state.storage, state.obstacles)


def all_layers(state, dead_cells):
    """Trả về (lớp1, lớp2×2, freeze, tổng); lớp chưa được cài đặt thì coi là False."""
    lop1 = any(b in dead_cells for b in state.boxes)
    lop3 = _is_2x2_deadlock(state) if CO_2X2 else False
    lop2 = _is_freeze_deadlock(state, dead_cells) if CO_FREEZE else False
    return lop1, lop3, lop2, is_deadlock(state, dead_cells)


def reference_dead_cells(state):
    """Cách tính ô chết ĐỘC LẬP với solution.py, dùng để đối chiếu.

    solution.py tính bằng Reverse BFS (kéo ngược từ ô đích). Ở đây tính theo hướng TIẾN:
    một thùng ở ô p có thể được đẩy sang p + d khi cả p + d lẫn p - d (ô robot đứng) đều
    không phải tường. Ô p sống nếu từ p đi theo các cú đẩy như vậy tới được một ô đích.
    """
    w, h, obs = state.width, state.height, state.obstacles

    def is_wall(c):
        return c[0] < 0 or c[1] < 0 or c[0] >= w or c[1] >= h or c in obs

    huong = ((0, -1), (1, 0), (0, 1), (-1, 0))
    cells = [(x, y) for x in range(w) for y in range(h) if (x, y) not in obs]
    chet = set()
    for start in cells:
        seen, queue, song = {start}, deque([start]), False
        while queue:
            p = queue.popleft()
            if p in state.storage:
                song = True
                break
            for dx, dy in huong:
                q, r = (p[0] + dx, p[1] + dy), (p[0] - dx, p[1] - dy)
                if q not in seen and not is_wall(q) and not is_wall(r):
                    seen.add(q)
                    queue.append(q)
        if not song:
            chet.add(start)
    return frozenset(chet)


# -----------------------------------------------------------------------------
# Oracle: duyệt toàn bộ không gian trạng thái
# -----------------------------------------------------------------------------
def explore(initial, max_states=100000):
    """BFS toàn bộ các trạng thái đạt được từ `initial` bằng successors() của framework.

    Trả về (states, preds) hoặc None nếu vượt quá max_states.
        states: {hashable_state: SokobanState}
        preds:  {hashable_state: [các hashable_state tiền nhiệm]}
    """
    states = {initial.hashable_state(): initial}
    preds = defaultdict(list)
    queue = deque([initial])
    while queue:
        s = queue.popleft()
        ks = s.hashable_state()
        for t in s.successors():
            kt = t.hashable_state()
            preds[kt].append(ks)
            if kt not in states:
                if len(states) >= max_states:
                    return None
                states[kt] = t
                queue.append(t)
    return states, preds


def solvable_keys(states, preds):
    """Tập các trạng thái còn giải được = những trạng thái đi được tới một trạng thái đích.

    Tính bằng BFS NGƯỢC trên đồ thị đã duyệt, bắt đầu từ mọi trạng thái đích.
    """
    goals = [k for k, s in states.items() if sokoban_goal_state(s)]
    ok = set(goals)
    queue = deque(goals)
    while queue:
        k = queue.popleft()
        for p in preds.get(k, ()):
            if p not in ok:
                ok.add(p)
                queue.append(p)
    return ok


def kiem_tra_oracle(testcase, initial, ten, max_states=100000):
    """Chạy oracle trên một map; khẳng định không có báo nhầm ở MỌI lớp.

    Trả về dict thống kê để in báo cáo: số trạng thái, số không giải được, số bắt được
    theo từng lớp. Trả về None nếu map quá lớn để duyệt hết.
    """
    kq = explore(initial, max_states)
    if kq is None:
        return None
    states, preds = kq
    ok = solvable_keys(states, preds)
    dead = _compute_dead_cells(initial)

    tk = dict(ten=ten, states=len(states), unsolvable=len(states) - len(ok),
              l1=0, l3=0, l2=0, tong=0, fp=0)
    for k, s in states.items():
        giai_duoc = k in ok
        lop1, lop3, lop2, tong = all_layers(s, dead)
        # Quy ước: tổng = hợp của 3 lớp.
        testcase.assertEqual(tong, lop1 or lop3 or lop2, "is_deadlock phải là hợp của 3 lớp")
        if giai_duoc:
            # Trạng thái giải được: KHÔNG lớp nào được báo deadlock.
            testcase.assertFalse(
                lop1 or lop3 or lop2,
                "BÁO NHẦM trên %s: lớp1=%s 2x2=%s freeze=%s\n%s"
                % (ten, lop1, lop3, lop2, s.state_string()))
        else:
            tk["l1"] += lop1
            tk["l3"] += lop3
            tk["l2"] += lop2
            tk["tong"] += tong
    return tk


def in_thong_ke(danh_sach):
    """In bảng thống kê (dùng làm số liệu cho báo cáo): tỉ lệ bắt được deadlock."""
    print("\n  %-22s %9s %11s %7s %7s %7s %9s" %
          ("Map", "Trạng thái", "Vô nghiệm", "Lớp 1", "2×2", "Freeze", "Bắt được"))
    for t in danh_sach:
        ti_le = 100.0 * t["tong"] / t["unsolvable"] if t["unsolvable"] else 100.0
        print("  %-22s %9d %11d %7d %7d %7d %8.1f%%" %
              (t["ten"], t["states"], t["unsolvable"], t["l1"], t["l3"], t["l2"], ti_le))


def random_state(rng, w, h, so_robot, so_thung, so_dich, so_tuong):
    """Sinh một map ngẫu nhiên hợp lệ (robot, thùng, tường đều không trùng nhau)."""
    cells = [(x, y) for x in range(w) for y in range(h)]
    rng.shuffle(cells)
    tuong = cells[:so_tuong]
    con_lai = cells[so_tuong:]
    robots = con_lai[:so_robot]
    thung = con_lai[so_robot:so_robot + so_thung]
    ho_duoc_dich = [c for c in cells if c not in tuong]
    dich = rng.sample(ho_duoc_dich, so_dich)
    return make_state(robots, thung, dich, w, h, tuong)


def uoc_luong_so_trang_thai(free, so_robot, so_thung):
    """Ước lượng cận trên số trạng thái để bỏ qua map ngẫu nhiên quá lớn."""
    tong = 1
    for i in range(so_robot):
        tong *= (free - i)
    tong2 = 1
    for i in range(so_thung):
        tong2 *= (free - so_robot - i)
    for i in range(1, so_thung + 1):
        tong2 //= i
    return tong * tong2


# =============================================================================
# 1. Ô CHẾT TĨNH (LỚP 1)
# =============================================================================
class TestDeadCells(unittest.TestCase):

    def test_open_board_center_storage(self):
        """Bàn 5×5 trống, đích ở giữa: ô chết là toàn bộ 16 ô ngoài rìa."""
        s = make_state([(2, 0)], [(1, 1)], [(2, 2)], 5, 5)
        dead = _compute_dead_cells(s)
        vien = {(x, y) for x in range(5) for y in range(5) if x in (0, 4) or y in (0, 4)}
        self.assertEqual(dead, vien)

    def test_corner_with_storage_is_alive(self):
        """Góc có ô đích KHÔNG phải ô chết (luật 'góc luôn chết' sẽ sai ở đây)."""
        s = make_state([(2, 1)], [(1, 1)], [(0, 0)], 4, 4)
        dead = _compute_dead_cells(s)
        self.assertNotIn((0, 0), dead)
        self.assertIn((3, 3), dead)  # góc đối diện không có đích -> chết

    def test_storage_cells_never_dead(self):
        """Mọi ô đích của cả 20 map công khai đều không phải ô chết."""
        for i, p in enumerate(PROBLEMS, 1):
            dead = _compute_dead_cells(p)
            self.assertFalse(dead & p.storage, "Map %d có ô đích bị coi là chết" % i)

    def test_walls_never_dead(self):
        """Ô chết chỉ gồm ô trống; tường (obstacles) không nằm trong tập ô chết."""
        for p in PROBLEMS:
            self.assertFalse(_compute_dead_cells(p) & p.obstacles)

    def test_wall_splits_board(self):
        """Tường chia đôi bàn: phía không có đích hoàn toàn là ô chết."""
        s = make_state([(1, 0)], [(0, 1)], [(4, 1)], 5, 3,
                       obstacles=[(2, 0), (2, 1), (2, 2)])
        dead = _compute_dead_cells(s)
        for y in range(3):
            for x in (0, 1):
                self.assertIn((x, y), dead)

    def test_return_type_is_frozenset(self):
        self.assertIsInstance(_compute_dead_cells(PROBLEMS[0]), frozenset)

    def test_matches_independent_reference_on_public_maps(self):
        """Reverse BFS phải cho đúng kết quả của cách tính TIẾN độc lập (20 map công khai)."""
        for i, p in enumerate(PROBLEMS, 1):
            self.assertEqual(_compute_dead_cells(p), reference_dead_cells(p),
                             "Map %d: ô chết khác với cách tính độc lập" % i)

    def test_matches_independent_reference_on_random_maps(self):
        """Như trên nhưng với 300 map ngẫu nhiên (tường, đích nhiều hơn thùng, ...)."""
        rng = random.Random(20261006)
        for _ in range(300):
            w, h = rng.randint(3, 7), rng.randint(3, 7)
            so_tuong = rng.randint(0, (w * h) // 4)
            so_dich = rng.randint(1, 4)
            # Cần chỗ cho 1 robot + 1 thùng ngoài các ô tường.
            s = random_state(rng, w, h, 1, 1, so_dich, so_tuong)
            self.assertEqual(_compute_dead_cells(s), reference_dead_cells(s), s.state_string())


# =============================================================================
# 2. CÁC TÌNH HUỐNG CỤ THỂ CỦA TỪNG LỚP
# =============================================================================
class TestDeadlockRules(unittest.TestCase):

    def _kiem_tra(self, state):
        return all_layers(state, _compute_dead_cells(state))

    # ---- Lớp 1 ----
    def test_box_in_dead_corner(self):
        s = make_state([(2, 1)], [(0, 0)], [(3, 3)], 4, 4)
        self.assertTrue(self._kiem_tra(s)[0])
        self.assertTrue(self._kiem_tra(s)[3])

    def test_box_on_dead_edge_without_storage_on_that_edge(self):
        """Thùng sát mép trên mà mép trên không có đích -> không bao giờ rời mép được."""
        s = make_state([(2, 1)], [(2, 0)], [(0, 4), (4, 4)], 5, 5)
        self.assertTrue(self._kiem_tra(s)[3])

    def test_box_on_edge_with_storage_on_that_edge_is_ok(self):
        """Mép có đích thì thùng sát mép vẫn giải được."""
        s = make_state([(2, 1)], [(2, 0)], [(0, 0)], 5, 5)
        self.assertFalse(self._kiem_tra(s)[3])

    # ---- Lớp 3: mẫu 2×2 ----
    @unittest.skipUnless(CO_2X2 and CO_FREEZE, LY_DO_2X2)
    def test_2x2_four_boxes(self):
        s = make_state([(0, 0)], [(1, 1), (2, 1), (1, 2), (2, 2)], [(4, 4)], 5, 5)
        _, lop3, lop2, tong = self._kiem_tra(s)
        self.assertTrue(lop3)
        self.assertTrue(lop2, "2×2 là trường hợp riêng của freeze nên freeze cũng phải báo")
        self.assertTrue(tong)

    def test_2x2_all_on_storage_is_not_deadlock(self):
        d = [(1, 1), (2, 1), (1, 2), (2, 2)]
        s = make_state([(0, 0)], d, d, 5, 5)
        self.assertEqual(self._kiem_tra(s), (False, False, False, False))

    @unittest.skipUnless(CO_2X2 or CO_FREEZE, LY_DO_2X2)
    def test_2x2_three_boxes_one_on_storage(self):
        """Khối 2×2 mà chỉ MỘT thùng nằm ngoài đích vẫn là deadlock (vì nó kẹt cứng)."""
        boxes = [(1, 1), (2, 1), (1, 2), (2, 2)]
        s = make_state([(0, 0)], boxes, [(1, 1), (2, 1), (1, 2), (4, 4)], 5, 5)
        self.assertTrue(self._kiem_tra(s)[3])

    @unittest.skipUnless(CO_2X2, LY_DO_2X2)
    def test_2x2_box_with_walls(self):
        """1 thùng + 3 tường tạo thành khối 2×2."""
        s = make_state([(3, 3)], [(1, 1)], [(3, 3)], 4, 4,
                       obstacles=[(2, 1), (1, 2), (2, 2)])
        self.assertTrue(self._kiem_tra(s)[1])

    # ---- Lớp 2: freeze đệ quy ----
    def _map_freeze_thuan(self):
        """Hai thùng cạnh nhau: A(2,2) có tường phía trên, B(3,2) có tường phía dưới.

        Không phải mẫu 2×2 (ô (3,1) trống) và cả hai ô đều 'sống' về mặt tĩnh, nên
        chỉ Lớp 2 (freeze) bắt được. Thực tế: A không đẩy ngang được vì B chặn và robot
        không đứng được vào ô của B; A không đẩy dọc được vì tường; B tương tự.
        """
        return make_state([(7, 0)], [(2, 2), (3, 2)], [(5, 2), (6, 2)], 8, 5,
                          obstacles=[(2, 1), (3, 3)])

    @unittest.skipUnless(CO_FREEZE, LY_DO_FREEZE)
    def test_pure_freeze_only_layer2_catches(self):
        s = self._map_freeze_thuan()
        lop1, lop3, lop2, tong = self._kiem_tra(s)
        self.assertFalse(lop1, "map này được thiết kế để Lớp 1 không bắt được")
        self.assertFalse(lop3, "map này được thiết kế để mẫu 2×2 không bắt được")
        self.assertTrue(lop2)
        self.assertTrue(tong)

    def test_pure_freeze_is_really_unsolvable(self):
        """Đối chứng bằng oracle: map freeze thuần ở trên thật sự vô nghiệm."""
        s = self._map_freeze_thuan()
        states, preds = explore(s)
        self.assertNotIn(s.hashable_state(), solvable_keys(states, preds))

    @unittest.skipUnless(CO_FREEZE, LY_DO_FREEZE)
    def test_freeze_chain_through_box_on_storage(self):
        """Thùng đã kẹt TRÊN ô đích vẫn chặn thùng kề: B sát S ở mép trên không thoát được."""
        # S nằm ở góc (0,0) là ô đích; B ở (1,0); đích còn lại ở (5,0) xa phía bên phải.
        s = make_state([(3, 2)], [(0, 0), (1, 0)], [(0, 0), (5, 0)], 6, 3)
        lop1, _, lop2, tong = self._kiem_tra(s)
        self.assertFalse(lop1, "ô (1,0) còn 'sống' về mặt tĩnh")
        self.assertTrue(lop2)
        self.assertTrue(tong)

    def test_wall_on_one_axis_only_is_not_frozen(self):
        """Chỉ bị chặn một trục (TC14/TC15 của TV4, thêm biến thể)."""
        s = make_state([(0, 0)], [(1, 1)], [(1, 3)], 3, 4, obstacles=[(0, 1), (2, 1)])
        self.assertFalse(self._kiem_tra(s)[3])

    def test_three_boxes_in_row_with_escape_routes(self):
        """TC18 của TV4: thùng giữa kẹt dọc nhưng hai thùng bên cạnh còn đường thoát dọc."""
        s = make_state([(6, 4)], [(1, 2), (2, 2), (3, 2)], [(1, 3), (3, 1), (3, 2)], 7, 5,
                       obstacles=[(0, 2), (4, 2), (2, 1), (2, 3)])
        self.assertFalse(self._kiem_tra(s)[3])

    # ---- Các "bẫy" báo nhầm ----
    def test_robot_is_not_an_obstacle(self):
        """Robot đứng hai bên thùng KHÔNG làm thùng bị kẹt (robot có thể đi chỗ khác)."""
        s = make_state([(1, 2), (3, 2)], [(2, 2)], [(4, 2)], 5, 5, obstacles=[(2, 1)])
        self.assertFalse(self._kiem_tra(s)[3])

    def test_robot_blocking_cells_around_box_in_open_field(self):
        """Bốn robot bao quanh một thùng giữa bàn trống không phải deadlock."""
        s = make_state([(1, 2), (3, 2), (2, 1), (2, 3)], [(2, 2)], [(0, 0)], 5, 5)
        self.assertFalse(self._kiem_tra(s)[3])

    def test_frozen_box_on_storage_alone_is_fine(self):
        """Thùng kẹt cứng nhưng đã nằm đúng ô đích: không phải deadlock."""
        s = make_state([(3, 3)], [(0, 0)], [(0, 0)], 4, 4)
        self.assertFalse(self._kiem_tra(s)[3])

    def test_more_storage_than_boxes(self):
        """Nhiều đích hơn thùng: không được giả định số đích bằng số thùng."""
        s = make_state([(0, 0)], [(2, 2)], [(0, 3), (3, 0), (3, 3), (0, 4)], 5, 5)
        self.assertFalse(self._kiem_tra(s)[3])

    def test_board_border_counts_as_wall(self):
        """Biên bàn chơi là tường: hai thùng sát nhau ở góc có đích khác vẫn kẹt đúng."""
        # Hai thùng (0,0) và (1,0) ở góc trên-trái, đích chỉ ở (4,4): (0,0) là ô chết.
        s = make_state([(2, 2)], [(0, 0), (1, 0)], [(4, 4)], 5, 5)
        self.assertTrue(self._kiem_tra(s)[3])


# =============================================================================
# 3. TÍNH CHẤT BẤT BIẾN
# =============================================================================
class TestInvariants(unittest.TestCase):

    def test_initial_states_of_public_maps_not_deadlock(self):
        """Trạng thái đầu của cả 20 map công khai đều giải được -> không được báo deadlock."""
        for i, p in enumerate(PROBLEMS, 1):
            dead = _compute_dead_cells(p)
            self.assertFalse(is_deadlock(p, dead), "Map %d bị báo deadlock ngay từ đầu" % i)
            self.assertEqual(all_layers(p, dead)[:3], (False, False, False))

    def test_goal_states_never_deadlock(self):
        """Trạng thái đích (mọi thùng trên ô đích) KHÔNG BAO GIỜ là deadlock."""
        rng = random.Random(7)
        for p in PROBLEMS:
            dead = _compute_dead_cells(p)
            dich = sorted(p.storage)
            for _ in range(30):
                boxes = rng.sample(dich, len(p.boxes))
                s = with_boxes(p, boxes)
                self.assertTrue(sokoban_goal_state(s))
                self.assertFalse(is_deadlock(s, dead))

    def test_does_not_mutate_state(self):
        """Gọi is_deadlock không được thay đổi state (đề bài cấm sửa trạng thái đầu vào)."""
        for p in PROBLEMS:
            truoc = (signature(p), p.action, p.gval, p.parent)
            is_deadlock(p)
            is_deadlock(p, _compute_dead_cells(p))
            sau = (signature(p), p.action, p.gval, p.parent)
            self.assertEqual(truoc, sau)

    def test_none_dead_cells_gives_same_answer(self):
        """is_deadlock(state) tự tính ô chết phải cho cùng kết quả với truyền sẵn dead_cells."""
        rng = random.Random(11)
        for p in PROBLEMS:
            dead = _compute_dead_cells(p)
            cells = [(x, y) for x in range(p.width) for y in range(p.height)
                     if (x, y) not in p.obstacles]
            for _ in range(20):
                s = with_boxes(p, rng.sample(cells, len(p.boxes)))
                self.assertEqual(is_deadlock(s), is_deadlock(s, dead))

    @unittest.skipUnless(CO_2X2 and CO_FREEZE, LY_DO_FREEZE)
    def test_2x2_implies_freeze(self):
        """Mẫu 2×2 là trường hợp riêng của freeze: 2×2 báo thì freeze phải báo."""
        rng = random.Random(13)
        so_mau = 0
        for _ in range(4000):
            w, h = rng.randint(3, 6), rng.randint(3, 6)
            so_tuong = rng.randint(0, 4)
            so_thung = rng.randint(2, min(6, w * h - so_tuong - 1))
            s = random_state(rng, w, h, 1, so_thung, rng.randint(1, 3), so_tuong)
            dead = _compute_dead_cells(s)
            if _is_2x2_deadlock(s):
                so_mau += 1
                self.assertTrue(_is_freeze_deadlock(s, dead), s.state_string())
        self.assertGreater(so_mau, 50, "test này phải thực sự gặp đủ nhiều mẫu 2×2")

    @unittest.skipUnless(CO_PRECOMPUTE_DEAD, LY_DO_PRECOMPUTE)
    def test_precompute_dead_cells_matches(self):
        """Khi đã tích hợp: precompute(state)["dead_cells"] phải đúng bằng _compute_dead_cells."""
        for p in PROBLEMS:
            self.assertEqual(_precompute(p)["dead_cells"], _compute_dead_cells(p))

    @unittest.skipUnless(CO_PRECOMPUTE_DEAD, LY_DO_PRECOMPUTE)
    def test_precompute_does_not_mutate_state(self):
        p = PROBLEMS[5]
        truoc = (signature(p), p.action, p.gval, p.parent)
        _precompute(p)
        self.assertEqual(truoc, (signature(p), p.action, p.gval, p.parent))


# =============================================================================
# 4. ORACLE: KHÔNG BÁO NHẦM TRÊN TOÀN BỘ KHÔNG GIAN TRẠNG THÁI
# =============================================================================
class TestOracle(unittest.TestCase):

    def test_public_small_maps(self):
        """Map công khai nhỏ (2, 5, 7, 8): duyệt hết không gian trạng thái, không báo nhầm."""
        tk = []
        for so_map in (2, 5, 7, 8):
            t = kiem_tra_oracle(self, PROBLEMS[so_map - 1], "Map công khai %d" % so_map)
            self.assertIsNotNone(t, "Map %d quá lớn để duyệt hết" % so_map)
            tk.append(t)
        in_thong_ke(tk)

    @unittest.skipUnless(os.environ.get("SOKOBAN_SLOW"),
                         "Chậm (khoảng 10 giây): đặt SOKOBAN_SLOW=1 để chạy")
    def test_public_map_4_slow(self):
        t = kiem_tra_oracle(self, PROBLEMS[3], "Map công khai 4", max_states=200000)  # có ~127.500 trạng thái
        self.assertIsNotNone(t)
        in_thong_ke([t])

    def test_random_tiny_maps(self):
        """Map ngẫu nhiên nhỏ (có tường, nhiều robot, nhiều đích hơn thùng): không báo nhầm."""
        rng = random.Random(20261006)
        tk, da_duyet = [], 0
        while da_duyet < SO_MAP_NGAU_NHIEN:
            w, h = rng.randint(3, 6), rng.randint(3, 5)
            so_tuong = rng.randint(0, 3)
            so_robot = rng.choice([1, 1, 2])
            so_thung = rng.choice([2, 2, 3]) if so_robot == 1 else 2
            so_dich = so_thung + rng.choice([0, 0, 1])
            free = w * h - so_tuong
            if free < so_robot + so_thung + 1 or so_dich > free:
                continue
            if uoc_luong_so_trang_thai(free, so_robot, so_thung) > 40000:
                continue
            s = random_state(rng, w, h, so_robot, so_thung, so_dich, so_tuong)
            t = kiem_tra_oracle(self, s, "ngẫu nhiên #%d (%dx%d)" % (da_duyet + 1, w, h))
            if t is None:
                continue
            tk.append(t)
            da_duyet += 1

        tong_vo_nghiem = sum(t["unsolvable"] for t in tk)
        tong_bat = sum(t["tong"] for t in tk)
        tong_trang_thai = sum(t["states"] for t in tk)
        # Kiểm tra test không rỗng: phải có cả trạng thái giải được lẫn vô nghiệm để
        # việc "không báo nhầm" thật sự có ý nghĩa.
        self.assertGreater(tong_vo_nghiem, 0)
        self.assertGreater(tong_trang_thai - tong_vo_nghiem, 0)
        self.assertGreater(tong_bat, 0, "phải bắt được ít nhất một số deadlock thật")
        print("\n  %d map ngẫu nhiên: %d trạng thái, %d vô nghiệm, bắt được %d (%.1f%%)"
              % (len(tk), tong_trang_thai, tong_vo_nghiem, tong_bat,
                 100.0 * tong_bat / tong_vo_nghiem))


# =============================================================================
# 5. HIỆU NĂNG
# =============================================================================
class TestPerformance(unittest.TestCase):

    def _trang_thai_ngau_nhien(self, p, so_buoc, seed):
        """Đi ngẫu nhiên từ state đầu để lấy các trạng thái thực tế như khi tìm kiếm."""
        rng = random.Random(seed)
        s, kq = p, []
        for _ in range(so_buoc):
            kq.append(s)
            s = rng.choice(s.successors())
        return kq

    def test_speed_on_largest_public_maps(self):
        """is_deadlock gọi trên MỖI successor nên phải nhanh: trung bình dưới 1 ms/lần
        trên map 8×8 có 6 thùng (ngưỡng rộng để không lỗi do máy chậm)."""
        p = PROBLEMS[19]
        dead = _compute_dead_cells(p)
        mau = self._trang_thai_ngau_nhien(p, 400, seed=3)
        so_lan = 0
        bat_dau = time.perf_counter()
        for _ in range(50):
            for s in mau:
                is_deadlock(s, dead)
                so_lan += 1
        us = (time.perf_counter() - bat_dau) / so_lan * 1e6
        print("\n  is_deadlock trung bình: %.1f µs/lần (%d lần gọi, map 8×8, 6 thùng)" % (us, so_lan))
        self.assertLess(us, 1000)

    def test_compute_dead_cells_is_cheap(self):
        """_compute_dead_cells chỉ chạy một lần mỗi map nên phải rất nhanh."""
        bat_dau = time.perf_counter()
        for p in PROBLEMS:
            _compute_dead_cells(p)
        self.assertLess(time.perf_counter() - bat_dau, 0.5)


if __name__ == "__main__":
    unittest.main(verbosity=2)
