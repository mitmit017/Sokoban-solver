"""Student entry point. See README.md for the API and scoring rules."""

import heapq
import time
from sokoban import sokoban_goal_state

def heuristic(state):
    """Teammate hook: replace this with the heuristic implementation."""
    return 0


from collections import deque

# Bốn hướng di chuyển, cùng thứ tự với sokoban.py: lên, phải, xuống, trái.
_DL_DIRS = ((0, -1), (1, 0), (0, 1), (-1, 0))


def _dl_is_wall(x, y, width, height, obstacles):
    """Trả về True nếu ô (x, y) là tường: ngoài bàn chơi (biên) hoặc là obstacle."""
    return x < 0 or y < 0 or x >= width or y >= height or (x, y) in obstacles


# -----------------------------------------------------------------------------
# LỚP 1 — Ô CHẾT TĨNH
# -----------------------------------------------------------------------------
def _compute_dead_cells(state):
    """Tính tập ô chết tĩnh của map bằng Reverse BFS ("kéo" thùng ngược từ ô đích).

    Ô chết: nếu một thùng nằm ở ô đó thì dù đẩy thế nào cũng KHÔNG THỂ tới được bất
    kỳ ô đích nào (chỉ xét tường, bỏ qua thùng khác và robot).

    Cách làm: bắt đầu từ mọi ô đích (luôn "sống"). Một cú đẩy thuận theo hướng d đưa
    thùng từ p sang q = p + d, khi đó robot đứng ở r = p - d. Đảo ngược lại: từ q ta
    tìm được p = q - d, với điều kiện p và r = q - 2d đều không phải tường. Ô nào
    không bao giờ được tìm thấy là ô chết.

    Vì bỏ qua thùng khác và robot nên tập ô chết tìm được là tập con chắc chắn của ô
    chết thật -> an toàn (không báo nhầm). Khác với luật "góc là ô chết", cách này tự
    xử lý đúng trường hợp góc có ô đích, tường trong bàn, bàn bị chia cắt...

    Trả về: frozenset các ô (x, y) chết (không gồm ô tường).
    """
    width, height, obstacles = state.width, state.height, state.obstacles

    # Ô đích hợp lệ là điểm xuất phát; chúng luôn sống nên không bao giờ là ô chết.
    alive = {c for c in state.storage
             if not _dl_is_wall(c[0], c[1], width, height, obstacles)}
    queue = deque(alive)

    while queue:
        qx, qy = queue.popleft()
        for dx, dy in _DL_DIRS:
            px, py = qx - dx, qy - dy          # ô thùng đứng TRƯỚC cú đẩy
            rx, ry = qx - 2 * dx, qy - 2 * dy  # ô robot phải đứng để thực hiện cú đẩy
            if (px, py) in alive:
                continue
            if _dl_is_wall(px, py, width, height, obstacles):
                continue
            if _dl_is_wall(rx, ry, width, height, obstacles):
                continue
            alive.add((px, py))
            queue.append((px, py))

    # Ô chết = mọi ô trong bàn, không phải tường, và không nằm trong tập "sống".
    return frozenset((x, y)
                     for x in range(width) for y in range(height)
                     if (x, y) not in obstacles and (x, y) not in alive)


# -----------------------------------------------------------------------------
# HÀM CHÍNH CHO NHÓM GỌI
# -----------------------------------------------------------------------------
def is_deadlock(state, dead_cells=None):
    """Trả về True nếu `state` CHẮC CHẮN không thể giải được (nên cắt tỉa).

    Tham số:
        state:      một SokobanState.
        dead_cells: tập ô chết từ _compute_dead_cells(initial_state). Nếu để None, hàm
                    tự tính lại từ `state` (đúng nhưng chậm) và KHÔNG lưu kết quả giữa
                    các lần gọi (không dùng biến toàn cục, theo ràng buộc của đề).

    Hàm không sửa `state`. Trạng thái đích (mọi thùng đã trên ô đích) luôn trả False.
    """
    if dead_cells is None:
        dead_cells = _compute_dead_cells(state)

    # Lớp 1: có thùng nằm trên ô chết (ô đích không bao giờ là ô chết).
    for box in state.boxes:
        if box in dead_cells:
            return True
    return False




"""
abs(box[0] - s[0]) + abs(box[1] - s[1]): Đây là công thức tính Khoảng cách Manhattan.
Tại sao? Trong Sokoban, bạn chỉ có thể đẩy hộp theo 4 hướng (Lên, Xuống, Trái, Phải), không thể đi chéo. Do đó, khoảng cách Manhattan là ước lượng chính xác nhất cho số bước di chuyển.
Tại sao lại dùng cách này? Để A* tìm được đường đi ngắn nhất (tối ưu), hàm heuristic bắt buộc phải là Admissible (luôn đánh giá ước lượng nhỏ hơn hoặc bằng số bước thực tế). Việc giả định "hộp bay xuyên tường đến đích gần nhất" chắc chắn tốn ít số bước hơn so với thực tế phải đẩy vòng vèo, do đó nó thỏa mãn điều kiện Admissible.
"""


def weighted_a_star(initial_state, timebound, weight=1.0):
    deadline = time.time() + timebound

    if sokoban_goal_state(initial_state):
        return initial_state
    if timebound <= 0:
        return False

    # open list: (f = g + w·h,  tie-break counter,  state)
    counter = 0
    open_list = []
    h0 = heuristic(initial_state)
    heapq.heappush(open_list, (initial_state.gval + weight * h0, counter, initial_state))
    counter += 1

    # closed: hashable_state → best g đã thấy
    closed = {initial_state.hashable_state(): initial_state.gval}

    while open_list:
        if time.time() >= deadline:
            return False

        _, _, cur = heapq.heappop(open_list)
        key = cur.hashable_state()

        # Đã có đường tốt hơn → bỏ qua
        if closed.get(key, float('inf')) < cur.gval:
            continue 
        # tại sao lại có dòng này? Trong hàng đợi có thể có 2 phiên bản của cùng 1 trạng thái (do lúc mở rộng ta nhét vào, nhưng chưa kịp lấy ra thì đã tìm được đường khác ngắn hơn cũng chui vào hàng đợi). Dòng này để vứt bỏ phiên bản có đương đi dài hơn

        if sokoban_goal_state(cur):
            return cur

        for succ in cur.successors():
            if is_deadlock(succ):
                continue

            sk = succ.hashable_state()
            sg = succ.gval
            if sg < closed.get(sk, float('inf')):
                closed[sk] = sg
                f = sg + weight * heuristic(succ)
                heapq.heappush(open_list, (f, counter, succ))
                counter += 1

    return False
"""
Tại sao có weight? Trọng số $w$ giúp thuật toán "tham lam" hơn. Nếu $w=2$, A* sẽ coi trọng việc tiến gần đích ($h$) gấp đôi so với việc chắt chiu từng bước đi ($g$), giúp thuật toán lao tới đích nhanh hơn rất nhiều, bù lại đường đi có thể dài hơn (sub-optimal).
Tại sao cần biến counter? Khi cho tuple (f, counter, state) vào heapq, nó sẽ sắp xếp theo $f$. Nếu hai state có $f$ bằng nhau, nó sẽ lấy phần tử thứ 2 (counter) ra so sánh. Nếu không có counter, nó sẽ so sánh 2 object SokobanState và Python sẽ báo lỗi sập chương trình vì nó không biết state nào "lớn hơn" state nào. counter tăng dần giúp phần tử nào được đưa vào hàng đợi trước sẽ được xét trước (FIFO).
Tại sao dùng closed là Dictionary thay vì Set (Tập hợp)? Set chỉ lưu việc "trạng thái này đã đi qua hay chưa". Nhưng với Weighted A*, đôi khi bạn quay lại một trạng thái cũ nhưng qua một con đường ngắn hơn con đường lúc trước. Dùng dictionary lưu cặp Trạng_thái -> Số_bước_g_nhỏ_nhất giúp ta kiểm tra cực nhanh ($O(1)$) xem có nên cập nhật lại đường đi này hay không.
Vòng lặp chạy liên tục rút trạng thái có $f$ nhỏ nhất ra. Hàm .hashable_state() thường chuyển các mảng toạ độ (thường là List/Tuple) thành string hoặc tuple cố định để có thể băm (hash) và làm key cho Dictionary.
"""

def two_tier_search(initial_state, timebound, weight=2.0, split=0.4):
    budget1 = timebound * split
    budget2 = timebound - budget1

    # --- Tier 1: Weighted A* (nhanh) ---
    result = weighted_a_star(initial_state, timebound=budget1, weight=weight)
    if result:
        return result

    # --- Tier 2: standard A* (optimal) ---
    return weighted_a_star(initial_state, timebound=budget2, weight=1.0)
"""
Cách này dùng 40% thời gian đầu (Tier 1) để chạy Weighted A* ($w=2$). Nó giống như "kẻ liều mạng", chạy thục mạng về phía đích mà không cần tính toán đường đi đẹp nhất. Thường nó sẽ giải được map rất nhanh. Đạt được mục tiêu: Có lời giải còn hơn là bị 0 điểm vì timeout.
Nếu map quá loằng ngoằng, Tier 1 chạy "bừa" mà không tới đích, ta dùng 60% thời gian còn lại (Tier 2) để gọi A* chuẩn ($w=1$), quét cẩn thận từng ngóc ngách như một phương án dự phòng (fallback).
"""

def solve(initial_state, timebound=120):
    return two_tier_search(initial_state, timebound=timebound)

