"""Student entry point. See README.md for the API and scoring rules."""
import heapq
import time
from collections import deque
from sokoban import SokobanState, sokoban_goal_state
def solve(initial_state, timebound=120):
    """Try push-level search first on the largest maps, then fall back to A*."""
    if timebound <= 0:
        return False

    start_time = time.perf_counter()
    precomputed = precompute(initial_state)

    if sokoban_goal_state(initial_state):
        return initial_state

    is_large_multi_box_map = (
        initial_state.width * initial_state.height >= 64 and
        len(initial_state.boxes) >= 6
    )
    if is_large_multi_box_map:
        remaining = timebound - (time.perf_counter() - start_time)
        push_budget = min(30.0, remaining * 0.25)
        push_result = _push_search(initial_state, push_budget, precomputed)
        if push_result is not False:
            return push_result

    remaining = timebound - (time.perf_counter() - start_time)
    return _step_search(initial_state, remaining, precomputed)

def precompute(initial_state):
    """
    Tinh truoc du lieu tinh cho ca map (chi chay 1 lan/map).
    Tra ve dict chua:
        'push_dist_table': dict[storage] -> dict[(x,y)] = so lan day toi thieu
        'dead_cells': frozenset cac o ma day thung vao la thua cuoc
    """
    push_dist_table = _build_push_dist_table(initial_state)
    dead_cells = _compute_dead_cells(initial_state)
    return {
        "push_dist_table": push_dist_table,
        "dead_cells": dead_cells,
    }

def heuristic(state, dist_table=None):
    """Ước lượng chi phí né tường bằng dist_table của TV2."""
    # Phòng vệ nếu người gọi truyền cả dict {"dist_table": ...}
    if isinstance(dist_table, dict) and "dist_table" in dist_table:
        dist_table = dist_table["dist_table"]
    # Phòng vệ nếu TV1 gọi không truyền dist_table -> Tự động tính theo Manhattan
    if dist_table is None:
        return sum(
            min(abs(b[0] - s[0]) + abs(b[1] - s[1]) for s in state.storage)
            for b in state.boxes
        )
    total_distance = 0
    for box in state.boxes:
        min_dist = float("inf")
        for target in state.storage:
            d = dist_table[target].get(box, float("inf"))
            if d < min_dist:
                min_dist = d
        # Nếu một thùng không thể tới bất kỳ đích nào -> Cắt nhánh thua ngay
        if min_dist == float("inf"):
            return float("inf")
        total_distance += min_dist
    return total_distance



def is_deadlock(state):
    """Teammate hook: return True when this state should be pruned."""
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

def _step_search(initial_state, timebound, precomputed):
    """
    Giai Sokoban bang Weighted A* (f = g + W*h).
    Tra ve SokobanState dich (co parent chain day du), hoac False neu het
    thoi gian. Khong thay doi initial_state, khong tao thread/process,
    khong dung bien toan cuc giua cac lan goi.
    """
    start_time = time.time()

    if timebound <= 0:
        return False

    WEIGHT = 4.0  # giai duoc 20/20 map trong 20s

    push_dist_table = precomputed["push_dist_table"]
    dead_cells = precomputed["dead_cells"]
    assignment_cache = {}

    def search_key(state):
        return tuple(sorted(state.robots)), state.boxes

    if sokoban_goal_state(initial_state):
        return initial_state

    # open list: (f_val, neg_g, count, state)
    #   f_val = g + W*h : uu tien node co f nho nhat
    #   neg_g = -gval    : cung f thi uu tien node di xa hon
    #   count            : tranh heapq phai so sanh truc tiep SokobanState
    counter = 0
    g0 = initial_state.gval
    h0 = _push_heuristic(initial_state, push_dist_table, assignment_cache)
    if h0 == float("inf"):
        return False
    open_list = [(g0 + WEIGHT * h0, -g0, counter, initial_state)]

    visited = {search_key(initial_state): g0}  # hashable_state -> gval tot nhat da biet

    while open_list:
        if time.time() - start_time >= timebound - 0.5:
            return False

        _, neg_g, _, current = heapq.heappop(open_list)
        current_g = -neg_g

        key = search_key(current)
        if visited.get(key, float("inf")) < current_g:
            continue  # node loi thoi, da co duong ngan hon toi cung cau hinh

        if sokoban_goal_state(current):
            return current

        for next_state in current.successors():
            next_key = search_key(next_state)
            next_g = next_state.gval

            if visited.get(next_key, float("inf")) <= next_g:
                continue

            if is_deadlock(next_state, dead_cells):
                continue

            visited[next_key] = next_g
            counter += 1
            next_h = _push_heuristic(
                next_state, push_dist_table, assignment_cache)
            if next_h == float("inf"):
                continue
            next_f = next_g + WEIGHT * next_h
            heapq.heappush(open_list, (next_f, -next_g, counter, next_state))

    return False

def _push_successors(state, dead_cells, deadline):
    """Yield legal pushes with their shortest local walking paths attached."""
    width = state.width
    height = state.height
    directions = ((0, -1, "up"), (1, 0, "right"),
                  (0, 1, "down"), (-1, 0, "left"))

    for robot_index, start in enumerate(state.robots):
        blocked = set(state.boxes) | set(state.obstacles)
        blocked.update(robot for index, robot in enumerate(state.robots)
                       if index != robot_index)
        parents = {start: None}
        queue = deque([start])

        while queue and time.perf_counter() < deadline:
            x, y = queue.popleft()
            for dx, dy, direction_name in directions:
                next_position = (x + dx, y + dy)
                if not (0 <= next_position[0] < width and
                        0 <= next_position[1] < height):
                    continue
                if next_position in blocked or next_position in parents:
                    continue
                parents[next_position] = ((x, y), direction_name)
                queue.append(next_position)

        for box in state.boxes:
            if time.perf_counter() >= deadline:
                return
            for dx, dy, direction_name in directions:
                stand = (box[0] - dx, box[1] - dy)
                destination = (box[0] + dx, box[1] + dy)
                if stand not in parents:
                    continue
                if not (0 <= destination[0] < width and
                        0 <= destination[1] < height):
                    continue
                if destination in state.boxes or destination in state.obstacles:
                    continue
                if destination in state.robots:
                    continue

                walk = []
                position = stand
                while position != start:
                    previous, walk_direction = parents[position]
                    walk.append((position, walk_direction))
                    position = previous

                walking_state = state
                for position, walk_direction in reversed(walk):
                    robots = list(walking_state.robots)
                    robots[robot_index] = position
                    walking_state = SokobanState(
                        f"{robot_index} {walk_direction}",
                        walking_state.gval + 1,
                        walking_state,
                        width,
                        height,
                        tuple(robots),
                        state.boxes,
                        state.storage,
                        state.obstacles,
                    )

                boxes = set(state.boxes)
                boxes.remove(box)
                boxes.add(destination)
                robots = list(walking_state.robots)
                robots[robot_index] = box
                pushed_state = SokobanState(
                    f"{robot_index} {direction_name}",
                    walking_state.gval + 1,
                    walking_state,
                    width,
                    height,
                    tuple(robots),
                    frozenset(boxes),
                    state.storage,
                    state.obstacles,
                )
                if not is_deadlock(pushed_state, dead_cells):
                    yield pushed_state

def _push_search(initial_state, timebound, precomputed):
    """Search over pushes while retaining a legal unit-step parent chain."""
    if timebound <= 0:
        return False

    deadline = time.perf_counter() + timebound
    push_dist_table = precomputed["push_dist_table"]
    dead_cells = precomputed["dead_cells"]
    assignment_cache = {}
    weight = 4.0

    def search_key(state):
        return tuple(sorted(state.robots)), state.boxes

    initial_h = _push_heuristic(initial_state, push_dist_table, assignment_cache)
    if initial_h == float("inf"):
        return False

    counter = 0
    open_list = [(initial_state.gval + weight * initial_h,
                  -initial_state.gval, counter, initial_state)]
    visited = {search_key(initial_state): initial_state.gval}

    while open_list and time.perf_counter() < deadline:
        _, negative_g, _, current = heapq.heappop(open_list)
        current_g = -negative_g
        key = search_key(current)
        if visited.get(key, float("inf")) < current_g:
            continue
        if sokoban_goal_state(current):
            return current

        for next_state in _push_successors(current, dead_cells, deadline):
            next_key = search_key(next_state)
            next_g = next_state.gval
            if visited.get(next_key, float("inf")) <= next_g:
                continue
            next_h = _push_heuristic(
                next_state, push_dist_table, assignment_cache)
            if next_h == float("inf"):
                continue
            visited[next_key] = next_g
            counter += 1
            priority = next_g + weight * next_h
            heapq.heappush(open_list,
                           (priority, -next_g, counter, next_state))

    return False

