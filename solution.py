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


def _build_push_dist_table(initial_state):
    """Return minimum static push counts from every cell to each storage."""
    width = initial_state.width
    height = initial_state.height
    obstacles = initial_state.obstacles
    directions = ((0, -1), (1, 0), (0, 1), (-1, 0))
    per_storage = {}

    for storage in initial_state.storage:
        distances = {storage: 0}
        queue = deque([storage])
        while queue:
            x, y = queue.popleft()
            for dx, dy in directions:
                previous_box = (x - dx, y - dy)
                player_stand = (x - 2 * dx, y - 2 * dy)
                if not (0 <= previous_box[0] < width and
                        0 <= previous_box[1] < height and
                        0 <= player_stand[0] < width and
                        0 <= player_stand[1] < height):
                    continue
                if (previous_box in obstacles or player_stand in obstacles or
                        previous_box in distances):
                    continue
                distances[previous_box] = distances[(x, y)] + 1
                queue.append(previous_box)
        per_storage[storage] = distances

    return per_storage


def _compute_dead_cells(initial_state):
    """
    Tinh tap 'dead_cells': o nao day thung vao se khong bao gio dua duoc
    toi bat ky o dich nao nua. Dung Reverse BFS: bat dau tu cac o dich,
    "keo nguoc" thung ra ngoai (robot dung o phia doi dien voi huong keo).
    O nao khong toi duoc bang BFS nguoc nay la o chet.
    """
    width = initial_state.width
    height = initial_state.height
    obstacles = initial_state.obstacles
    storage = initial_state.storage

    all_cells = frozenset(
        (x, y)
        for x in range(width)
        for y in range(height)
        if (x, y) not in obstacles
    )

    reachable = set(storage)
    queue = deque(storage)
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0)]

    while queue:
        box = queue.popleft()
        for dx, dy in directions:
            prev_box = (box[0] - dx, box[1] - dy)      # vi tri thung truoc khi bi day
            robot_pos = (prev_box[0] - dx, prev_box[1] - dy)  # robot dung phia ngoai de day
            if (prev_box in all_cells and
                    robot_pos in all_cells and
                    prev_box not in reachable):
                reachable.add(prev_box)
                queue.append(prev_box)

    return frozenset(all_cells - reachable)


def _hungarian_min_cost(cost):
    """
    Kuhn-Munkres O(n^3) tren ma tran chi phi vuong NxN (co the dung inf
    cho cap khong hop le). Tra ve (tong_chi_phi_toi_thieu, assignment)
    voi assignment[i] = j nghia la hang i duoc gan cho cot j.
    """
    n = len(cost)
    if n == 0:
        return 0.0, []

    BIG = 1e18  # thay cho inf de tranh loi so hoc trong thuat toan potentials
    c = [[BIG if cost[i][j] == float("inf") else cost[i][j] for j in range(n)]
         for i in range(n)]

    u = [0.0] * (n + 1)
    v = [0.0] * (n + 1)
    p = [0] * (n + 1)
    way = [0] * (n + 1)

    for i in range(1, n + 1):
        p[0] = i
        j0 = 0
        minv = [float("inf")] * (n + 1)
        used = [False] * (n + 1)
        while True:
            used[j0] = True
            i0 = p[j0]
            delta = float("inf")
            j1 = -1
            for j in range(1, n + 1):
                if not used[j]:
                    cur = c[i0 - 1][j - 1] - u[i0] - v[j]
                    if cur < minv[j]:
                        minv[j] = cur
                        way[j] = j0
                    if minv[j] < delta:
                        delta = minv[j]
                        j1 = j
            for j in range(0, n + 1):
                if used[j]:
                    u[p[j]] += delta
                    v[j] -= delta
                else:
                    minv[j] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        while True:
            j1 = way[j0]
            p[j0] = p[j1]
            j0 = j1
            if j0 == 0:
                break

    assignment = [0] * n
    total = 0.0
    for j in range(1, n + 1):
        if p[j] != 0:
            assignment[p[j] - 1] = j - 1
            total += cost[p[j] - 1][j - 1]
    return total, assignment


def _push_heuristic(state, push_dist_table, assignment_cache):
    """Estimate remaining pushes plus a lower bound on initial robot walking."""
    boxes = tuple(sorted(box for box in state.boxes if box not in state.storage))
    if not boxes:
        return 0

    push_cost = assignment_cache.get(boxes)
    if push_cost is None:
        storages = tuple(state.storage)
        box_count = len(boxes)
        storage_count = len(storages)
        size = max(box_count, storage_count)
        costs = [[float("inf")] * size for _ in range(size)]

        for row, box in enumerate(boxes):
            for column, storage in enumerate(storages):
                costs[row][column] = push_dist_table[storage].get(
                    box, float("inf"))
            for column in range(storage_count, size):
                costs[row][column] = 0  # padding: hang/cot gia, chi phi 0

        for row in range(box_count, size):
            for column in range(size):
                costs[row][column] = 0

        push_cost, _ = _hungarian_min_cost(costs)
        assignment_cache[boxes] = push_cost

    if push_cost >= 1e17:
        return float("inf")

    nearest_robot_box = min(
        abs(robot[0] - box[0]) + abs(robot[1] - box[1])
        for robot in state.robots
        for box in boxes
    )
    return int(push_cost) + max(0, nearest_robot_box - 1)


def _is_frozen_deadlock(state):
    """
    Deadlock dong kieu "freeze de quy": mot o ke bi coi la chan VINH VIEN
    neu la tuong/mep ban, HOAC la thung ma chinh thung do cung dong cung
    tren CA HAI truc (hoan toan khong the di chuyen huong nao). Neu ca 2
    phia tren 1 truc deu bi chan vinh vien thi thung dong cung truc do;
    deadlock khi dong cung ca 2 truc va chua o dich.

    Quan trong: phai doi hoi thung hang xom dong cung CA HAI truc, khong
    chi truc dang xet — vi mot thung dong cung truc ngang van co the duoc
    day DOC de roi di, lam cho vi tri do trong tro lai sau nay. Vi du: 3
    thung L-B-R nam ngang, B ket doc that su (tuong tren/duoi truc tiep)
    nhung L/R van con duong thoat doc -> khong phai deadlock that.

    Mau 2x2 noi tu do (khong co tuong) duoc bat rieng boi _is_2x2_deadlock.
    """
    width = state.width
    height = state.height
    obstacles = state.obstacles
    boxes = state.boxes
    storage = state.storage

    def is_wall(x, y):
        return x < 0 or x >= width or y < 0 or y >= height or (x, y) in obstacles

    memo = {}   # (box, axis) -> bool

    def frozen_on_axis(box, axis):
        key = (box, axis)
        if key in memo:
            return memo[key]

        memo[key] = True   # gia su dong cung truoc khi tinh, de xu ly chu ky

        bx, by = box
        neighbors = [(bx-1, by), (bx+1, by)] if axis == 0 else [(bx, by-1), (bx, by+1)]

        for nx, ny in neighbors:
            if is_wall(nx, ny):
                continue
            if ((nx, ny) in boxes
                    and frozen_on_axis((nx, ny), 0)
                    and frozen_on_axis((nx, ny), 1)):
                continue
            memo[key] = False
            return False

        memo[key] = True
        return True

    for box in boxes:
        if box in storage:
            continue
        if frozen_on_axis(box, 0) and frozen_on_axis(box, 1):
            return True

    return False


def _is_2x2_deadlock(state):
    """
    Deadlock dong mau 2x2: neu 4 o hinh vuong lien ke deu bi chiem (thung,
    tuong, hoac mep ban) va co it nhat 1 thung chua vao o dich -> deadlock
    (khong the thao go). Bo sung cho truong hop _is_frozen_deadlock bo lot
    (2x2 noi tu do, khong co tuong).
    """
    width = state.width
    height = state.height

    def is_blocked(x, y):
        return x < 0 or x >= width or y < 0 or y >= height or (x, y) in state.boxes or (x, y) in state.obstacles

    for bx, by in state.boxes:
        for dx, dy in [(0, 0), (-1, 0), (0, -1), (-1, -1)]:
            cx, cy = bx + dx, by + dy
            corners = [(cx, cy), (cx+1, cy), (cx, cy+1), (cx+1, cy+1)]
            if not all(is_blocked(x, y) for x, y in corners):
                continue
            for cx2, cy2 in corners:
                if (0 <= cx2 < width and 0 <= cy2 < height and
                        (cx2, cy2) in state.boxes and
                        (cx2, cy2) not in state.storage):
                    return True
    return False


def is_deadlock(state, dead_cells=None):
    """
    3 lop kiem tra, theo thu tu:
      1. Deadlock tinh: thung nam trong 'dead_cells' (tinh truoc 1 lan/map).
      2. Freeze de quy: thung dong cung ca 2 truc.
      3. Mau 2x2: 4 o lien ke bi chiem, co thung chua vao dich.

    TUYET DOI khong tra True nham (false positive) — se lam mat loi giai!
    """
    if dead_cells is not None:
        for box in state.boxes:
            if box not in state.storage and box in dead_cells:
                return True

    if _is_frozen_deadlock(state):
        return True

    return _is_2x2_deadlock(state)


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
