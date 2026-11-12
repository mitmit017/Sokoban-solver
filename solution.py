
import heapq
import time
from collections import deque
from sokoban import SokobanState, sokoban_goal_state

_DIRECTIONS = ((0, -1, "up"), (1, 0, "right"),
               (0, 1, "down"), (-1, 0, "left"))
_WEIGHT = 3.0
_FIRST_SEARCH_WEIGHTS = (_WEIGHT, 6.0, 10.0)
# Per-attempt distinct-state cap, chosen for this experimental version.
_FIRST_SEARCH_STATE_LIMIT = 250000
# Optional improvement uses at most 25% of the original budget and 20 seconds.
_IMPROVEMENT_MAX_SECONDS = 20.0
_IMPROVEMENT_BUDGET_FRACTION = 0.25
# Stop after two unsuccessful attempts, or 8 seconds without a better path.
_IMPROVEMENT_MAX_STALLED_ATTEMPTS = 2
_IMPROVEMENT_STALL_SECONDS = 8.0
_CACHE_LIMIT = 100000
_IMPROVEMENT_WEIGHTS = (2.0, 1.4, 1.0)
_IMPROVEMENT_STATE_LIMIT = 100000


class _Timeout(Exception):
    pass


class _SearchLimit(_Timeout):
    pass


def _check_time(deadline):
    if time.perf_counter() >= deadline:
        raise _Timeout


class _Node:
    __slots__ = ("robots", "boxes", "cost", "parent", "actions")

    def __init__(self, robots, boxes, cost, parent=None, actions=()):
        self.robots = robots
        self.boxes = boxes
        self.cost = cost
        self.parent = parent
        self.actions = actions


class _Board:
    def __init__(self, initial, deadline):
        self.width = initial.width
        self.height = initial.height
        self.storage = initial.storage
        self.obstacles = initial.obstacles
        self.deadline = deadline
        self.neighbors = {}
        for y in range(self.height):
            _check_time(deadline)
            for x in range(self.width):
                if (x, y) in self.obstacles:
                    continue
                self.neighbors[x, y] = tuple(
                    (x + dx, y + dy)
                    if (0 <= x + dx < self.width and 0 <= y + dy < self.height
                        and (x + dx, y + dy) not in self.obstacles) else None
                    for dx, dy, _ in _DIRECTIONS)
        self.goals = tuple(sorted(self.storage))
        self.push_distances = {}
        reachable = set()
        for goal in self.goals:
            distances = {goal: 0}
            queue = deque([goal])
            while queue:
                _check_time(deadline)
                cell = queue.popleft()
                for direction in range(4):
                    previous = self.neighbors[cell][direction]
                    if previous is None:
                        continue
                    support = self.neighbors[previous][direction]
                    if support is None or previous in distances:
                        continue
                    distances[previous] = distances[cell] + 1
                    queue.append(previous)
            self.push_distances[goal] = distances
            reachable.update(distances)
        self.dead_cells = frozenset(self.neighbors).difference(reachable)
        self.assignment_cache = {}
        self.deadlock_cache = {}

    def deadlocked(self, boxes):
        _check_time(self.deadline)
        cached = self.deadlock_cache.get(boxes)
        if cached is not None:
            return cached
        result = (bool(boxes & self.dead_cells)
                  or _is_frozen_deadlock(boxes, self))
        if len(self.deadlock_cache) >= _CACHE_LIMIT:
            self.deadlock_cache.clear()
        self.deadlock_cache[boxes] = result
        return result

    def estimate(self, robots, boxes, deadline):
        _check_time(deadline)
        if boxes <= self.storage:
            return 0
        push_cost = self.assignment_cache.get(boxes)
        if push_cost is None:
            if len(boxes) > len(self.goals):
                return float("inf")
            costs = []
            for box in sorted(boxes):
                _check_time(deadline)
                costs.append([self.push_distances[goal].get(box, float("inf"))
                              for goal in self.goals])
            costs.extend([[0] * len(self.goals)
                          for _ in range(len(self.goals) - len(costs))])
            push_cost, _ = _hungarian_min_cost(costs, deadline)
            if len(self.assignment_cache) >= _CACHE_LIMIT:
                self.assignment_cache.clear()
            self.assignment_cache[boxes] = push_cost
        if push_cost == float("inf"):
            return push_cost
        # Before the first push, a robot must reach some box, including goal boxes.
        approach = min(abs(rx-bx) + abs(ry-by)
                       for rx, ry in robots for bx, by in boxes)
        return int(push_cost) + max(0, approach - 1)


def _step_successors(node, board, deadline):
    for robot_index, position in enumerate(node.robots):
        other_robots = set(node.robots)
        other_robots.remove(position)
        for direction, destination in enumerate(board.neighbors[position]):
            _check_time(deadline)
            if destination is None or destination in other_robots:
                continue
            boxes = node.boxes
            if destination in boxes:
                box_destination = board.neighbors[destination][direction]
                if (box_destination is None or box_destination in boxes
                        or box_destination in other_robots):
                    continue
                boxes = (boxes - {destination}) | {box_destination}
                if board.deadlocked(boxes):
                    continue
            robots = list(node.robots)
            robots[robot_index] = destination
            yield tuple(robots), boxes, ((robot_index, direction),)


def _push_successors(node, board, deadline):
    for robot_index, position in enumerate(node.robots):
        other_robots = set(node.robots)
        other_robots.remove(position)
        blocked = node.boxes | other_robots
        parents = {position: None}
        queue = deque([position])
        while queue:
            _check_time(deadline)
            cell = queue.popleft()
            for direction, neighbor in enumerate(board.neighbors[cell]):
                if neighbor is not None and neighbor not in blocked and neighbor not in parents:
                    parents[neighbor] = (cell, direction)
                    queue.append(neighbor)
        for box in sorted(node.boxes):
            for direction, destination in enumerate(board.neighbors[box]):
                _check_time(deadline)
                stand = board.neighbors[box][(direction + 2) % 4]
                if (stand not in parents or destination is None
                        or destination in blocked):
                    continue
                boxes = (node.boxes - {box}) | {destination}
                if board.deadlocked(boxes):
                    continue
                actions = []
                cell = stand
                while cell != position:
                    _check_time(deadline)
                    cell, walk_direction = parents[cell]
                    actions.append((robot_index, walk_direction))
                actions.reverse()
                actions.append((robot_index, direction))
                robots = list(node.robots)
                robots[robot_index] = box
                yield tuple(robots), boxes, tuple(actions)


def _search(initial, board, deadline, pushes, stats, weight=_WEIGHT,
            best_cost_bound=float('inf'), state_limit=None):
    stats['searches'].append('push' if pushes else 'step')
    root = _Node(initial.robots, initial.boxes, initial.gval)
    if board.deadlocked(root.boxes):
        return None
    estimate = board.estimate(root.robots, root.boxes, deadline)
    if estimate == float('inf') or root.cost + estimate >= best_cost_bound:
        return None
    frontier = [(root.cost + weight * estimate, -root.cost, 0, root)]
    best_cost = {(tuple(sorted(root.robots)), root.boxes): root.cost}
    serial = 0
    successors = _push_successors if pushes else _step_successors
    while frontier:
        _check_time(deadline)
        _, _, _, current = heapq.heappop(frontier)
        key = (tuple(sorted(current.robots)), current.boxes)
        if best_cost.get(key, float('inf')) < current.cost:
            continue
        if current.boxes <= board.storage:
            return current
        stats['expanded'] += 1
        for robots, boxes, actions in successors(current, board, deadline):
            stats['generated'] += 1
            cost = current.cost + len(actions)
            key = (tuple(sorted(robots)), boxes)
            if best_cost.get(key, float('inf')) <= cost:
                continue
            estimate = board.estimate(robots, boxes, deadline)
            if estimate == float('inf') or cost + estimate >= best_cost_bound:
                continue
            if state_limit is not None and key not in best_cost and len(best_cost) >= state_limit:
                raise _SearchLimit
            best_cost[key] = cost
            serial += 1
            node = _Node(robots, boxes, cost, current, actions)
            heapq.heappush(frontier, (cost + weight * estimate, -cost, serial, node))
        stats['peak_frontier'] = max(stats['peak_frontier'], len(frontier))
    return None


def _reconstruct(initial, node, deadline):
    segments = []
    while node.parent is not None:
        _check_time(deadline)
        segments.append(node.actions)
        node = node.parent
    current = initial
    for actions in reversed(segments):
        for robot_index, direction in actions:
            _check_time(deadline)
            dx, dy, name = _DIRECTIONS[direction]
            x, y = current.robots[robot_index]
            destination = (x + dx, y + dy)
            robots = list(current.robots)
            robots[robot_index] = destination
            boxes = current.boxes
            if destination in boxes:
                boxes = (boxes - {destination}) | {(destination[0]+dx, destination[1]+dy)}
            current = SokobanState(f'{robot_index} {name}', current.gval + 1,
                                   current, initial.width, initial.height,
                                   tuple(robots), boxes, initial.storage, initial.obstacles)
    return current


def solve(initial_state, timebound=120):
    """Return a legal goal chain or False. Does not guarantee move optimality."""
    return _solve(initial_state, timebound)


def _find_first(initial, board, deadline, pushes, stats):
    """Retry a bounded search with larger weights after a state-limit stop.

    All attempts share a deadline. A state cap is not a hard byte limit:
    heap entries, parent nodes, actions and caches also occupy memory.
    """
    for weight in _FIRST_SEARCH_WEIGHTS:
        _check_time(deadline)
        stats['first_attempts'].append((pushes, weight))
        try:
            return _search(initial, board, deadline, pushes, stats,
                           weight=weight, state_limit=_FIRST_SEARCH_STATE_LIMIT)
        except _SearchLimit:
            stats['first_state_limits'] += 1
        # The exception and its search frames have been released here.
        board.assignment_cache.clear()
        board.deadlock_cache.clear()
    return None


def _solve(initial_state, timebound=120, *, stats=None):
    """Solve with bounded restarts and optional, limited improvement."""
    if stats is None:
        stats = {}
    stats.update(expanded=0, generated=0, peak_frontier=0, searches=[],
                 first_steps=None, first_attempts=[], first_state_limits=0,
                 improvements=[], improvement_stop=None)
    if timebound <= 0:
        return False
    deadline = time.perf_counter() + timebound - min(0.02, timebound * 0.02)
    # Reserve 5% on normal budgets; keep a bounded margin on short budgets.
    reserve = min(max(1.0, timebound * 0.05), timebound * 0.2)
    search_deadline = deadline - reserve
    if sokoban_goal_state(initial_state):
        return initial_state
    best_solution = False
    try:
        board = _Board(initial_state, search_deadline)
        single_robot = len(initial_state.robots) == 1
        large_map = initial_state.width * initial_state.height >= 64 and len(initial_state.boxes) >= 6
        node = None
        if single_robot:
            node = _find_first(initial_state, board, search_deadline, True, stats)
        else:
            # Step search also allows robots to walk aside without pushing.
            if large_map:
                push_deadline = min(search_deadline, time.perf_counter() +
                                    min(30.0, (search_deadline-time.perf_counter()) * 0.25))
                try:
                    node = _find_first(initial_state, board, push_deadline, True, stats)
                except _Timeout:
                    pass
            if node is None:
                node = _find_first(initial_state, board, search_deadline, False, stats)
        if node is None:
            return False
        best_solution = _reconstruct(initial_state, node, deadline)
        stats['first_steps'] = best_solution.gval
        node = None

        start = time.perf_counter()
        improvement_budget = min(_IMPROVEMENT_MAX_SECONDS,
                                 timebound * _IMPROVEMENT_BUDGET_FRACTION)
        improvement_deadline = min(search_deadline, start + improvement_budget)
        remaining = improvement_deadline - start
        if remaining <= 0:
            stats['improvement_stop'] = 'no_time_remaining'
            return best_solution
        stall_window = min(_IMPROVEMENT_STALL_SECONDS, timebound * 0.1)
        last_improvement = start
        stalled_attempts = 0
        for weight, fraction in zip(_IMPROVEMENT_WEIGHTS, (0.35, 0.70, 1.0)):
            phase_deadline = start + remaining * fraction
            while time.perf_counter() < phase_deadline:
                attempt_deadline = min(phase_deadline, last_improvement + stall_window)
                if time.perf_counter() >= attempt_deadline:
                    stats['improvement_stop'] = 'stalled_time'
                    return best_solution
                try:
                    node = _search(initial_state, board, attempt_deadline,
                                   single_robot, stats, weight, best_solution.gval,
                                   _IMPROVEMENT_STATE_LIMIT)
                    if node is None:
                        stats['improvement_stop'] = 'bounded_search_exhausted'
                        return best_solution
                    candidate = _reconstruct(initial_state, node, deadline)
                    if candidate.gval < best_solution.gval:
                        best_solution = candidate
                        stats['improvements'].append((weight, candidate.gval))
                        last_improvement = time.perf_counter()
                        stalled_attempts = 0
                    node = None
                except _SearchLimit:
                    stats['improvement_stop'] = 'state_limit'
                    stalled_attempts += 1
                    break
                except _Timeout:
                    stats['improvement_stop'] = 'phase_timeout'
                    stalled_attempts += 1
                    break
            if time.perf_counter() >= last_improvement + stall_window:
                stats['improvement_stop'] = 'stalled_time'
                return best_solution
            if stalled_attempts >= _IMPROVEMENT_MAX_STALLED_ATTEMPTS:
                stats['improvement_stop'] = 'stalled_attempts'
                return best_solution
        stats['improvement_stop'] = 'improvement_budget'
    except _Timeout:
        stats['improvement_stop'] = 'deadline'
    return best_solution


def _hungarian_min_cost(cost, deadline):
    """Find a minimum-cost assignment for a square matrix in O(n^3).

    Return (total_cost, assignment), where assignment[row] is its column.
    Infinite costs represent forbidden pairs."""
    matrix_size = len(cost)
    if matrix_size == 0:
        return 0.0, []

    INFINITE_COST = 1e18  # Avoid invalid arithmetic when updating potentials.
    cost_matrix = [
        [INFINITE_COST if cost[row][col] == float("inf") else cost[row][col]
         for col in range(matrix_size)]
        for row in range(matrix_size)
    ]

    row_potential = [0.0] * (matrix_size + 1)
    col_potential = [0.0] * (matrix_size + 1)
    matched_row = [0] * (matrix_size + 1)
    previous_col = [0] * (matrix_size + 1)

    for row in range(1, matrix_size + 1):
        _check_time(deadline)
        matched_row[0] = row
        current_col = 0
        min_slack = [float("inf")] * (matrix_size + 1)
        visited_cols = [False] * (matrix_size + 1)
        while True:
            _check_time(deadline)
            visited_cols[current_col] = True
            current_row = matched_row[current_col]
            potential_step = float("inf")
            next_col = -1
            for col in range(1, matrix_size + 1):
                if not visited_cols[col]:
                    reduced_cost = (
                        cost_matrix[current_row - 1][col - 1]
                        - row_potential[current_row] - col_potential[col]
                    )
                    if reduced_cost < min_slack[col]:
                        min_slack[col] = reduced_cost
                        previous_col[col] = current_col
                    if min_slack[col] < potential_step:
                        potential_step = min_slack[col]
                        next_col = col
            for col in range(0, matrix_size + 1):
                if visited_cols[col]:
                    row_potential[matched_row[col]] += potential_step
                    col_potential[col] -= potential_step
                else:
                    min_slack[col] -= potential_step
            current_col = next_col
            if matched_row[current_col] == 0:
                break
        while True:
            next_col = previous_col[current_col]
            matched_row[current_col] = matched_row[next_col]
            current_col = next_col
            if current_col == 0:
                break

    assignment = [0] * matrix_size
    total_cost = 0.0
    for col in range(1, matrix_size + 1):
        if matched_row[col] != 0:
            assignment[matched_row[col] - 1] = col - 1
            total_cost += cost[matched_row[col] - 1][col - 1]
    return total_cost, assignment

def _is_frozen_deadlock(boxes, board):
   """Xác định một tập hợp các khối bị khóa lẫn nhau; một khối lân cận cố định sẽ chặn toàn bộ một trục.

    Loại bỏ các khối có trục còn khả năng di chuyển cho đến khi tập hợp này không còn thay đổi.
    Khi đó, không còn khối nào có thể được đẩy trước: trên mỗi trục, vị trí đích của khối hoặc ô đứng của robot đều đã bị chặn bởi một bức tường hoặc một khối khác.
    Các ô đích (goal boxes) đóng vai trò là vật cản, nhưng bản thân chúng không được coi là trạng thái bế tắc (deadlock).
    """
    width = board.width
    height = board.height
    obstacles = board.obstacles
    storage = board.storage

    def is_wall(x, y):
        return x < 0 or x >= width or y < 0 or y >= height or (x, y) in obstacles

    frozen = set(boxes)

    def blocked(x, y):
        return is_wall(x, y) or (x, y) in frozen

    while frozen:
        removable = set()
        for x, y in frozen:
            _check_time(board.deadline)
            horizontal = blocked(x - 1, y) or blocked(x + 1, y)
            vertical = blocked(x, y - 1) or blocked(x, y + 1)
            if not (horizontal and vertical):
                removable.add((x, y))
        if not removable:
            return bool(frozen - storage)
        frozen.difference_update(removable)
    return False

