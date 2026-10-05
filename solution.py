"""Student entry point. See README.md for the API and scoring rules."""

import heapq
import time
from sokoban import sokoban_goal_state

def heuristic(state):
    total = 0
    for box in state.boxes:
        if state.storage:
            total += min(
                abs(box[0] - s[0]) + abs(box[1] - s[1])
                for s in state.storage
            )
    return total


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

        if sokoban_goal_state(cur):
            return cur

        for succ in cur.successors():
            sk = succ.hashable_state()
            sg = succ.gval
            if sg < closed.get(sk, float('inf')):
                closed[sk] = sg
                f = sg + weight * heuristic(succ)
                heapq.heappush(open_list, (f, counter, succ))
                counter += 1

    return False

def two_tier_search(initial_state, timebound, weight=2.0, split=0.4):
    budget1 = timebound * split
    budget2 = timebound - budget1

    # --- Tier 1: Weighted A* (nhanh) ---
    result = weighted_a_star(initial_state, timebound=budget1, weight=weight)
    if result:
        return result

    # --- Tier 2: standard A* (optimal) ---
    return weighted_a_star(initial_state, timebound=budget2, weight=1.0)

def solve(initial_state, timebound=120):
    return two_tier_search(initial_state, timebound=timebound)

