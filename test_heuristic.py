"""Tests for the current push-distance heuristic.

Run: python test_heuristic.py
Admissibility means h(state) <= the optimal remaining move count.
These finite checks do not prove admissibility for every possible board.
"""
import unittest
import time
from collections import deque
from itertools import permutations

from sokoban import SokobanState, sokoban_goal_state
from solution import _Board
from validation import signature


def make_state(robots, boxes, storage, width=4, height=3, obstacles=()):
    """Build a valid test board; reject accidental overlapping pieces."""
    robots = tuple(robots)
    boxes = frozenset(boxes)
    storage = frozenset(storage)
    obstacles = frozenset(obstacles)
    cells = set(robots) | boxes | storage | obstacles
    if not robots or len(set(robots)) != len(robots):
        raise ValueError("Robots must be nonempty and distinct")
    if set(robots) & boxes or (set(robots) | boxes | storage) & obstacles:
        raise ValueError("Pieces must not overlap boxes or walls")
    if len(storage) < len(boxes):
        raise ValueError("There must be at least as many goals as boxes")
    if any(not (0 <= x < width and 0 <= y < height) for x, y in cells):
        raise ValueError("Coordinates must be inside the board")
    return SokobanState("TEST", 0, None, width, height,
                        robots, boxes, storage, obstacles)


def estimate(state, cache=None):
    deadline = time.perf_counter() + 10
    board = _Board(state, deadline)
    if cache is not None:
        board.assignment_cache = cache
    return board.estimate(state.robots, state.boxes, deadline)


def optimal_moves(initial_state, max_states=50000):
    """Find exact cost using unit-step BFS, without solver heuristics/pruning.

    Return None only when the reachable state space is exhausted.
    A safety-limit failure must not be mistaken for an unsolvable board.
    """
    queue = deque([initial_state])
    seen = {initial_state.hashable_state()}
    while queue:
        state = queue.popleft()
        if sokoban_goal_state(state):
            return state.gval - initial_state.gval
        for successor in state.successors():
            key = successor.hashable_state()
            if key in seen:
                continue
            seen.add(key)
            if len(seen) > max_states:
                raise AssertionError("BFS safety limit exceeded; use a smaller board")
            queue.append(successor)
    return None


class HeuristicTests(unittest.TestCase):
    def assert_admissible(self, state):
        optimum = optimal_moves(state)
        self.assertIsNotNone(optimum, "This fixture must be solvable")
        value = estimate(state)
        self.assertGreaterEqual(value, 0)
        self.assertLessEqual(value, optimum)
        return optimum

    def test_goal_and_extra_storage(self):
        state = make_state([(0, 0)], [(1, 1)], [(1, 1), (3, 2)])
        self.assertEqual(estimate(state), 0)
        self.assertEqual(optimal_moves(state), 0)

    def test_one_push(self):
        state = make_state([(0, 1)], [(1, 1)], [(2, 1)])
        self.assertEqual(self.assert_admissible(state), 1)
        self.assertEqual(estimate(state), 1)

    def test_walking_cost(self):
        state = make_state([(0, 0)], [(2, 1)], [(3, 1)])
        self.assertEqual(self.assert_admissible(state), 3)
        self.assertEqual(estimate(state), 3)

    def test_solvable_wall_detour(self):
        state = make_state([(1, 1)], [(2, 1)], [(4, 2)],
                           width=5, height=4, obstacles=[(3, 1)])
        self.assert_admissible(state)

    def test_multiple_boxes_and_robots(self):
        state = make_state([(0, 0), (0, 2)], [(1, 0), (1, 2)],
                           [(2, 0), (2, 2)])
        self.assertEqual(self.assert_admissible(state), 2)
        self.assertEqual(estimate(state), 2)

    def test_partially_solved_with_extra_storage(self):
        state = make_state([(0, 1)], [(1, 0), (1, 1)],
                           [(1, 0), (2, 1), (3, 2)])
        self.assertEqual(self.assert_admissible(state), 1)

    def test_static_deadlocks(self):
        # The old tests treated these as walking detours, ignoring push support.
        boards = [
            make_state([(3, 0)], [(0, 1)], [(4, 1)],
                       width=5, height=3, obstacles=[(1, 1)]),
            make_state([(0, 0)], [(2, 0)], [(4, 0)],
                       width=5, height=3, obstacles=[(3, 0)]),
        ]
        for state in boards:
            with self.subTest(boxes=state.boxes):
                self.assertIsNone(optimal_moves(state))
                self.assertEqual(estimate(state), float("inf"))

    def test_cache_preserves_robot_distance_and_input(self):
        near = make_state([(1, 1)], [(2, 1)], [(3, 1)])
        far = make_state([(0, 0)], [(2, 1)], [(3, 1)])
        cache = {}
        for state, expected in [(near, 1), (far, 3), (near, 1)]:
            before = (signature(state), state.action, state.gval, state.parent)
            self.assertEqual(estimate(state, cache), expected)
            self.assertEqual(estimate(state, cache), estimate(state))
            self.assertEqual(
                (signature(state), state.action, state.gval, state.parent), before)

    def test_all_single_box_positions_on_small_boards(self):
        cells = [(x, y) for x in range(3) for y in range(3)]
        solved_cases = 0
        for goal in [(0, 0), (1, 1), (2, 1)]:
            for robot, box in permutations(cells, 2):
                state = make_state([robot], [box], [goal], width=3, height=3)
                optimum = optimal_moves(state)
                if optimum is None:
                    continue
                with self.subTest(goal=goal, robot=robot, box=box):
                    value = estimate(state)
                    self.assertGreaterEqual(value, 0)
                    self.assertLessEqual(value, optimum)
                solved_cases += 1
        self.assertGreater(solved_cases, 0)

    def test_fixture_rejects_robot_on_box(self):
        with self.assertRaises(ValueError):
            make_state([(0, 0)], [(0, 0)], [(2, 1)])

    def test_full_matching_detects_competing_goals(self):
        state = make_state([(0,0)], [(1,1),(1,2)], [(1,2),(5,2)],
                           width=7, height=4, obstacles=[(3,y) for y in range(4)])
        self.assertIsNone(optimal_moves(state))
        self.assertEqual(estimate(state), float('inf'))


if __name__ == "__main__":
    unittest.main(verbosity=2)
