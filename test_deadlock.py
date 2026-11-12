"""Deadlock regression cases for the selected solver in solution.py.

All fixtures obey board constraints. A false result from deadlocked() alone
is not a proof of solvability; positive solvability checks replay a solution.
"""
import copy
import time
import unittest
from itertools import combinations

import solution as solver
from test_heuristic import make_state, optimal_moves
from validation import signature, validate_solution


def board_for(state):
    return solver._Board(state, time.perf_counter()+10)


class DeadlockTests(unittest.TestCase):
    def test_one_wall_per_axis_freezes_corner(self):
        state = make_state([(4, 4)], [(2, 2)], [(4, 2)], width=6, height=6,
                           obstacles=[(1, 2), (2, 1)])
        self.assertTrue(solver._is_frozen_deadlock(state.boxes, board_for(state)))
        self.assertIsNone(optimal_moves(state))

    def test_zigzag_freeze_without_static_deadlock(self):
        boxes = [(2, 2), (3, 2), (3, 3)]
        walls = [(2, 1), (4, 3)]
        state = make_state([(0, 0)], boxes, [(1, 1), (4, 1), (4, 4)],
                           width=6, height=6, obstacles=walls)
        board = board_for(state)
        self.assertFalse(state.boxes & board.dead_cells)
        self.assertTrue(solver._is_frozen_deadlock(state.boxes, board))
        self.assertTrue(board.deadlocked(state.boxes))
        self.assertIsNone(optimal_moves(state))
        # Frozen goal boxes remain blockers for a non-goal member of the group.
        partial = make_state([(0, 0)], boxes, [(2, 2), (3, 2), (4, 4)],
                             width=6, height=6, obstacles=walls)
        self.assertTrue(solver._is_frozen_deadlock(partial.boxes, board_for(partial)))
        completed = make_state([(0, 0)], boxes, boxes, width=6, height=6,
                               obstacles=walls)
        self.assert_solvable_not_pruned(completed)

    def test_small_board_freeze_claims_against_bfs(self):
        cells = [(x, y) for y in range(3) for x in range(3)]
        checked = 0
        for walls in [(), ((1, 0),), ((1, 1),), ((0, 1), (1, 0))]:
            floor = [cell for cell in cells if cell not in walls]
            goals = [floor[0], floor[-1]]
            for boxes in combinations(floor, 2):
                for robot in floor:
                    if robot in boxes:
                        continue
                    state = make_state([robot], boxes, goals, width=3, height=3,
                                       obstacles=walls)
                    if solver._is_frozen_deadlock(state.boxes, board_for(state)):
                        with self.subTest(walls=walls, boxes=boxes, robot=robot):
                            self.assertIsNone(optimal_moves(state))
                        checked += 1
        self.assertGreater(checked, 0)

    def assert_solvable_not_pruned(self, state):
        reference = copy.deepcopy(state)
        before = (signature(state), state.action, state.gval, state.parent)
        self.assertFalse(board_for(state).deadlocked(state.boxes))
        result = solver.solve(state, 2)
        self.assertIsNot(result, False)
        valid, message = validate_solution(reference, result)
        self.assertTrue(valid, message)
        self.assertEqual(before, (signature(state), state.action, state.gval, state.parent))

    def test_corner_and_goal_corner(self):
        stuck = make_state([(2,2)],[(0,0)],[(3,3)],width=4,height=4)
        self.assertTrue(board_for(stuck).deadlocked(stuck.boxes))
        self.assertIsNone(optimal_moves(stuck))
        goal = make_state([(2,2)],[(0,0)],[(0,0)],width=4,height=4)
        self.assert_solvable_not_pruned(goal)

    def test_edge_without_goal(self):
        state = make_state([(3,2)],[(0,0),(1,0),(2,0)],
                           [(1,3),(3,3),(4,3)],width=6,height=5)
        self.assertTrue(board_for(state).deadlocked(state.boxes))

    def test_wall_separates_box_and_goal(self):
        state = make_state([(1,0)],[(1,1)],[(4,1)],width=5,height=3,
                           obstacles=[(2,y) for y in range(3)])
        self.assertIn((1,1),board_for(state).dead_cells)
        self.assertTrue(board_for(state).deadlocked(state.boxes))
        self.assertIsNone(optimal_moves(state))

    def test_four_box_square_and_completed_square(self):
        boxes=[(1,1),(2,1),(1,2),(2,2)]
        state=make_state([(0,0)],boxes,[(0,0),(4,0),(0,4),(4,4)],width=5,height=5)
        board=board_for(state)
        self.assertFalse(state.boxes & board.dead_cells)
        self.assertTrue(solver._is_frozen_deadlock(state.boxes,board))
        self.assertTrue(board.deadlocked(state.boxes))
        goal=make_state([(0,0)],boxes,boxes,width=5,height=5)
        self.assert_solvable_not_pruned(goal)

    def test_box_and_three_walls_square(self):
        state=make_state([(3,3)],[(1,1)],[(3,3)],width=4,height=4,
                         obstacles=[(2,1),(1,2),(2,2)])
        board=board_for(state)
        self.assertTrue(solver._is_frozen_deadlock(state.boxes,board))
        self.assertTrue(board.deadlocked(state.boxes))

    def test_box_surrounded_by_walls(self):
        state=make_state([(3,3)],[(1,1)],[(4,1)],width=5,height=5,
                         obstacles=[(0,1),(1,0),(2,1),(1,2)])
        self.assertTrue(solver._is_frozen_deadlock(state.boxes,board_for(state)))
        self.assertTrue(board_for(state).deadlocked(state.boxes))

    def test_one_axis_blocked_can_still_solve(self):
        cases=[
            make_state([(0,0)],[(1,1)],[(1,3)],width=3,height=4,obstacles=[(0,1),(2,1)]),
            make_state([(0,1)],[(1,1)],[(4,1)],width=5,height=5,obstacles=[(1,0),(1,2)]),
            make_state([(0,2)],[(1,1)],[(4,1)],width=5,height=5,obstacles=[(1,0)]),
        ]
        for state in cases:
            with self.subTest(obstacles=state.obstacles):
                self.assert_solvable_not_pruned(state)

    def test_neighboring_boxes_can_move_away(self):
        state=make_state([(6,4)],[(1,2),(2,2),(3,2)],[(1,3),(3,1),(3,2)],
                         width=7,height=5,obstacles=[(0,2),(4,2),(2,1),(2,3)])
        self.assert_solvable_not_pruned(state)

    def test_deadlock_cache_preserves_results(self):
        state=make_state([(0,1)],[(1,1)],[(2,1)])
        board=board_for(state)
        self.assertFalse(board.deadlocked(state.boxes))
        self.assertIn(state.boxes,board.deadlock_cache)
        self.assertFalse(board.deadlocked(state.boxes))


if __name__ == '__main__':
    unittest.main(verbosity=2)
