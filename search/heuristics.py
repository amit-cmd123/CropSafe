"""
search/heuristics.py  -  Iteration 4 (Task 3): heuristic for informed search

Manhattan distance:  h(n) = |row_n - row_goal| + |col_n - col_goal|

Why Manhattan?
  - The robot moves only Up/Down/Left/Right, one cell per move.
  - Even on an empty field it needs at least |dr| + |dc| moves.
  - Every move costs at least 1, so h(n) never overestimates the true
    remaining cost  ->  h is ADMISSIBLE.
  - For neighbours n, n':  h(n) <= 1 + h(n') <= cost(n, n') + h(n')
    ->  h is CONSISTENT (monotone). This matters for A* in Iteration 5.
"""


def manhattan(cell, goal):
    return abs(cell[0] - goal[0]) + abs(cell[1] - goal[1])