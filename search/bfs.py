"""
search/bfs.py  -  Iteration 3 (Task 3): Breadth-First Search

Uninformed. Explores the field in "rings": every cell 1 move away, then
every cell 2 moves away, and so on. The frontier is a FIFO queue.

Properties (4-directional grid, every move costs 1):
  complete : yes
  optimal  : yes for number of moves (not for weighted costs)
  time     : O(b^d)   space: O(b^d)
"""

import time
from collections import deque

from search.common import SearchResult, reconstruct_path


def bfs(problem):
    t0 = time.perf_counter()

    start = problem.initial_state
    frontier = deque([start])        # FIFO queue
    parent = {start: None}           # also acts as the "reached" set
    expanded = []

    while frontier:
        state = frontier.popleft()
        expanded.append(state)

        if problem.goal_test(state):
            path = reconstruct_path(parent, state)
            return SearchResult("BFS", True, path, problem.path_cost(path),
                                len(expanded), (time.perf_counter() - t0) * 1000, expanded)

        for _, nxt in problem.successors(state):
            if nxt not in parent:    # never queue the same cell twice
                parent[nxt] = state
                frontier.append(nxt)

    return SearchResult("BFS", False, [], float("inf"),
                        len(expanded), (time.perf_counter() - t0) * 1000, expanded)


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    from field import build_default_field, draw_field
    from problem import PathProblem

    field = build_default_field()
    result = bfs(PathProblem(field))
    result.report()
    draw_field(field, path=result.path, explored=result.expanded,
               title=f"BFS - {result.path_length} moves, {result.nodes_expanded} nodes expanded",
               save_path="screenshots/03_bfs.png")
    plt.show()