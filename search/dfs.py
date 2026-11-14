"""
search/dfs.py  -  Iteration 3 (Task 3): Depth-First Search

Uninformed. Always goes as deep as possible along one branch before
backing up. The frontier is a LIFO stack.

Graph-search version: a cell is marked visited when it is expanded, so
no cell is expanded twice and DFS cannot loop forever.

Properties:
  complete : yes on a finite grid (because of the visited set)
  optimal  : no - the path is often long and winding
  time     : O(b^m)   space: O(b*m) for the stack (m = max depth)
"""

import time

from search.common import SearchResult, reconstruct_path


def dfs(problem):
    t0 = time.perf_counter()

    start = problem.initial_state
    stack = [(start, None)]          # (cell, the cell we came from)
    parent = {}
    visited = set()
    expanded = []

    while stack:
        state, came_from = stack.pop()
        if state in visited:         # already expanded via another route
            continue
        visited.add(state)
        parent[state] = came_from
        expanded.append(state)

        if problem.goal_test(state):
            path = reconstruct_path(parent, state)
            return SearchResult("DFS", True, path, problem.path_cost(path),
                                len(expanded), (time.perf_counter() - t0) * 1000, expanded)

        # push in reverse so the FIRST action (Up) is popped and tried first
        for _, nxt in reversed(problem.successors(state)):
            if nxt not in visited:
                stack.append((nxt, state))

    return SearchResult("DFS", False, [], float("inf"),
                        len(expanded), (time.perf_counter() - t0) * 1000, expanded)


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    from field import build_default_field, draw_field
    from problem import PathProblem

    field = build_default_field()
    result = dfs(PathProblem(field))
    result.report()
    draw_field(field, path=result.path, explored=result.expanded,
               title=f"DFS - {result.path_length} moves, {result.nodes_expanded} nodes expanded",
               save_path="screenshots/03_dfs.png")
    plt.show()