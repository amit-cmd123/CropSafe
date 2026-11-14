"""
search/greedy.py  -  Iteration 4 (Task 3): Greedy Best-First Search

Informed. Always expands the frontier cell that LOOKS closest to the goal,
using only the heuristic:   f(n) = h(n)   (Manhattan distance).

It ignores how far the robot has already travelled (g(n)), so it is fast
but can be fooled by obstacles and crop rows into a longer path.

Properties:
  complete : yes on a finite grid (graph search with a visited set)
  optimal  : no
  time/space: O(b^m) worst case, usually far fewer nodes with a good h
"""

import heapq
import itertools
import time

from search.common import SearchResult, reconstruct_path
from search.heuristics import manhattan


def greedy(problem, heuristic=manhattan):
    t0 = time.perf_counter()

    start, goal = problem.initial_state, problem.goal
    counter = itertools.count()      # tie-breaker: equal h -> first pushed comes first
    frontier = [(heuristic(start, goal), next(counter), start, None)]
    parent = {}
    visited = set()
    expanded = []

    while frontier:
        _, _, state, came_from = heapq.heappop(frontier)   # smallest h
        if state in visited:
            continue
        visited.add(state)
        parent[state] = came_from
        expanded.append(state)

        if problem.goal_test(state):
            path = reconstruct_path(parent, state)
            return SearchResult("Greedy", True, path, problem.path_cost(path),
                                len(expanded), (time.perf_counter() - t0) * 1000, expanded)

        for _, nxt in problem.successors(state):
            if nxt not in visited:
                heapq.heappush(frontier, (heuristic(nxt, goal), next(counter), nxt, state))

    return SearchResult("Greedy", False, [], float("inf"),
                        len(expanded), (time.perf_counter() - t0) * 1000, expanded)


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    from field import build_default_field, draw_field
    from problem import PathProblem

    field = build_default_field()
    result = greedy(PathProblem(field))
    result.report()
    draw_field(field, path=result.path, explored=result.expanded,
               title=f"Greedy Best-First - {result.path_length} moves, {result.nodes_expanded} nodes expanded",
               save_path="screenshots/04_greedy.png")
    plt.show()