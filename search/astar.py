"""
search/astar.py  -  Iteration 5 (Task 3): A* Search

Informed. Expands the frontier cell with the smallest

        f(n) = g(n) + h(n)

  g(n) : actual cost from the start to n (sum of problem.step_cost)
  h(n) : Manhattan-distance estimate from n to the goal

Because h is admissible and consistent (see heuristics.py), A* returns a
path with the LOWEST total cost. Whatever problem.step_cost says - plain
moves now, crop-safety penalties in Iteration 6 - A* optimises that.

Properties:
  complete : yes
  optimal  : yes (admissible + consistent heuristic)
  time/space: exponential worst case, but a good h prunes most of the grid
"""

import heapq
import itertools
import time

from search.common import SearchResult, reconstruct_path
from search.heuristics import manhattan


def astar(problem, heuristic=manhattan, name="A*"):
    t0 = time.perf_counter()

    start, goal = problem.initial_state, problem.goal
    counter = itertools.count()
    g = {start: 0}                   # best known cost from start to each cell
    parent = {start: None}
    # entries: (f, h, tie-breaker, g, cell). On equal f, prefer smaller h
    # (the cell nearer the goal) -> fewer expansions.
    h0 = heuristic(start, goal)
    frontier = [(h0, h0, next(counter), 0, start)]
    closed = set()
    expanded = []

    while frontier:
        _, _, _, g_state, state = heapq.heappop(frontier)
        if state in closed:          # stale entry: a cheaper route was found later
            continue
        closed.add(state)
        expanded.append(state)

        if problem.goal_test(state):
            path = reconstruct_path(parent, state)
            return SearchResult(name, True, path, problem.path_cost(path),
                                len(expanded), (time.perf_counter() - t0) * 1000, expanded)

        for action, nxt in problem.successors(state):
            if nxt in closed:
                continue
            new_g = g_state + problem.step_cost(state, action, nxt)
            if new_g < g.get(nxt, float("inf")):   # found a cheaper way to nxt
                g[nxt] = new_g
                parent[nxt] = state
                h = heuristic(nxt, goal)
                heapq.heappush(frontier, (new_g + h, h, next(counter), new_g, nxt))

    return SearchResult(name, False, [], float("inf"),
                        len(expanded), (time.perf_counter() - t0) * 1000, expanded)


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    from field import build_default_field, draw_field
    from problem import PathProblem

    field = build_default_field()
    result = astar(PathProblem(field))
    result.report()
    draw_field(field, path=result.path, explored=result.expanded,
               title=f"A* - {result.path_length} moves, {result.nodes_expanded} nodes expanded",
               save_path="screenshots/05_astar.png")
    plt.show()