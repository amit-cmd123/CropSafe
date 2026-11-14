"""
search/ucs.py  -  extra baseline: Uniform Cost Search (UCS)

Uninformed. Always expands the frontier cell with the smallest path cost
so far, g(n). It is exactly A* with the heuristic switched off (h = 0):

        A*  : f(n) = g(n) + h(n)
        UCS : f(n) = g(n)

Why it is in the project:
  - BFS is optimal only when every move costs the same. With the crop-safe
    cost, moves cost different amounts, so BFS can return a costlier route.
  - UCS is optimal for any non-negative step costs, but it has no sense of
    direction, so it expands more nodes than A*.
  -> Comparing UCS with A* shows how much work the Manhattan heuristic saves.

Properties:
  complete : yes
  optimal  : yes (any step costs >= 0)
"""

from search.astar import astar


def zero_heuristic(cell, goal):
    return 0


def ucs(problem):
    return astar(problem, heuristic=zero_heuristic, name="UCS")


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    from field import build_default_field, draw_field
    from problem import PathProblem
    from search.bfs import bfs

    field = build_default_field()
    problem = PathProblem(field, crop_safe=True)
    u, a, b = ucs(problem), astar(problem, name="A* (crop-safe)"), bfs(problem)
    for r in (b, u, a):
        print(f"{r.algorithm:<15} crop-safe cost {problem.path_cost(r.path):>4.0f} | "
              f"moves {r.path_length:>3} | expanded {r.nodes_expanded:>4}")
    draw_field(field, path=u.path, explored=u.expanded,
               title=f"UCS (crop-safe cost): {u.path_length} moves, {u.nodes_expanded} nodes expanded",
               save_path="screenshots/05_ucs.png")
    plt.show()