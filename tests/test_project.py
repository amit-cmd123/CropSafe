"""
tests/test_project.py  -  automated checks for the whole project

Run from the project folder:
    python tests/test_project.py        (no extra libraries needed)
    python -m pytest tests              (if pytest is installed)

Each test_ function checks one claim the README makes. Random tests use a
fixed seed, so every run checks exactly the same fields.
"""

import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))   # project folder

from field import build_default_field, OBSTACLE, SAFE
from problem import PathProblem
from replanning import navigate
from search.astar import astar
from search.bfs import bfs
from search.dfs import dfs
from search.greedy import greedy
from search.heuristics import manhattan
from search.ucs import ucs

ALGORITHMS = [bfs, dfs, greedy, astar, ucs]
SEED = 7


def random_field(rng, known=25, hidden=3):
    """Default field plus random known obstacles and random hidden obstacles."""
    f = build_default_field()
    f.hidden_obstacles = set()
    free = [(r, c) for r in range(f.rows) for c in range(f.cols)
            if f.grid[r, c] == SAFE and (r, c) not in (f.start, f.goal)]
    cells = rng.sample(free, known + hidden)
    for cell in cells[:known]:
        f.grid[cell] = OBSTACLE
    f.hidden_obstacles = set(cells[known:])
    return f


def assert_valid_path(field, path):
    """Starts at S, ends at G, one 4-directional step at a time, only on safe cells."""
    assert path[0] == field.start and path[-1] == field.goal, "path must go from start to goal"
    for a, b in zip(path, path[1:]):
        assert abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1, f"illegal jump {a} -> {b}"
    for cell in path:
        assert field.grid[cell] == SAFE, f"path enters a non-safe cell {cell}"


# ----------------------------------------------------------------------
def test_every_algorithm_gives_a_valid_crop_safe_path():
    field = build_default_field()
    for crop_safe in (False, True):
        problem = PathProblem(field, crop_safe=crop_safe)
        for algorithm in ALGORITHMS:
            result = algorithm(problem)
            assert result.found, f"{result.algorithm} did not reach the goal"
            assert_valid_path(field, result.path)
            assert len(set(result.expanded)) == len(result.expanded), "a cell was expanded twice"


def test_optimality_on_random_fields():
    """BFS and plain A* agree on the fewest moves; UCS and crop-safe A* agree on the lowest cost."""
    rng = random.Random(SEED)
    for _ in range(200):
        f = random_field(rng, hidden=0)
        plain, safe = PathProblem(f), PathProblem(f, crop_safe=True)
        b, a = bfs(plain), astar(plain)
        assert b.found == a.found
        if not b.found:
            continue
        assert b.path_length == a.path_length, "A* must match BFS on uniform costs"
        u, s = ucs(safe), astar(safe)
        assert safe.path_cost(u.path) == safe.path_cost(s.path), "A* must match UCS on crop-safe costs"
        for other in (bfs(safe), dfs(safe), greedy(safe)):
            assert safe.path_cost(other.path) >= safe.path_cost(s.path), "nothing may beat optimal A*"


def test_manhattan_heuristic_is_admissible():
    """h(n) never exceeds the true cheapest crop-safe cost from n to the goal."""
    field = build_default_field()
    for r in range(field.rows):
        for c in range(field.cols):
            if field.grid[r, c] != SAFE:
                continue
            problem = PathProblem(field, crop_safe=True)
            problem.initial_state = (r, c)
            best = ucs(problem)
            if best.found:
                assert manhattan((r, c), field.goal) <= problem.path_cost(best.path)


def test_unreachable_goal_is_reported_not_crashed():
    field = build_default_field()
    for cell in [(3, 16), (5, 16), (4, 15), (4, 17)]:      # wall the goal in
        field.grid[cell] = OBSTACLE
    for algorithm in ALGORITHMS:
        assert not algorithm(PathProblem(field)).found
    assert not navigate(field).reached


def test_default_replanning_scenario():
    nav = navigate(build_default_field())
    assert nav.reached and nav.replans == 1 and nav.moves == 25
    step, position, cell = nav.detections[0]
    assert (step, position, cell) == (18, (5, 10), (4, 11))


def test_replanning_never_hits_an_obstacle():
    """On random fields the robot never enters a real obstacle, and reaches the goal exactly when it can."""
    rng = random.Random(SEED)
    for _ in range(200):
        f = random_field(rng, known=20, hidden=6)
        nav = navigate(f)                          # raises if it ever drives into an obstacle
        real = f.true_grid()
        assert all(real[cell] == SAFE for cell in nav.trail)
        oracle = f.copy()
        oracle.grid = real
        oracle.hidden_obstacles = set()
        assert nav.reached == astar(PathProblem(oracle)).found


def test_safer_route_can_be_longer():
    """Goal (1, 16): crop-safe A* accepts 2 extra moves to avoid running beside a potato row."""
    field = build_default_field()
    field.set_goal((1, 16))
    scorer = PathProblem(field, crop_safe=True)
    plain, safe = astar(PathProblem(field)), astar(scorer)
    assert safe.path_length > plain.path_length
    assert scorer.cells_near_crops(safe.path) < scorer.cells_near_crops(plain.path)


def test_path_cost_rejects_an_invalid_path():
    try:
        PathProblem(build_default_field()).path_cost([(13, 0), (11, 0)])
    except ValueError:
        return
    raise AssertionError("path_cost accepted a path that jumps two cells")


# ----------------------------------------------------------------------
if __name__ == "__main__":
    import time
    tests = [(name, fn) for name, fn in sorted(globals().items()) if name.startswith("test_") and callable(fn)]
    failed = 0
    for name, fn in tests:
        t0 = time.perf_counter()
        try:
            fn()
            print(f"PASS  {name}  ({time.perf_counter() - t0:.1f} s)")
        except Exception as error:                 # report every failure, keep going
            failed += 1
            print(f"FAIL  {name}: {type(error).__name__}: {error}")
    print(f"\n{len(tests) - failed} passed, {failed} failed")
    sys.exit(1 if failed else 0)