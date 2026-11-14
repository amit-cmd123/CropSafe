"""
problem.py  -  Iteration 2 (Task 2: Problem formulation)

Turns the field into a formal search problem:

  State             : the robot's cell (row, col)
  Initial state     : field.start
  Actions           : Up, Down, Left, Right (4-directional, one cell per move)
  Transition model  : result(s, a) = s moved one cell in direction a
  Goal test         : state == field.goal
  Path cost         : sum of step costs; every move costs 1 for now
                      (Iteration 6 adds a crop-safety penalty)

A move is allowed only if the next cell is inside the grid and is a SAFE
cell on the robot's known map. Potato rows, crop-protection zones and
obstacles can never be entered.
"""

from field import build_default_field

# Order matters: BFS and DFS try actions in this order, so their results
# are repeatable from run to run.
ACTIONS = {
    "Up": (-1, 0),
    "Down": (1, 0),
    "Left": (0, -1),
    "Right": (0, 1),
}


class PathProblem:
    def __init__(self, field):
        self.field = field
        self.initial_state = field.start
        self.goal = field.goal

    # ---------------- transition model ----------------
    def result(self, state, action):
        """The cell reached by doing 'action' in 'state' (not checked for safety)."""
        dr, dc = ACTIONS[action]
        return (state[0] + dr, state[1] + dc)

    def actions(self, state):
        """Legal actions in 'state': the move must stay on the grid and land on a SAFE cell."""
        legal = []
        for action in ACTIONS:
            nxt = self.result(state, action)
            if self.field.in_bounds(nxt) and not self.field.is_blocked(nxt):
                legal.append(action)
        return legal

    def successors(self, state):
        """(action, next_state) pairs. The search algorithms use this."""
        return [(a, self.result(state, a)) for a in self.actions(state)]

    # ---------------- goal test ----------------
    def goal_test(self, state):
        return state == self.goal

    # ---------------- path cost ----------------
    def step_cost(self, state, action, next_state):
        """Cost of one move. Uniform for now."""
        return 1

    def path_cost(self, path):
        """Total cost of a path given as a list of cells [start, ..., end]."""
        total = 0
        for a, b in zip(path, path[1:]):
            action = next(name for name, d in ACTIONS.items()
                          if (a[0] + d[0], a[1] + d[1]) == b)
            total += self.step_cost(a, action, b)
        return total

    def describe(self):
        print("PROBLEM FORMULATION")
        print(f"  State            : robot cell (row, col), grid {self.field.rows} x {self.field.cols}")
        print(f"  Initial state    : {self.initial_state}")
        print(f"  Goal state       : {self.goal}")
        print(f"  Actions          : {list(ACTIONS)}  (4-directional)")
        print("  Transition model : result((r, c), a) = (r + dr, c + dc)")
        print("  Legal move       : next cell in bounds and SAFE (no potato, protected or obstacle cell)")
        print("  Goal test        : state == goal")
        print("  Path cost        : number of moves (each move costs 1)")


if __name__ == "__main__":
    field = build_default_field()
    problem = PathProblem(field)
    problem.describe()

    s = problem.initial_state
    print(f"\nLegal actions at start {s}: {problem.actions(s)}")
    for a, nxt in problem.successors(s):
        print(f"  {a:<5} -> {nxt}")

    # A cell just above a potato row: Down must be illegal because (6, 3) is a plant
    probe = (5, 3)
    print(f"\nLegal actions at {probe} (potato row just below): {problem.actions(probe)}")

    # Goal test
    print(f"\ngoal_test({s}) = {problem.goal_test(s)}")
    print(f"goal_test({problem.goal}) = {problem.goal_test(problem.goal)}")

    # Path cost of a hand-written 3-move path
    manual = [(13, 0), (12, 0), (11, 0), (11, 1)]
    print(f"\nPath cost of {manual} = {problem.path_cost(manual)}")