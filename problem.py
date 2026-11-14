"""
problem.py  -  Iteration 2 (Task 2) + Iteration 6 (Task 4: crop-safe cost)

Turns the field into a formal search problem:

  State             : the robot's cell (row, col)
  Initial state     : field.start
  Actions           : Up, Down, Left, Right (4-directional, one cell per move)
  Transition model  : result(s, a) = s moved one cell in direction a
  Goal test         : state == field.goal
  Path cost         : sum of step costs
                        plain     : every move costs 1
                        crop-safe : 1 + proximity penalty of the cell entered

A move is allowed only if the next cell is inside the grid and is a SAFE
cell on the robot's known map. Potato rows, crop-protection zones and
obstacles can never be entered (hard constraint).

Crop-safe planning (soft constraint):
  Entering a cell that is close to a potato or protected cell costs extra.
  "Close" uses the 8-neighbourhood (Chebyshev distance), so diagonal
  neighbours count too - a robot's body/wheels can brush a plant diagonally.
      distance 1 from a crop cell  -> +3
      distance 2 from a crop cell  -> +1
      further away                 -> +0
  The penalty never makes a step cost less than 1, so the Manhattan
  heuristic stays admissible and consistent and A* stays optimal.
"""

import numpy as np

from field import build_default_field, POTATO, PROTECTED

# Order matters: BFS and DFS try actions in this order, so their results
# are repeatable from run to run.
ACTIONS = {
    "Up": (-1, 0),
    "Down": (1, 0),
    "Left": (0, -1),
    "Right": (0, 1),
}

# distance-to-crop -> extra cost
DEFAULT_PENALTIES = {1: 3, 2: 1}


def proximity_penalty_map(field, penalties=DEFAULT_PENALTIES):
    """
    For every cell, the extra cost of entering it because it is near crops.
    Computed once up front so step_cost is just a lookup.
    """
    crop = np.isin(field.grid, (POTATO, PROTECTED))
    crop_cells = np.argwhere(crop)
    penalty = np.zeros(field.grid.shape, dtype=float)
    if len(crop_cells) == 0:
        return penalty

    for r in range(field.rows):
        for c in range(field.cols):
            # Chebyshev distance to the nearest crop cell
            d = np.min(np.maximum(np.abs(crop_cells[:, 0] - r), np.abs(crop_cells[:, 1] - c)))
            penalty[r, c] = penalties.get(int(d), 0)
    return penalty


class PathProblem:
    def __init__(self, field, crop_safe=False, penalties=DEFAULT_PENALTIES):
        self.field = field
        self.initial_state = field.start
        self.goal = field.goal
        self.crop_safe = crop_safe
        self.penalty = proximity_penalty_map(field, penalties) if crop_safe else None

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
        """Cost of one move: 1, plus the crop-proximity penalty when crop_safe is on."""
        if self.crop_safe:
            return 1 + self.penalty[next_state]
        return 1

    def path_cost(self, path):
        """Total cost of a path given as a list of cells [start, ..., end]."""
        total = 0
        for a, b in zip(path, path[1:]):
                        action = next((name for name, d in ACTIONS.items()
                                    if (a[0] + d[0], a[1] + d[1]) == b), None)
                        if action is None:
                             raise ValueError(f"Invalid path: {a} -> {b} is not a single Up/Down/Left/Right move")
                        total += self.step_cost(a, action, b)
                          
        return total

    def cells_near_crops(self, path):
        """How many cells of the path are directly next to a crop (distance 1)."""
        pen = self.penalty if self.penalty is not None else proximity_penalty_map(self.field)
        top = max(DEFAULT_PENALTIES.values())
        return sum(1 for cell in path[1:] if pen[cell] >= top)

    def describe_lines(self):
        """The formulation as a list of text lines (printed and drawn)."""
        lines = [
            "PROBLEM FORMULATION",
            f"  State            : robot cell (row, col), grid {self.field.rows} x {self.field.cols}",
            f"  Initial state    : {self.initial_state}",
            f"  Goal state       : {self.goal}",
            f"  Actions          : {list(ACTIONS)}  (4-directional)",
            "  Transition model : result((r, c), a) = (r + dr, c + dc)",
            "  Legal move       : next cell in bounds and SAFE",
            "                     (no potato, protected or obstacle cell)",
            "  Goal test        : state == goal",
        ]
        if self.crop_safe:
            lines += ["  Path cost        : sum of (1 + crop-proximity penalty) per move",
                      "                     [+3 next to crops, +1 two cells away]"]
        else:
            lines.append("  Path cost        : number of moves (each move costs 1)")
        return lines

    def describe(self):
        print("\n".join(self.describe_lines()))


# Fixed examples used to demonstrate the formulation
PROBE_CELL = (5, 3)                                  # just above potato row 6
MANUAL_PATH = [(13, 0), (12, 0), (11, 0), (11, 1)]   # a hand-written 3-move path


def formulation_report(problem):
    """Formulation + worked examples, as text lines."""
    s = problem.initial_state
    lines = problem.describe_lines()
    lines += ["", f"Legal actions at start {s}: {problem.actions(s)}"]
    lines += [f"  {a:<5} -> {nxt}" for a, nxt in problem.successors(s)]
    lines += ["", f"Legal actions at {PROBE_CELL} (potato row just below): {problem.actions(PROBE_CELL)}"]
    lines += ["", f"goal_test({s}) = {problem.goal_test(s)}",
              f"goal_test({problem.goal}) = {problem.goal_test(problem.goal)}"]
    lines += ["", f"Path cost of {MANUAL_PATH} = {problem.path_cost(MANUAL_PATH):g}"]
    if not problem.crop_safe:
        safe = PathProblem(problem.field, crop_safe=True)
        lines.append(f"Crop-safe cost of the same path = {safe.path_cost(MANUAL_PATH):g}")
    return lines


def fit_text_to_axes(text, ax, max_size=12, min_size=5):
    """Shrink the font of 'text' until its box fits inside 'ax' (called on every window resize)."""
    fig = ax.figure
    fig.canvas.draw()                       # make sure sizes are up to date
    renderer = fig.canvas.get_renderer()
    size = max_size
    text.set_fontsize(size)
    while size > min_size:
        box = ax.get_window_extent(renderer)
        tb = text.get_bbox_patch().get_window_extent(renderer) if text.get_bbox_patch() \
            else text.get_window_extent(renderer)
        if tb.width <= box.width * 0.98 and tb.height <= box.height * 0.98:
            break
        size -= 0.5
        text.set_fontsize(size)
        fig.canvas.draw()
    fig.canvas.draw_idle()


def draw_formulation(problem, title="Problem formulation", save_path=None):
    """
    A figure 'page': the formulation text on the left, and on the right the
    field with arrows showing the legal moves at the start and at PROBE_CELL
    (a red cross marks an illegal move into a crop).
    """
    import matplotlib.pyplot as plt
    from field import draw_field

    fig, (text_ax, map_ax) = plt.subplots(1, 2, figsize=(20, 9), layout="constrained",
                                          gridspec_kw={"width_ratios": [1, 1.15]})
    text_ax.axis("off")
    text = text_ax.text(0.02, 0.98, "\n".join(formulation_report(problem)), family="monospace",
                        fontsize=12, va="top", ha="left", transform=text_ax.transAxes,
                        bbox=dict(boxstyle="round,pad=0.8", facecolor="#f7f3ea", edgecolor="#cbbf9f"))

    draw_field(problem.field, path=MANUAL_PATH, ax=map_ax, show_hidden=False,
               title="Legal moves (arrows) and the example path (red)")

    for cell, colour in ((problem.initial_state, "#1f77b4"), (PROBE_CELL, "#7b2cbf")):
        legal = problem.actions(cell)
        for action, (dr, dc) in ACTIONS.items():
            if action in legal:
                map_ax.annotate("", xy=(cell[1] + 0.8 * dc, cell[0] + 0.8 * dr), xytext=(cell[1], cell[0]),
                                arrowprops=dict(arrowstyle="->", color=colour, lw=2), zorder=8)
            elif problem.field.in_bounds((cell[0] + dr, cell[1] + dc)):
                map_ax.scatter(cell[1] + 0.6 * dc, cell[0] + 0.6 * dr, marker="x", s=90,
                               c="#d00000", linewidths=3, zorder=8)
    map_ax.scatter(PROBE_CELL[1], PROBE_CELL[0], s=120, c="#7b2cbf", edgecolors="white", zorder=7)

    # extend the legend with the symbols used on this page
    from matplotlib.lines import Line2D
    legend = map_ax.get_legend()
    handles = list(legend.legend_handles) + [
        Line2D([], [], color="#1f77b4", marker=">", lw=2, label="Legal move from start"),
        Line2D([], [], color="#7b2cbf", marker=">", lw=2, label=f"Legal move from {PROBE_CELL}"),
        Line2D([], [], color="w", marker="x", markeredgecolor="#d00000", markeredgewidth=3,
               markersize=10, label="Illegal move (into a crop)"),
    ]
    # legend UNDER the map, so it can never be cut off at the right edge
    map_ax.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, -0.06),
                  ncol=4, fontsize=8, frameon=True)

    fig.suptitle(title, fontsize=14, fontweight="bold")

    # keep the text inside its panel at any window size
    fit_text_to_axes(text, text_ax)
    fig.canvas.mpl_connect("resize_event", lambda event: fit_text_to_axes(text, text_ax))
    if save_path:
        import os
        os.makedirs(os.path.dirname(save_path) or ".", exist_ok=True)
        fig.savefig(save_path, dpi=150, bbox_inches="tight")
    return fig


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    field = build_default_field()
    problem = PathProblem(field)
    print("\n".join(formulation_report(problem)))
    draw_formulation(problem, title="Iteration 2 - Problem formulation",
                     save_path="screenshots/02_problem.png")
    plt.show()