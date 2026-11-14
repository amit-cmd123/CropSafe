"""
field.py  -  Iteration 1 (Task 1: Field environment)

Models the potato field as a 2D grid and draws it with Matplotlib.

Coordinates are (row, col). Row 0 is the top of the picture.
"""

import copy
import os

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm
from matplotlib.patches import Patch, Rectangle
from matplotlib.lines import Line2D

# ----------------------------------------------------------------------
# Cell types
# ----------------------------------------------------------------------
SAFE = 0        # bare soil the robot may drive on
POTATO = 1      # a potato plant (part of a crop row) - never enter
PROTECTED = 2   # crop-protection zone (e.g. seedlings / just sprayed) - never enter
OBSTACLE = 3    # stone, tool, any object - never enter

CELL_NAMES = {
    SAFE: "Safe travel cell",
    POTATO: "Potato row",
    PROTECTED: "Crop-protection zone",
    OBSTACLE: "Obstacle (known)",
}
CELL_COLORS = ["#efe6d2", "#3f7d20", "#b9e3a3", "#4a4a4a"]


class Field:
    """
    The field.

    self.grid             -> what the robot KNOWS (its map)
    self.hidden_obstacles -> obstacles that really exist but the robot does
                             not know about yet (used in Iteration 7)
    """

    def __init__(self, rows, cols):
        self.rows = rows
        self.cols = cols
        self.grid = np.full((rows, cols), SAFE, dtype=int)
        self.start = None
        self.goal = None
        self.hidden_obstacles = set()

    # ---------------- basic queries ----------------
    def in_bounds(self, cell):
        r, c = cell
        return 0 <= r < self.rows and 0 <= c < self.cols

    def cell_type(self, cell):
        return int(self.grid[cell])

    def is_blocked(self, cell):
        """True if the robot (with its current knowledge) may NOT enter cell."""
        return self.grid[cell] != SAFE

    # ---------------- building the field ----------------
    def add_potato_row(self, row, col_start, col_end, gaps=()):
        """Plant a horizontal crop row; 'gaps' are columns left open as crossings."""
        for c in range(col_start, col_end + 1):
            if c not in gaps:
                self.grid[row, c] = POTATO

    def add_protected_region(self, r0, r1, c0, c1):
        """Rectangle of protected cells (inclusive bounds)."""
        self.grid[r0:r1 + 1, c0:c1 + 1] = PROTECTED

    def add_obstacle(self, cell):
        self.grid[cell] = OBSTACLE

    def add_hidden_obstacle(self, cell):
        """Real obstacle the robot does not know about. Its map still says SAFE."""
        if self.grid[cell] != SAFE:
            raise ValueError(f"Hidden obstacle {cell} must be on a safe cell")
        self.hidden_obstacles.add(cell)

    def set_start(self, cell):
        self._check_free(cell, "Start")
        self.start = cell

    def set_goal(self, cell):
        self._check_free(cell, "Goal")
        self.goal = cell

    def _check_free(self, cell, name):
        if not self.in_bounds(cell) or self.grid[cell] != SAFE or cell in self.hidden_obstacles:
            raise ValueError(f"{name} {cell} must be an in-bounds safe cell")

    # ---------------- helpers ----------------
    def true_grid(self):
        """The real world: known map + hidden obstacles."""
        g = self.grid.copy()
        for cell in self.hidden_obstacles:
            g[cell] = OBSTACLE
        return g

    def copy(self):
        return copy.deepcopy(self)

    def summary(self):
        print(f"Field size : {self.rows} x {self.cols}")
        print(f"Start      : {self.start}")
        print(f"Goal       : {self.goal}")
        for t, name in CELL_NAMES.items():
            print(f"{name:<22}: {int(np.sum(self.grid == t))}")
        print(f"{'Hidden obstacles':<22}: {len(self.hidden_obstacles)} {sorted(self.hidden_obstacles)}")


# ----------------------------------------------------------------------
# The field used throughout the project
# ----------------------------------------------------------------------
def build_default_field():
    """
    15 x 20 field.
      - 3 horizontal potato rows (rows 2, 6, 10), columns 2..17.
        Columns 0-1 and 18-19 are open headlands at both ends.
        Row 6 has a crossing gap at columns 9-10.
      - Travel lanes between the crop rows.
      - One crop-protection zone (rows 12-13, cols 12-15).
      - 3 known obstacles, 1 hidden obstacle.
    """
    f = Field(15, 20)

    for r in (2, 6, 10):
        gaps = (9, 10) if r == 6 else ()
        f.add_potato_row(r, 2, 17, gaps=gaps)

    f.add_protected_region(12, 13, 12, 15)

    for cell in [(4, 5), (8, 13), (13, 6)]:
        f.add_obstacle(cell)

    f.add_hidden_obstacle((4, 11))

    f.set_start((13, 0))
    f.set_goal((4, 16))
    return f


# ----------------------------------------------------------------------
# Visualisation
# ----------------------------------------------------------------------
def draw_field(field, path=None, explored=None, title="Potato field",
               show_hidden=True, ax=None, save_path=None):
    """
    Draw the grid. Optional:
      path     : list of (row, col) cells -> red line
      explored : iterable of (row, col)  -> blue dots (expanded nodes)
    Returns the Matplotlib axes so later iterations can draw more on top.
    """
    if ax is None:
        _, ax = plt.subplots(figsize=(11, 7))

    cmap = ListedColormap(CELL_COLORS)
    norm = BoundaryNorm([-0.5, 0.5, 1.5, 2.5, 3.5], cmap.N)
    ax.imshow(field.grid, cmap=cmap, norm=norm, origin="upper")

    # hatch protected cells so they stand out even in greyscale prints
    for r, c in zip(*np.where(field.grid == PROTECTED)):
        ax.add_patch(Rectangle((c - 0.5, r - 0.5), 1, 1, fill=False,
                               hatch="//", edgecolor="#5a8f3e", lw=0))

    # cell grid lines
    ax.set_xticks(np.arange(-0.5, field.cols, 1), minor=True)
    ax.set_yticks(np.arange(-0.5, field.rows, 1), minor=True)
    ax.grid(which="minor", color="white", linewidth=0.6)
    ax.set_xticks(range(field.cols))
    ax.set_yticks(range(field.rows))
    ax.tick_params(which="both", length=0, labelsize=7)

    if explored:
        er = [r for r, _ in explored]
        ec = [c for _, c in explored]
        ax.scatter(ec, er, s=14, c="#4aa3df", alpha=0.6, zorder=3)

    if path:
        ax.plot([c for _, c in path], [r for r, _ in path],
                color="#d62828", linewidth=2.5, zorder=4)

    if show_hidden and field.hidden_obstacles:
        hr = [r for r, _ in field.hidden_obstacles]
        hc = [c for _, c in field.hidden_obstacles]
        ax.scatter(hc, hr, marker="X", s=160, c="#e63946", edgecolors="black", zorder=5)

    sr, sc = field.start
    gr, gc = field.goal
    ax.scatter(sc, sr, marker="o", s=260, c="#1f77b4", edgecolors="white", zorder=6)
    ax.text(sc, sr, "S", color="white", ha="center", va="center", fontsize=9, fontweight="bold", zorder=7)
    ax.scatter(gc, gr, marker="*", s=420, c="#ffbf00", edgecolors="black", zorder=6)
    ax.text(gc, gr + 0.05, "G", color="black", ha="center", va="center", fontsize=7, fontweight="bold", zorder=7)

    handles = [Patch(facecolor=CELL_COLORS[t], edgecolor="grey", label=n) for t, n in CELL_NAMES.items()]
    handles[PROTECTED] = Patch(facecolor=CELL_COLORS[PROTECTED], edgecolor="#5a8f3e",
                               hatch="//", label=CELL_NAMES[PROTECTED])
    handles += [
        Line2D([], [], marker="o", color="w", markerfacecolor="#1f77b4", markersize=11, label="Start"),
        Line2D([], [], marker="*", color="w", markerfacecolor="#ffbf00", markeredgecolor="black",
               markersize=15, label="Goal"),
    ]
    if show_hidden and field.hidden_obstacles:
        handles.append(Line2D([], [], marker="X", color="w", markerfacecolor="#e63946",
                              markeredgecolor="black", markersize=11, label="Hidden obstacle (unknown to robot)"))
    if explored:
        handles.append(Line2D([], [], marker="o", color="w", markerfacecolor="#4aa3df",
                              markersize=6, label="Expanded node"))
    if path:
        handles.append(Line2D([], [], color="#d62828", linewidth=2.5, label="Path"))

    ax.legend(handles=handles, loc="upper left", bbox_to_anchor=(1.01, 1), fontsize=8, frameon=True)
    ax.set_title(title, fontsize=12, fontweight="bold")

    if save_path:
        os.makedirs(os.path.dirname(save_path) or ".", exist_ok=True)
        ax.figure.savefig(save_path, dpi=150, bbox_inches="tight")
    return ax


if __name__ == "__main__":
    field = build_default_field()
    field.summary()
    draw_field(field, title="Iteration 1 - Potato field environment",
               save_path="screenshots/01_field.png")
    plt.show()