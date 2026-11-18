"""
field.py  -  Iteration 1 (Task 1: Field environment)

Models the potato field as a 2D grid and draws it with Matplotlib so it
looks like a real field: tilled soil, potato plants on earth ridges, a
taped-off crop-protection zone, stones / crates / logs / tools, and a flag
at the monitoring point. Thin lines mark every grid cell and bold lines
divide the field into 5 x 5 plots.

Coordinates are (row, col). Row 0 is the top of the picture.
"""

import copy
import os

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patheffects as pe
from matplotlib.collections import PatchCollection
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, FancyBboxPatch, Patch, Polygon, Rectangle

# ----------------------------------------------------------------------
# Cell types
# ----------------------------------------------------------------------
SAFE = 0        # bare soil the robot may drive on
POTATO = 1      # a potato plant (part of a crop row) - never enter
PROTECTED = 2   # crop-protection zone (e.g. seedlings / just sprayed) - never enter
OBSTACLE = 3    # stone, crate, log, tool ... - never enter

CELL_NAMES = {
    SAFE: "Safe travel cell",
    POTATO: "Potato row",
    PROTECTED: "Crop-protection zone",
    OBSTACLE: "Obstacle (known)",
}

OBSTACLE_KINDS = ("stone", "crate", "log", "tool")


class Field:
    """
    The field.

    self.grid             -> what the robot KNOWS (its map)
    self.hidden_obstacles -> obstacles that really exist but the robot does
                             not know about yet (used in Task 5)
    self.obstacle_kinds   -> what each obstacle looks like (drawing only)
    """

    def __init__(self, rows, cols):
        self.rows = rows
        self.cols = cols
        self.grid = np.full((rows, cols), SAFE, dtype=int)
        self.start = None
        self.goal = None
        self.hidden_obstacles = set()
        self.obstacle_kinds = {}

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

    def add_obstacle(self, cell, kind="stone"):
        self.grid[cell] = OBSTACLE
        self.obstacle_kinds[cell] = kind

    def add_hidden_obstacle(self, cell, kind="stone"):
        """Real obstacle the robot does not know about. Its map still says SAFE."""
        if self.grid[cell] != SAFE:
            raise ValueError(f"Hidden obstacle {cell} must be on a safe cell")
        self.hidden_obstacles.add(cell)
        self.obstacle_kinds[cell] = kind

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
      - 10 known obstacles (stones, crates, logs, tools) and 1 hidden obstacle.
    """
    f = Field(15, 20)

    for r in (2, 6, 10):
        gaps = (9, 10) if r == 6 else ()
        f.add_potato_row(r, 2, 17, gaps=gaps)

    f.add_protected_region(12, 13, 12, 15)

    known_obstacles = [
        ((4, 5), "stone"), ((8, 13), "crate"), ((13, 6), "log"),        # original three
        ((8, 6), "crate"), ((11, 3), "stone"), ((3, 8), "tool"), ((12, 9), "log"),
        ((5, 17), "crate"), ((9, 15), "stone"), ((14, 10), "tool"),
    ]
    for cell, kind in known_obstacles:
        f.add_obstacle(cell, kind)

    f.add_hidden_obstacle((4, 11), "stone")

    f.set_start((13, 0))
    f.set_goal((4, 16))
    return f


# ----------------------------------------------------------------------
# Visualisation: a realistic-looking field
# ----------------------------------------------------------------------
SOIL = np.array([196, 160, 118])        # tilled soil (safe cells)
RIDGE = np.array([122, 88, 58])         # earth ridge the potatoes grow on
NURSERY = np.array([176, 160, 112])     # soil in the protection zone
LEAF_GREENS = ["#2f6b1f", "#3d8128", "#4f9a35", "#5fae40"]
PX = 16                                  # texture pixels per grid cell


def _soil_texture(field, seed=3):
    """RGB image of the ground: soil grain, furrows along the rows, darker ridges."""
    rng = np.random.default_rng(seed)
    h, w = field.rows * PX, field.cols * PX
    img = np.empty((h, w, 3))
    base = {SAFE: SOIL, OBSTACLE: SOIL, POTATO: RIDGE, PROTECTED: NURSERY}
    for r in range(field.rows):
        for c in range(field.cols):
            img[r * PX:(r + 1) * PX, c * PX:(c + 1) * PX] = base[int(field.grid[r, c])]
    # potato ridges are mounded: lighter on top, darker at the edges
    y = np.linspace(-1, 1, PX)
    mound = (1 - y ** 2)[:, None, None] * 18 - 9
    for r in range(field.rows):
        for c in range(field.cols):
            if field.grid[r, c] == POTATO:
                img[r * PX:(r + 1) * PX, c * PX:(c + 1) * PX] += mound
    img[(np.arange(h) % 4) == 0] -= 10                 # furrow lines running along the field
    img += rng.normal(0, 7, size=(h, w, 1))            # soil grain
    img += rng.normal(0, 4, size=(h, w, 3))            # colour speckle
    return np.clip(img, 0, 255).astype(np.uint8)


def _draw_potato_plants(ax, field, seed=5):
    """Each potato cell gets a leafy plant: a few overlapping green leaf clusters, some with flowers."""
    rng = np.random.default_rng(seed)
    leaves, colours, flowers = [], [], []
    for r, c in zip(*np.where(field.grid == POTATO)):
        for _ in range(4):
            dx, dy = rng.uniform(-0.22, 0.22, 2)
            leaves.append(Circle((c + dx, r + dy), rng.uniform(0.17, 0.26)))
            colours.append(LEAF_GREENS[rng.integers(len(LEAF_GREENS))])
        if rng.random() < 0.35:                         # potato flowers
            flowers.append((c + rng.uniform(-0.2, 0.2), r + rng.uniform(-0.2, 0.2)))
    ax.add_collection(PatchCollection(leaves, facecolors=colours, edgecolors="#1f4a14",
                                      linewidths=0.4, zorder=3))
    if flowers:
        fx, fy = zip(*flowers)
        ax.scatter(fx, fy, s=10, c="#f3f0ff", edgecolors="#b7a6e0", linewidths=0.5, zorder=3.1)


def _draw_protection_zone(ax, field, seed=9):
    """Seedlings inside the zone, faint hatching, and dashed warning tape around it."""
    cells = list(zip(*np.where(field.grid == PROTECTED)))
    if not cells:
        return
    rng = np.random.default_rng(seed)
    sx, sy = [], []
    for r, c in cells:
        ax.add_patch(Rectangle((c - 0.5, r - 0.5), 1, 1, fill=False, hatch="//",
                               edgecolor="#6b8f3a", lw=0, alpha=0.5, zorder=1.5))
        for _ in range(5):
            sx.append(c + rng.uniform(-0.38, 0.38))
            sy.append(r + rng.uniform(-0.38, 0.38))
    ax.scatter(sx, sy, s=12, marker="^", c="#8fd16a", edgecolors="#3d6b22", linewidths=0.4, zorder=3)
    rows = [r for r, _ in cells]
    cols = [c for _, c in cells]
    r0, r1, c0, c1 = min(rows), max(rows), min(cols), max(cols)
    tape = dict(fill=False, lw=2.2, zorder=3.2)
    ax.add_patch(Rectangle((c0 - 0.5, r0 - 0.5), c1 - c0 + 1, r1 - r0 + 1, edgecolor="#facc15", **tape))
    ax.add_patch(Rectangle((c0 - 0.5, r0 - 0.5), c1 - c0 + 1, r1 - r0 + 1, edgecolor="#111111",
                           linestyle=(0, (3, 3)), **tape))


def _draw_obstacle(ax, cell, kind, alpha=1.0, zorder=5):
    """Draw one obstacle as a stone, crate, log or tool."""
    r, c = cell
    if kind == "crate":
        ax.add_patch(Rectangle((c - 0.36, r - 0.36), 0.72, 0.72, facecolor="#b07a3c",
                               edgecolor="#5c3a17", lw=1.4, alpha=alpha, zorder=zorder))
        ax.plot([c - 0.36, c + 0.36], [r - 0.36, r + 0.36], color="#5c3a17", lw=1.1, alpha=alpha, zorder=zorder)
        ax.plot([c - 0.36, c + 0.36], [r + 0.36, r - 0.36], color="#5c3a17", lw=1.1, alpha=alpha, zorder=zorder)
    elif kind == "log":
        ax.add_patch(FancyBboxPatch((c - 0.42, r - 0.17), 0.84, 0.34, boxstyle="round,pad=0.02,rounding_size=0.16",
                                    facecolor="#8a5a2b", edgecolor="#4a2e12", lw=1.2, alpha=alpha, zorder=zorder))
        ax.add_patch(Circle((c + 0.32, r), 0.15, facecolor="#d9b27c", edgecolor="#4a2e12", lw=1,
                            alpha=alpha, zorder=zorder + 0.1))
        ax.add_patch(Circle((c + 0.32, r), 0.06, fill=False, edgecolor="#8a5a2b", lw=0.8,
                            alpha=alpha, zorder=zorder + 0.2))
    elif kind == "tool":                                  # a hoe lying on the ground
        ax.plot([c - 0.38, c + 0.3], [r + 0.3, r - 0.3], color="#9a6b3a", lw=3, alpha=alpha,
                solid_capstyle="round", zorder=zorder)
        ax.add_patch(Polygon([(c + 0.18, r - 0.42), (c + 0.44, r - 0.16), (c + 0.34, r - 0.06),
                              (c + 0.12, r - 0.3)], facecolor="#9aa3ad", edgecolor="#3f4650", lw=1,
                             alpha=alpha, zorder=zorder + 0.1))
    else:                                                 # stone: an irregular grey rock
        angles = np.linspace(0, 2 * np.pi, 9)[:-1]
        radii = np.array([0.36, 0.31, 0.38, 0.33, 0.37, 0.30, 0.35, 0.32])
        pts = [(c + rad * np.cos(a), r + rad * 0.85 * np.sin(a)) for a, rad in zip(angles, radii)]
        ax.add_patch(Polygon(pts, facecolor="#8d8d8a", edgecolor="#4a4a48", lw=1.3, alpha=alpha, zorder=zorder))
        ax.add_patch(Circle((c - 0.1, r - 0.1), 0.09, facecolor="#c4c4c0", edgecolor="none",
                            alpha=alpha, zorder=zorder + 0.1))


def _draw_goal_flag(ax, goal):
    r, c = goal
    ax.plot([c - 0.12, c - 0.12], [r + 0.42, r - 0.45], color="#3b3b3b", lw=2, zorder=7)
    ax.add_patch(Polygon([(c - 0.1, r - 0.45), (c + 0.42, r - 0.3), (c - 0.1, r - 0.12)],
                         facecolor="#ffbf00", edgecolor="#7a5a00", lw=1, zorder=7))
    ax.add_patch(Circle((c - 0.12, r + 0.42), 0.07, facecolor="#3b3b3b", zorder=7))


def _draw_start_robot(ax, start):
    r, c = start
    ax.add_patch(FancyBboxPatch((c - 0.36, r - 0.3), 0.72, 0.6, boxstyle="round,pad=0.02,rounding_size=0.12",
                                facecolor="#1f77b4", edgecolor="white", lw=1.5, zorder=7))
    for dy in (-0.34, 0.26):                               # wheels
        ax.add_patch(Rectangle((c - 0.3, r + dy), 0.6, 0.08, facecolor="#222222", zorder=6.9))
    ax.text(c, r, "S", color="white", ha="center", va="center", fontsize=8, fontweight="bold", zorder=7.1)


def _draw_grid(ax, field):
    """Thin lines around every cell, bold lines every 5 cells (plots)."""
    for x in np.arange(-0.5, field.cols, 1):
        major = (x + 0.5) % 5 == 0
        ax.axvline(x, color="#2b1d10", lw=1.3 if major else 0.5, alpha=0.55 if major else 0.25, zorder=2)
    for y in np.arange(-0.5, field.rows, 1):
        major = (y + 0.5) % 5 == 0
        ax.axhline(y, color="#2b1d10", lw=1.3 if major else 0.5, alpha=0.55 if major else 0.25, zorder=2)


def legend_handles(show_hidden=True, explored=False, path=False):
    handles = [
        Patch(facecolor="#c4a076", edgecolor="#6b4f33", label="Safe soil (drivable)"),
        Line2D([], [], marker="o", color="w", markerfacecolor="#3d8128", markeredgecolor="#1f4a14",
               markersize=11, label="Potato plants on ridge"),
        Patch(facecolor="#b0a070", edgecolor="#111111", hatch="//", linestyle="--",
              label="Crop-protection zone (seedlings)"),
        Line2D([], [], marker="o", color="w", markerfacecolor="#8d8d8a", markeredgecolor="#4a4a48",
               markersize=11, label="Obstacle: stone / crate / log / tool"),
        Line2D([], [], marker="s", color="w", markerfacecolor="#1f77b4", markersize=11, label="Start (robot)"),
        Line2D([], [], marker=">", color="w", markerfacecolor="#ffbf00", markeredgecolor="#7a5a00",
               markersize=12, label="Goal (monitoring flag)"),
    ]
    if show_hidden:
        handles.append(Line2D([], [], marker="X", color="w", markerfacecolor="#e63946",
                              markeredgecolor="black", markersize=11, label="Hidden obstacle (unknown to robot)"))
    if explored:
        handles.append(Line2D([], [], marker="o", color="w", markerfacecolor="#38bdf8",
                              markeredgecolor="#0c4a6e", markersize=6, label="Expanded node"))
    if path:
        handles.append(Line2D([], [], color="#e11d48", linewidth=2.8, label="Path"))
    return handles


def draw_field(field, path=None, explored=None, title="Potato field",
               show_hidden=True, ax=None, save_path=None):
    """
    Draw the field. Optional:
      path     : list of (row, col) cells -> red line
      explored : iterable of (row, col)  -> blue dots (expanded nodes)
    Returns the Matplotlib axes so later code can draw more on top.
    """
    if ax is None:
        _, ax = plt.subplots(figsize=(11, 7))

    ext = (-0.5, field.cols - 0.5, field.rows - 0.5, -0.5)
    ax.imshow(_soil_texture(field), extent=ext, interpolation="bilinear", zorder=0)
    _draw_protection_zone(ax, field)
    _draw_potato_plants(ax, field)
    _draw_grid(ax, field)

    for r, c in zip(*np.where(field.grid == OBSTACLE)):
        _draw_obstacle(ax, (r, c), field.obstacle_kinds.get((r, c), "stone"))
    if show_hidden:
        for cell in field.hidden_obstacles:
            _draw_obstacle(ax, cell, field.obstacle_kinds.get(cell, "stone"), alpha=0.35)
        if field.hidden_obstacles:
            hr = [r for r, _ in field.hidden_obstacles]
            hc = [c for _, c in field.hidden_obstacles]
            ax.scatter(hc, hr, marker="X", s=150, c="#e63946", edgecolors="black", zorder=6)

    if explored:
        ax.scatter([c for _, c in explored], [r for r, _ in explored], s=14, c="#38bdf8",
                   edgecolors="#0c4a6e", linewidths=0.4, alpha=0.85, zorder=4)
    if path:
        ax.plot([c for _, c in path], [r for r, _ in path], color="#e11d48", linewidth=2.8, zorder=4.5,
                path_effects=[pe.Stroke(linewidth=4.6, foreground="white", alpha=0.8), pe.Normal()])

    _draw_start_robot(ax, field.start)
    _draw_goal_flag(ax, field.goal)

    ax.set_xlim(-0.5, field.cols - 0.5)
    ax.set_ylim(field.rows - 0.5, -0.5)
    ax.set_aspect("equal")
    ax.set_xticks(range(field.cols))
    ax.set_yticks(range(field.rows))
    ax.tick_params(length=0, labelsize=7)
    for spine in ax.spines.values():
        spine.set_edgecolor("#4a7a2a")                    # grass-green field border
        spine.set_linewidth(3)

    ax.legend(handles=legend_handles(show_hidden and bool(field.hidden_obstacles), bool(explored), bool(path)),
              loc="upper left", bbox_to_anchor=(1.01, 1), fontsize=8, frameon=True)
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
