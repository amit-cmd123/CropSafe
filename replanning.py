"""
replanning.py  -  Iteration 7 (Task 5): obstacle detection and replanning

The robot does NOT know about field.hidden_obstacles. It plans with A* on
its own known map, then drives the route one cell at a time:

    loop until at goal:
        SENSE  : look at the 8 cells around the robot in the REAL world
        UPDATE : any obstacle it did not know about is added to its known map
        REPLAN : if a newly found obstacle lies on the remaining route,
                 run A* again from the robot's current cell
        ACT    : move one cell along the (possibly new) route

Run:
        python replanning.py            -> page 1 (maps) + page 2 (results table)
    python replanning.py --animate  -> also page 3: the robot driving (smooth animation)
    python replanning.py --gif      -> same, and saves screenshots/07_replanning.gif
"""

import sys
import time
from dataclasses import dataclass, field as dc_field

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

from field import build_default_field, draw_field, OBSTACLE, SAFE
from problem import PathProblem, fit_text_to_axes
from search.astar import astar

SENSOR_RADIUS = 1      # the robot sees the 8 cells around it


@dataclass
class PlanRecord:
    step: int          # how many moves the robot had made when this plan was made
    position: tuple    # where the robot was
    path: list         # the planned route from there to the goal
    reason: str        # "initial plan" or which obstacle triggered the replan


@dataclass
class NavigationResult:
    reached: bool
    trail: list                    # cells the robot actually drove through
    plans: list                    # every PlanRecord (first one = initial plan)
    detections: list               # (step, robot position, obstacle cell)
    replans: int
    nodes_expanded: int            # summed over all A* calls
    planning_time_ms: float        # summed over all A* calls
    total_time_ms: float           # whole simulation
    known: object                  # the robot's map at the end
    frames: list = dc_field(default_factory=list)   # snapshots for the animation

    @property
    def moves(self):
        return len(self.trail) - 1


def sense(position, true_grid, known, radius=SENSOR_RADIUS):
    """Return obstacles within 'radius' that exist in reality but are not on the known map."""
    found = []
    r0, c0 = position
    for dr in range(-radius, radius + 1):
        for dc in range(-radius, radius + 1):
            cell = (r0 + dr, c0 + dc)
            if known.in_bounds(cell) and true_grid[cell] == OBSTACLE and known.grid[cell] == SAFE:
                found.append(cell)
    return found


def plan_from(known, position, crop_safe):
    """A* on the robot's known map, starting from its current cell."""
    problem = PathProblem(known, crop_safe=crop_safe)
    problem.initial_state = position
    return astar(problem, name="A* (replan)")


def navigate(field, crop_safe=True, record_frames=False, max_steps=1000):
    t0 = time.perf_counter()

    true_grid = field.true_grid()        # the real world, including hidden obstacles
    known = field.copy()                 # the robot's map ...
    known.hidden_obstacles = set()       # ... which knows nothing about them

    position = field.start
    trail = [position]
    detections, plans, frames = [], [], []
    replans = nodes = 0
    plan_ms = 0.0

    result = plan_from(known, position, crop_safe)
    nodes += result.nodes_expanded
    plan_ms += result.time_ms
    plans.append(PlanRecord(0, position, result.path, "initial plan"))
    route = result.path
    if not result.found:
        return NavigationResult(False, trail, plans, detections, replans, nodes, plan_ms,
                                (time.perf_counter() - t0) * 1000, known, frames)

    step = 0
    while position != field.goal and step < max_steps:
        # ---- SENSE + UPDATE ----
        new_obstacles = sense(position, true_grid, known)
        for cell in new_obstacles:
            known.grid[cell] = OBSTACLE
            detections.append((step, position, cell))

        # ---- REPLAN only if the remaining route is now blocked ----
        remaining = route[route.index(position) + 1:]
        blocked = [c for c in new_obstacles if c in remaining]
        if blocked:
            replans += 1
            result = plan_from(known, position, crop_safe)
            nodes += result.nodes_expanded
            plan_ms += result.time_ms
            plans.append(PlanRecord(step, position, result.path, f"obstacle at {blocked[0]}"))
            if not result.found:
                break                    # goal unreachable with what we now know
            route = result.path

        if record_frames:
            frames.append((position, list(trail), list(route), known.grid.copy()))

        # ---- ACT: one step along the route ----
        nxt = route[route.index(position) + 1]
        if true_grid[nxt] != SAFE:      # should never happen: the sensor sees 1 cell ahead
            raise RuntimeError(f"Robot drove into an obstacle at {nxt}")
        position = nxt
        trail.append(position)
        step += 1

    if record_frames:
        frames.append((position, list(trail), [], known.grid.copy()))

    return NavigationResult(position == field.goal, trail, plans, detections, replans, nodes,
                            plan_ms, (time.perf_counter() - t0) * 1000, known, frames)


# ----------------------------------------------------------------------
# Reporting
# ----------------------------------------------------------------------
def comparison_rows(field, nav, crop_safe=True):
    """Online run vs. the initial plan vs. an 'oracle' that knew every obstacle from the start."""
    scorer = PathProblem(field, crop_safe=True)          # score everything with the crop-safe cost
    oracle_field = field.copy()
    for cell in oracle_field.hidden_obstacles:
        oracle_field.grid[cell] = OBSTACLE
    oracle_field.hidden_obstacles = set()
    oracle = astar(PathProblem(oracle_field, crop_safe=crop_safe), name="A* oracle")

    initial = nav.plans[0].path
    blocked = any(cell in field.hidden_obstacles for cell in initial)
    return [
        ["Initial plan (known map)", len(initial) - 1, f"{scorer.path_cost(initial):.0f}",
         "no (blocked)" if blocked else "yes", "-", "-"],
        ["Online A* with replanning", nav.moves, f"{scorer.path_cost(nav.trail):.0f}",
         "yes" if nav.reached else "no", nav.nodes_expanded, nav.replans],
        ["A* oracle (knew everything)", oracle.path_length, f"{scorer.path_cost(oracle.path):.0f}",
         "yes" if oracle.found else "no", oracle.nodes_expanded, 0],
    ]


HEADERS = ["Run", "Moves", "Crop-safe cost", "Reached goal", "Nodes expanded", "Replans"]


def report_lines(nav):
    """What happened during the drive, as text lines (printed and drawn)."""
    lines = ["WHAT HAPPENED DURING THE DRIVE"]
    if nav.detections:
        for s, p, c in nav.detections:
            lines.append(f"  step {s:>2}: robot at {p} sensed an unknown obstacle at {c}")
    else:
        lines.append("  no unknown obstacles were sensed")
    lines.append("")
    for rec in nav.plans:
        lines.append(f"  plan at step {rec.step:>2} from {rec.position}: "
                     f"{len(rec.path) - 1} moves  ({rec.reason})")
    lines += ["",
              f"  Replans            : {nav.replans}",
              f"  Moves driven       : {nav.moves}",
              f"  Goal reached       : {'yes' if nav.reached else 'no'}",
              f"  A* planning time   : {nav.planning_time_ms:.3f} ms (all A* runs)",
              f"  Whole simulation   : {nav.total_time_ms:.3f} ms"]
    return lines


def print_report(nav, rows):
    print("\n".join(report_lines(nav)) + "\n")
    print(" | ".join(f"{h:<27}" if i == 0 else f"{h:>14}" for i, h in enumerate(HEADERS)))
    print("-" * 115)
    for row in rows:
        print(" | ".join(f"{str(v):<27}" if i == 0 else f"{str(v):>14}" for i, v in enumerate(row)))


# ----------------------------------------------------------------------
# Drawing
# ----------------------------------------------------------------------
OLD_ROUTE = dict(ls=":", lw=2.5, color="#888888")
NEW_ROUTE = dict(ls="--", lw=2.5, color="#1d4ed8")
SENSOR_BOX = dict(fill=False, ls="--", lw=1.5, edgecolor="#9333ea")

DETECTED_HANDLE = Line2D([], [], marker="s", color="w", markerfacecolor="#4a4a4a",
                         markeredgecolor="#f97316", markeredgewidth=3, markersize=12,
                         label="Detected obstacle (was hidden)")


def mark_detected(ax, cells):
    """Orange ring around obstacles the robot discovered while driving."""
    for r, c in cells:
        ax.add_patch(plt.Rectangle((c - 0.5, r - 0.5), 1, 1, fill=False, lw=3,
                                   edgecolor="#f97316", zorder=9))


def mark_robot(ax, position, filled=False):
    """Robot as a blue square, with its 3x3 sensor range around it."""
    r, c = position
    if filled:
        ax.scatter(c, r, s=260, marker="s", c="#1d4ed8", edgecolors="white", zorder=10)
    else:
        ax.scatter(c, r, s=260, marker="s", facecolors="none", edgecolors="#1d4ed8", lw=2.5, zorder=10)
    ax.add_patch(plt.Rectangle((c - 1.5, r - 1.5), 3, 3, zorder=9, **SENSOR_BOX))


def shared_legend_handles(field_legend_handles):
    """One legend for the whole page: the field symbols plus the replanning symbols."""
    return list(field_legend_handles) + [
        DETECTED_HANDLE,
        Line2D([], [], label="Old route (now blocked)", **OLD_ROUTE),
        Line2D([], [], label="New A* route after replan", **NEW_ROUTE),
        Line2D([], [], marker="s", color="w", markeredgecolor="#1d4ed8", markeredgewidth=2,
               markersize=11, label="Robot when it sensed the obstacle"),
        Line2D([], [], ls="--", lw=1.5, color="#9333ea", label="Sensor range (8 cells)"),
    ]


def draw_replanning_maps(field, nav, save_path="screenshots/07_replanning.png"):
    """Page 1: before / at detection / after, with ONE shared legend underneath."""
    fig, axes = plt.subplots(1, 3, figsize=(20, 8), layout="constrained")

    initial = nav.plans[0].path
    replan = nav.plans[1] if len(nav.plans) > 1 else None

    # 1. initial plan; the red X is the obstacle the robot cannot see
    draw_field(field, path=initial, ax=axes[0],
               title=f"1. Initial A* plan: {len(initial) - 1} moves\n(passes the hidden obstacle)")
    field_handles = axes[0].get_legend().legend_handles

    # 2. the moment the obstacle is sensed
    if replan:
        known_then = field.copy()
        known_then.hidden_obstacles = set()
        found_so_far = [c for s, _, c in nav.detections if s <= replan.step]
        for cell in found_so_far:
            known_then.grid[cell] = OBSTACLE
        draw_field(known_then, path=nav.trail[:replan.step + 1], ax=axes[1], show_hidden=False,
                   title=f"2. Step {replan.step}: obstacle sensed at {found_so_far[0]}\n"
                         f"-> A* replans from {replan.position}")
        old_rest = initial[initial.index(replan.position):]
        axes[1].plot([c for _, c in old_rest], [r for r, _ in old_rest], zorder=4, **OLD_ROUTE)
        axes[1].plot([c for _, c in replan.path], [r for r, _ in replan.path], zorder=5, **NEW_ROUTE)
        mark_robot(axes[1], replan.position)
        mark_detected(axes[1], found_so_far)
    else:
        draw_field(field, ax=axes[1], title="2. No unknown obstacle on the route\n(no replanning needed)")

    # 3. what the robot actually drove
    draw_field(nav.known, path=nav.trail, ax=axes[2], show_hidden=False,
               title=f"3. Route driven: {nav.moves} moves, {nav.replans} replan(s)\n"
                     f"{'goal reached' if nav.reached else 'goal NOT reached'}")
    mark_detected(axes[2], [c for _, _, c in nav.detections])

    for ax in axes:
        ax.get_legend().remove()
    fig.legend(handles=shared_legend_handles(field_handles), loc="outside lower center",
               ncol=5, fontsize=9, frameon=True)
    fig.suptitle("Task 5 - Unknown obstacle detection and A* replanning", fontsize=14, fontweight="bold")
    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches="tight")
    return fig


def draw_replanning_results(nav, rows, save_path="screenshots/07_replanning_results.png"):
    """Page 2: what happened (text) and the comparison table."""
    fig = plt.figure(figsize=(16, 7), layout="constrained")
    grid = fig.add_gridspec(2, 1, height_ratios=[1.1, 1])
    text_ax = fig.add_subplot(grid[0])
    table_ax = fig.add_subplot(grid[1])
    text_ax.axis("off")
    table_ax.axis("off")

    text = text_ax.text(0.02, 0.98, "\n".join(report_lines(nav)), family="monospace", fontsize=12,
                        va="top", ha="left", transform=text_ax.transAxes,
                        bbox=dict(boxstyle="round,pad=0.8", facecolor="#f7f3ea", edgecolor="#cbbf9f"))

    headers = ["Run", "Moves", "Crop-safe\ncost", "Reached\ngoal", "Nodes\nexpanded", "Replans"]
    table = table_ax.table(cellText=[[str(v) for v in row] for row in rows], colLabels=headers,
                           cellLoc="center", bbox=[0, 0, 1, 0.9],     # fill the panel
                           colWidths=[0.32, 0.12, 0.14, 0.14, 0.14, 0.12])
    table.auto_set_font_size(False)
    table.set_fontsize(11)
    for col in range(len(headers)):
        table[0, col].set_facecolor("#3f7d20")
        table[0, col].set_text_props(color="white", fontweight="bold")
        table[2, col].set_facecolor("#e8f4e0")       # highlight the online run
    table_ax.set_title("All routes scored with the crop-safe cost", fontsize=12, fontweight="bold")

    fig.suptitle("Task 5 - Replanning results", fontsize=14, fontweight="bold")
    fit_text_to_axes(text, text_ax)
    fig.canvas.mpl_connect("resize_event", lambda event: fit_text_to_axes(text, text_ax))
    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches="tight")
    return fig

def animate(field, nav, move_ms=300, substeps=8, pause_frames=20):
    """
    Page 3 (optional): the robot driving, as a smooth animation.

    The field, grid and legend are drawn ONCE and saved as a background
    picture. Each frame only repaints the few things that move (robot,
    sensor box, trail, route, ...) - this is called "blitting". So the
    animation is fast and nothing flickers or jumps.
      move_ms      : time for one cell-to-cell move
      substeps     : in-between frames per move (makes the robot glide)
      pause_frames : frames to hold when an obstacle is found and A* replans
    Needs navigate(..., record_frames=True). Returns the FuncAnimation, which
    the caller must keep in a variable until plt.show() (or it stops).
    """
    import numpy as np
    from matplotlib.animation import FuncAnimation

    frames = nav.frames
    replan_steps = {p.step for p in nav.plans[1:]}
    total_steps = len(frames) - 1

    # ---------- timeline: (step k, fraction t of the move k -> k+1, holding?) ----------
    timeline = []
    for k in range(total_steps):
        if k in replan_steps:
            timeline += [(k, 0.0, True)] * pause_frames
        timeline += [(k, s / substeps, False) for s in range(substeps)]
    timeline += [(total_steps, 0.0, False)] * pause_frames             # rest at the goal

    # ---------- static background, drawn once ----------
    fig, ax = plt.subplots(figsize=(11, 8), layout="constrained")
    start_known = field.copy()
    start_known.hidden_obstacles = set()
    draw_field(start_known, ax=ax, show_hidden=False,
               title="Robot driving with live obstacle detection (crop-safe A*)")
    handles = list(ax.get_legend().legend_handles) + [
        Line2D([], [], marker="X", color="w", markerfacecolor="#e63946", markeredgecolor="black",
               markersize=11, label="Hidden obstacle (unknown to robot)"),
        DETECTED_HANDLE,
        Line2D([], [], color="#d62828", lw=2.5, label="Route driven so far"),
        Line2D([], [], label="Current planned route", **NEW_ROUTE),
        Line2D([], [], label="Old route (blocked)", **OLD_ROUTE),
        Line2D([], [], marker="s", color="w", markerfacecolor="#1d4ed8", markersize=11, label="Robot"),
        Line2D([], [], ls="--", lw=1.5, color="#9333ea", label="Sensor range (8 cells)"),
    ]
    ax.get_legend().remove()
    ax.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, -0.06), ncol=4, fontsize=8)

    # ---------- moving artists, created once and updated every frame ----------
    found_squares = {cell: ax.add_patch(plt.Rectangle((cell[1] - 0.5, cell[0] - 0.5), 1, 1,
                                                      facecolor="#4a4a4a", edgecolor="#f97316",
                                                      lw=3, zorder=6, visible=False))
                     for cell in field.hidden_obstacles}
    hidden_dots = ax.scatter([], [], marker="X", s=160, c="#e63946", edgecolors="black", zorder=6)
    trail_line, = ax.plot([], [], color="#d62828", lw=2.5, zorder=7)
    route_line, = ax.plot([], [], zorder=8, **NEW_ROUTE)
    old_line, = ax.plot([], [], zorder=8, **OLD_ROUTE)
    sensor = ax.add_patch(plt.Rectangle((0, 0), 3, 3, zorder=9, **SENSOR_BOX))
    robot = ax.scatter([], [], s=260, marker="s", c="#1d4ed8", edgecolors="white", zorder=10)
    status = ax.text(0.01, 0.985, "", transform=ax.transAxes, ha="left", va="top", fontsize=10,
                     family="monospace", zorder=11,
                     bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.85, edgecolor="#999999"))
    banner = ax.text(0.5, 0.02, "", transform=ax.transAxes, ha="center", va="bottom", fontsize=12,
                     fontweight="bold", color="white", zorder=11,
                     bbox=dict(boxstyle="round,pad=0.5", facecolor="#c2410c", edgecolor="none"))
    moving = [*found_squares.values(), hidden_dots, trail_line, route_line, old_line,
              sensor, robot, status, banner]

    fig.canvas.draw()
    fig.set_layout_engine("none")              # freeze the layout: nothing can jump between frames

    def update(i):
        k, t, holding = timeline[i]
        position, trail, route, known_grid = frames[k]
        nxt = frames[k + 1][0] if k < total_steps else position
        r = position[0] + (nxt[0] - position[0]) * t          # glide between cells
        c = position[1] + (nxt[1] - position[1]) * t

        unknown = []
        for cell, square in found_squares.items():
            is_found = bool(known_grid[cell] == OBSTACLE)
            square.set_visible(is_found)                      # found -> grey square with orange ring
            if not is_found:
                unknown.append(cell)                          # not found yet -> red X
        hidden_dots.set_offsets(np.array([(cc, rr) for rr, cc in unknown]) if unknown else np.empty((0, 2)))

        driven = list(trail) + [(r, c)]
        trail_line.set_data([p[1] for p in driven], [p[0] for p in driven])
        route_line.set_data([p[1] for p in route], [p[0] for p in route])
        if holding and k > 0:                                 # show the blocked route while replanning
            old = frames[k - 1][2]
            old = old[old.index(position):] if position in old else old
            old_line.set_data([p[1] for p in old], [p[0] for p in old])
        else:
            old_line.set_data([], [])

        robot.set_offsets([[c, r]])
        sensor.set_xy((c - 1.5, r - 1.5))

        found = len(field.hidden_obstacles) - len(unknown)
        replans = sum(1 for s in replan_steps if s < k or (s == k and (holding or t > 0)))
        status.set_text(f"Step {k:>2}/{total_steps}  robot {str(position):<8}  "
                        f"found {found}  replans {replans}")

        if k == total_steps:
            banner.set_text("Goal reached" if nav.reached else "Goal NOT reachable")
            banner.get_bbox_patch().set_facecolor("#15803d" if nav.reached else "#991b1b")
            banner.set_visible(True)
        elif holding:
            cell = [cc for s, _, cc in nav.detections if s == k][0]
            banner.set_text(f"Obstacle sensed at {cell}  ->  A* replans from {position}")
            banner.get_bbox_patch().set_facecolor("#c2410c")
            banner.set_visible(True)
        else:
            banner.set_visible(False)
        return moving

    update(0)
    return FuncAnimation(fig, update, frames=len(timeline), interval=move_ms / substeps,
                         blit=True, repeat=False)


if __name__ == "__main__":
    field = build_default_field()
    make_gif = "--gif" in sys.argv
    want_animation = "--animate" in sys.argv or make_gif

    nav = navigate(field, crop_safe=True, record_frames=want_animation)
    rows = comparison_rows(field, nav)
    print_report(nav, rows)
    draw_replanning_maps(field, nav)
    draw_replanning_results(nav, rows)

    if make_gif:
        print("Saving screenshots/07_replanning.gif (takes about half a minute)...")
        gif = animate(field, nav, substeps=4, pause_frames=10)        # fewer frames -> smaller file
        gif.save("screenshots/07_replanning.gif", writer="pillow", fps=12, dpi=70)
        plt.close(gif._fig)
        print("Saved.")
    if want_animation:
        anim = animate(field, nav)             # keep this variable alive until plt.show()
    plt.show()