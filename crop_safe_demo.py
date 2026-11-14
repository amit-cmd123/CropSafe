"""
crop_safe_demo.py  -  Iteration 6 (Task 4): does the safety penalty change the route?

Runs A* twice on the same field:
  1. plain A*     : every move costs 1
  2. crop-safe A* : every move costs 1 + proximity penalty
and scores BOTH routes with the crop-safe cost, so they can be compared fairly.

Pages:
  1. default goal (4, 16)  : same length, half the cells next to crops
  2. trade-off goal (1, 16): the safe route is LONGER but avoids the crops
  3. penalty-weight sweep  : how strong the penalty must be to change the route
  4. the crop-safe problem formulation
"""

import matplotlib.pyplot as plt
import numpy as np

from field import build_default_field, draw_field, SAFE
from problem import PathProblem, DEFAULT_PENALTIES, formulation_report, draw_formulation
from search.astar import astar

TRADEOFF_GOAL = (1, 16)                        # a goal where safety costs extra moves
SWEEP_WEIGHTS = [0, 0.05, 0.1, 0.5, 1, 3]      # multiplies the default penalties (+3 / +1)

HEADER_GREEN = "#3f7d20"


def overlay_penalty(ax, field, penalty):
    """Shade driveable cells by their proximity penalty (orange = close to crops)."""
    shaded = np.ma.masked_where((penalty == 0) | (field.grid != SAFE), penalty)
    ax.imshow(shaded, cmap="Oranges", alpha=0.35, vmin=0, vmax=penalty.max() + 1, zorder=2)


def style_table(table, n_cols, highlight_row=None):
    table.auto_set_font_size(False)
    table.set_fontsize(11)
    for col in range(n_cols):
        table[0, col].set_facecolor(HEADER_GREEN)
        table[0, col].set_text_props(color="white", fontweight="bold")
        if highlight_row is not None:
            table[highlight_row, col].set_facecolor("#e8f4e0")


def compare_routes(field, title, save_path, show_hidden=True):
    """One page: plain A* vs crop-safe A* on the same field, with a table underneath."""
    plain_problem = PathProblem(field, crop_safe=False)
    safe_problem = PathProblem(field, crop_safe=True)
    plain = astar(plain_problem, name="A* (plain)")
    safe = astar(safe_problem, name="A* (crop-safe)")

    headers = ["Planner", "Moves", "Crop-safe cost", "Cells next to crops", "Nodes expanded"]
    rows = [[r.algorithm, r.path_length, f"{safe_problem.path_cost(r.path):.0f}",
             safe_problem.cells_near_crops(r.path), r.nodes_expanded] for r in (plain, safe)]

    print(f"\n{title}  (goal {field.goal}, both routes scored with the crop-safe cost)")
    print(f"{headers[0]:<16}| {headers[1]:>5} | {headers[2]:>14} | {headers[3]:>19} | {headers[4]:>14}")
    print("-" * 80)
    for row in rows:
        print(f"{row[0]:<16}| {row[1]:>5} | {row[2]:>14} | {row[3]:>19} | {row[4]:>14}")

    fig = plt.figure(figsize=(20, 9), layout="constrained")
    grid = fig.add_gridspec(2, 2, height_ratios=[7, 1.3])
    axes = [fig.add_subplot(grid[0, 0]), fig.add_subplot(grid[0, 1])]
    table_ax = fig.add_subplot(grid[1, :])
    table_ax.axis("off")
    table = table_ax.table(cellText=[[str(v) for v in row] for row in rows], colLabels=headers,
                           loc="center", cellLoc="center", colWidths=[0.16] * len(headers))
    style_table(table, len(headers), highlight_row=2)
    table.scale(1, 1.8)

    for ax, r in zip(axes, (plain, safe)):
        near = safe_problem.cells_near_crops(r.path)
        draw_field(field, path=r.path, ax=ax, show_hidden=show_hidden,
                   title=f"{r.algorithm}: {r.path_length} moves, "
                         f"{near} cell{'' if near == 1 else 's'} next to crops, "
                         f"safe cost {safe_problem.path_cost(r.path):.0f}")
        overlay_penalty(ax, field, safe_problem.penalty)
    axes[0].get_legend().remove()          # one legend is enough
    fig.suptitle(f"{title}\norange shading = proximity penalty (darker = closer to crops)",
                 fontsize=13, fontweight="bold")
    fig.savefig(save_path, dpi=150, bbox_inches="tight")
    return plain, safe


def penalty_sweep(save_path="screenshots/06_penalty_sweep.png"):
    """
    Run crop-safe A* with the penalty scaled by each weight, for both goals.
    Weight 0 = plain A*. Weight 1 = the default penalties (+3 next to a crop, +1 two cells away).
    Every route is scored with the default crop-safe cost so the rows compare fairly.
    """
    goals = [((4, 16), "default goal (4, 16)"), (TRADEOFF_GOAL, f"trade-off goal {TRADEOFF_GOAL}")]
    rows = []
    for w in SWEEP_WEIGHTS:
        penalties = {d: p * w for d, p in DEFAULT_PENALTIES.items()}
        row = [f"{w:g}", f"+{penalties[1]:g} / +{penalties[2]:g}"]
        for goal, _ in goals:
            f = build_default_field()
            f.set_goal(goal)
            scorer = PathProblem(f, crop_safe=True)
            r = astar(PathProblem(f, crop_safe=True, penalties=penalties))
            row += [r.path_length, scorer.cells_near_crops(r.path), r.nodes_expanded]
        rows.append(row)

    headers = ["Weight", "Penalty\n(next to / 2 away)",
               "Moves", "Cells next\nto crops", "Nodes\nexpanded",
               "Moves", "Cells next\nto crops", "Nodes\nexpanded"]
    print("\nPENALTY-WEIGHT SWEEP (crop-safe A*, weight x default penalties)")
    print(f"{'':<26}| {goals[0][1]:^34} | {goals[1][1]:^34}")
    print(f"{'Weight':<7}{'Penalty':<19}| {'moves':>8} {'near crops':>11} {'expanded':>10}   "
          f"| {'moves':>8} {'near crops':>11} {'expanded':>10}")
    for row in rows:
        print(f"{row[0]:<7}{row[1]:<19}| {row[2]:>8} {row[3]:>11} {row[4]:>10}   "
              f"| {row[5]:>8} {row[6]:>11} {row[7]:>10}")

    fig, ax = plt.subplots(figsize=(16, 6), layout="constrained")
    ax.axis("off")
    table = ax.table(cellText=[[str(v) for v in row] for row in rows], colLabels=headers,
                     cellLoc="center", bbox=[0, 0, 1, 0.82],
                     colWidths=[0.08, 0.16] + [0.126] * 6)
    style_table(table, len(headers), highlight_row=SWEEP_WEIGHTS.index(1) + 1)
    # group labels above the two halves of the table
    ax.text(0.24 + 0.189, 0.86, goals[0][1], ha="center", va="bottom", fontsize=11, fontweight="bold",
            transform=ax.transAxes)
    ax.text(0.24 + 0.567, 0.86, goals[1][1], ha="center", va="bottom", fontsize=11, fontweight="bold",
            transform=ax.transAxes)
    fig.suptitle("Task 4 - How strong must the crop penalty be?\n"
                 "weight 0 = plain A*, weight 1 = default (+3 next to a crop, +1 two cells away); "
                 "highlighted row = default",
                 fontsize=13, fontweight="bold")
    fig.savefig(save_path, dpi=150, bbox_inches="tight")
    return rows


def main(show=True):
    # page 1: the default field and goal
    field = build_default_field()
    compare_routes(field, "Task 4 - Crop-safe planning on the default field",
                   "screenshots/06_crop_safe.png")

    # page 2: a goal where the safe route has to be longer
    tradeoff = build_default_field()
    tradeoff.set_goal(TRADEOFF_GOAL)
    compare_routes(tradeoff, f"Task 4 - Trade-off: goal {TRADEOFF_GOAL}, the safer route is longer",
                   "screenshots/06_crop_safe_tradeoff.png", show_hidden=False)

    # page 3: how the penalty weight changes the route
    penalty_sweep()

    # page 4: the problem formulation with crop-safe cost
    safe_problem = PathProblem(field, crop_safe=True)
    print()
    print("\n".join(formulation_report(safe_problem)))
    draw_formulation(safe_problem, title="Crop-safe problem formulation",
                     save_path="screenshots/06_crop_safe_formulation.png")

    if show:
        plt.show()  # opens all four pages


if __name__ == "__main__":
    main()