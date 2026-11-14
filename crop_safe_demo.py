"""
crop_safe_demo.py  -  Iteration 6 (Task 4): does the safety penalty change the route?

Runs A* twice on the same field:
  1. plain A*     : every move costs 1
  2. crop-safe A* : every move costs 1 + proximity penalty
and scores BOTH routes with the crop-safe cost, so they can be compared fairly.
"""

import matplotlib.pyplot as plt
import numpy as np

from field import build_default_field, draw_field, SAFE
from problem import PathProblem, formulation_report, draw_formulation
from search.astar import astar


def overlay_penalty(ax, field, penalty):
    """Shade driveable cells by their proximity penalty (orange = close to crops)."""
    shaded = np.ma.masked_where((penalty == 0) | (field.grid != SAFE), penalty)
    ax.imshow(shaded, cmap="Oranges", alpha=0.35, vmin=0, vmax=penalty.max() + 1, zorder=2)


def main():
    field = build_default_field()
    plain_problem = PathProblem(field, crop_safe=False)
    safe_problem = PathProblem(field, crop_safe=True)

    plain = astar(plain_problem, name="A* (plain)")
    safe = astar(safe_problem, name="A* (crop-safe)")

    # ---------------- comparison table (console + figure) ----------------
    headers = ["Planner", "Moves", "Crop-safe cost", "Cells next to crops", "Nodes expanded"]
    rows = [[r.algorithm, r.path_length, f"{safe_problem.path_cost(r.path):.0f}",
             safe_problem.cells_near_crops(r.path), r.nodes_expanded] for r in (plain, safe)]

    print("Both routes scored with the CROP-SAFE cost:\n")
    print(f"{headers[0]:<16}| {headers[1]:>5} | {headers[2]:>14} | {headers[3]:>19} | {headers[4]:>14}")
    print("-" * 80)
    for row in rows:
        print(f"{row[0]:<16}| {row[1]:>5} | {row[2]:>14} | {row[3]:>19} | {row[4]:>14}")

    # ---------------- figure: two maps on top, table underneath ----------------
    fig = plt.figure(figsize=(20, 9))
    grid = fig.add_gridspec(2, 2, height_ratios=[7, 1.3])
    axes = [fig.add_subplot(grid[0, 0]), fig.add_subplot(grid[0, 1])]
    table_ax = fig.add_subplot(grid[1, :])
    table_ax.axis("off")

    table = table_ax.table(cellText=[[str(v) for v in row] for row in rows], colLabels=headers,
                           loc="center", cellLoc="center", colWidths=[0.16] * len(headers))
    table.auto_set_font_size(False)
    table.set_fontsize(11)
    table.scale(1, 1.8)
    for col in range(len(headers)):
        table[0, col].set_facecolor("#3f7d20")
        table[0, col].set_text_props(color="white", fontweight="bold")
        table[2, col].set_facecolor("#e8f4e0")       # highlight the crop-safe row

    for ax, r in zip(axes, (plain, safe)):
        draw_field(field, path=r.path, ax=ax,
                   title=f"{r.algorithm}: {r.path_length} moves, "
                         f"{safe_problem.cells_near_crops(r.path)} cells next to crops, "
                         f"safe cost {safe_problem.path_cost(r.path):.0f}")
        overlay_penalty(ax, field, safe_problem.penalty)
    axes[0].get_legend().remove()          # one legend is enough
    fig.suptitle("Crop-safe planning: orange shading = proximity penalty (darker = closer to crops)",
                 fontsize=13, fontweight="bold")
    fig.tight_layout()
    fig.savefig("screenshots/06_crop_safe.png", dpi=150, bbox_inches="tight")

    # ---------------- page 2: the problem formulation with crop-safe cost ----------------
    print()
    print("\n".join(formulation_report(safe_problem)))
    draw_formulation(safe_problem, title="Crop-safe problem formulation",
                     save_path="screenshots/06_crop_safe_formulation.png")

    plt.show()      # opens both pages (two windows)
if __name__ == "__main__":
    main()