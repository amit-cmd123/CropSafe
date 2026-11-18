"""
main.py  -  Iteration 9: run the whole project and produce every screenshot

    python main.py              -> runs every task, saves all screenshots, prints all tables
    python main.py --show       -> same, then opens every figure in a window
    python main.py --animate    -> same as --show, plus the live replanning animation
    python main.py --gif        -> also saves screenshots/07_replanning.gif (~30 s extra)

Screenshots produced (folder: screenshots/):
    01_field.png                     Task 1  field, crop rows, protected zone, obstacles, start, goal
    02_problem.png                   Task 2  problem formulation + legal moves
    03_bfs.png, 03_dfs.png           Task 3  each algorithm's path and expanded nodes
    04_greedy.png, 05_astar.png      Task 3
    05_ucs.png                       extra   Uniform Cost Search on the crop-safe cost
    06_crop_safe.png                 Task 4  plain vs crop-safe A*
    06_crop_safe_tradeoff.png        Task 4  a goal where the safer route is longer
    06_penalty_sweep.png             Task 4  how the penalty weight changes the route
    06_crop_safe_formulation.png     Task 4  formulation with the crop-safe cost
    07_replanning.png                Task 5  hidden obstacle detected, route replanned
    07_replanning_results.png        Task 5  what happened + table
    08_evaluation_table.png          Task 6  comparison tables
    08_evaluation_charts.png         Task 6  bar charts
    09_all_paths.png                 every algorithm's path side by side
"""

import os
import sys

SHOW = any(flag in sys.argv for flag in ("--show", "--animate"))
if not SHOW:
    import matplotlib
    matplotlib.use("Agg")          # no windows: just save the pictures

import matplotlib.pyplot as plt

import crop_safe_demo
from evaluation import (evaluate, evaluate_random, observations, print_table, save_results,
                        draw_tables, draw_charts, MAIN_COLUMNS, RANDOM_COLUMNS, RANDOM_FIELDS)
from field import build_default_field, draw_field
from problem import PathProblem, formulation_report, draw_formulation
from replanning import (navigate, comparison_rows, print_report, draw_replanning_maps,
                        draw_replanning_results, animate, save_gif, mark_detected, DETECTED_HANDLE)
from search.astar import astar
from search.bfs import bfs
from search.dfs import dfs
from search.greedy import greedy
from search.ucs import ucs

KNOWN_MAP_NOTE = "planned on the known map (red X = hidden obstacle the robot cannot see)"


def heading(text):
    print("\n" + "=" * 78 + f"\n{text}\n" + "=" * 78)


def save_single_path(field, result, filename):
    """One algorithm's path + expanded nodes (Task 3 screenshots)."""
    ax = draw_field(field, path=result.path, explored=result.expanded,
                    title=f"{result.algorithm}: {result.path_length} moves, "
                          f"{result.nodes_expanded} nodes expanded\n{KNOWN_MAP_NOTE}",
                    save_path=os.path.join("screenshots", filename))
    return ax.figure


def draw_all_paths(field, results, nav, save_path="screenshots/09_all_paths.png"):
    """Every algorithm's path side by side (2 x 3), one shared legend."""
    scorer = PathProblem(field, crop_safe=True)
    fig, axes = plt.subplots(2, 3, figsize=(20, 11), layout="constrained")
    for ax, r in zip(axes.flat, results):
        draw_field(field, path=r.path, explored=r.expanded, ax=ax, title="")
        ax.set_title(f"{r.algorithm}\n{r.path_length} moves | {r.nodes_expanded} expanded\n"
                     f"crop-safe cost {scorer.path_cost(r.path):.0f}", fontsize=10, fontweight="bold")
        handles = ax.get_legend().legend_handles
        ax.get_legend().remove()

    last = axes.flat[5]
    draw_field(nav.known, path=nav.trail, ax=last, show_hidden=False, title="")
    last.set_title(f"Online A* + replanning (driven)\n{nav.moves} moves | {nav.nodes_expanded} expanded | "
                   f"{nav.replans} replan\ncrop-safe cost {scorer.path_cost(nav.trail):.0f}",
                   fontsize=10, fontweight="bold")
    mark_detected(last, [c for _, _, c in nav.detections])
    last.get_legend().remove()

    fig.legend(handles=list(handles) + [DETECTED_HANDLE], loc="outside lower center",
               ncol=6, fontsize=9, frameon=True)
    fig.suptitle("Task 3 - Paths from every search algorithm\n"
                 "first five planned on the known map, blue dots = expanded nodes",
                 fontsize=13, fontweight="bold")
    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches="tight")
    return fig


def main():
    os.makedirs("screenshots", exist_ok=True)
    field = build_default_field()

    heading("TASK 1 - Field environment")
    field.summary()
    draw_field(field, title="Task 1 - Potato field environment", save_path="screenshots/01_field.png")

    heading("TASK 2 - Problem formulation")
    plain = PathProblem(field, crop_safe=False)
    print("\n".join(formulation_report(plain)))
    draw_formulation(plain, title="Task 2 - Problem formulation", save_path="screenshots/02_problem.png")

    heading("TASK 3 - Search algorithms (planned on the known map)")
    results = [bfs(plain), dfs(plain), greedy(plain), astar(plain, name="A* (plain)"),
               astar(PathProblem(field, crop_safe=True), name="A* (crop-safe)")]
    for r in results:
        r.report()
    for r, name in zip(results[:4], ["03_bfs.png", "03_dfs.png", "04_greedy.png", "05_astar.png"]):
        save_single_path(field, r, name)
        
    # extra baseline: UCS is optimal for the crop-safe cost, like A*, but has no heuristic
    safe = PathProblem(field, crop_safe=True)
    uniform = ucs(safe)
    uniform.report()
    print(f"  crop-safe cost: BFS {safe.path_cost(results[0].path):.0f}, UCS {safe.path_cost(uniform.path):.0f}, "
          f"A* (crop-safe) {safe.path_cost(results[4].path):.0f}  ->  UCS and A* are optimal, "
          f"A* expands {uniform.nodes_expanded - results[4].nodes_expanded} fewer nodes")
    save_single_path(field, uniform, "05_ucs.png")

    heading("TASK 4 - Crop-safe planning")
    crop_safe_demo.main(show=False)

    heading("TASK 5 - Hidden obstacle detection and replanning")
    nav = navigate(field, crop_safe=True, record_frames=True)
    rows5 = comparison_rows(field, nav)
    print_report(nav, rows5)
    draw_replanning_maps(field, nav)
    draw_replanning_results(nav, rows5)
    draw_all_paths(field, results, nav)
    if "--gif" in sys.argv:
        print("\nSaving screenshots/07_replanning.gif (about half a minute)...")
        save_gif(field, nav)

    heading("TASK 6 - Performance evaluation")
    rows6 = evaluate(field)
    print(f"Averaging over {RANDOM_FIELDS} random layouts...")
    random_rows = evaluate_random()
    obs = observations(rows6, random_rows)
    print_table("PERFORMANCE COMPARISON - default field (crop-safe cost, median time)", rows6, MAIN_COLUMNS)
    print_table(f"AVERAGE OVER {RANDOM_FIELDS} RANDOM LAYOUTS", random_rows, RANDOM_COLUMNS)
    print("\nOBSERVATIONS")
    for o in obs:
        print(f"  - {o}")
    save_results(rows6, random_rows, obs)
    draw_tables(rows6, random_rows)
    draw_charts(rows6)

    heading("DONE")
    for name in sorted(os.listdir("screenshots")):
        print(f"  screenshots/{name}")
    print("  results/results_table.md\n  results/results.csv")

    if "--animate" in sys.argv:
        anim = animate(field, nav)         # keep the variable alive until plt.show()
    if SHOW:
        print("\nClose the windows to finish.")
        plt.show()


if __name__ == "__main__":
    main()