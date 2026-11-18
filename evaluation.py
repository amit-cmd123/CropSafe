"""
evaluation.py  -  Iteration 8 (Task 6): performance evaluation

Compares every planner on the SAME field and scores every route with the
SAME crop-safe cost, so the numbers are comparable:

    BFS, DFS, UCS, Greedy, A* (plain), A* (crop-safe) - offline, plan once on the known map
  Online A* with replanning                       - drives, senses, replans (Task 5)

Metrics: goal reached, path length (moves), crop-safe cost, cells next to
crops, nodes expanded, execution time, replans, and whether the planned path
runs into the hidden obstacle.

Timing: one run of a planner takes well under a millisecond, so a single
measurement is mostly noise. Each planner is run REPEATS times and the
median time is reported.

A second table averages the same metrics over many random field layouts,
to check that the trends are not an accident of one map.

Run:
    python evaluation.py
Outputs:
    screenshots/08_evaluation_table.png   - both tables
    screenshots/08_evaluation_charts.png  - bar charts
    results/results_table.md              - tables in Markdown (paste into README)
    results/results.csv                   - raw numbers
"""

import csv
import os
import random
import statistics
import time

import matplotlib.pyplot as plt

from field import build_default_field, OBSTACLE, SAFE
from problem import PathProblem
from replanning import navigate
from search.astar import astar
from search.bfs import bfs
from search.dfs import dfs
from search.greedy import greedy
from search.ucs import ucs

REPEATS = 30              # timing repetitions per planner
RANDOM_FIELDS = 100       # layouts for the robustness table
RANDOM_SEED = 42          # fixed seed -> the same random layouts every run

ORDER = ["BFS", "DFS", "UCS", "Greedy", "A* (plain)", "A* (crop-safe)", "Online A* + replanning"]


# ----------------------------------------------------------------------
# Running the planners
# ----------------------------------------------------------------------
def median_time_ms(fn, repeats=REPEATS):
    times = []
    for _ in range(repeats):
        t0 = time.perf_counter()
        fn()
        times.append((time.perf_counter() - t0) * 1000)
    return statistics.median(times)


def evaluate(field, repeats=REPEATS):
    """Run every planner on one field. Returns one dict of metrics per planner."""
    plain = PathProblem(field, crop_safe=False)
    safe = PathProblem(field, crop_safe=True)        # used to SCORE every route
    hidden = field.hidden_obstacles

    offline = [
        ("BFS", lambda: bfs(safe)),
        ("DFS", lambda: dfs(safe)),
        ("UCS", lambda: ucs(safe)),
        ("Greedy", lambda: greedy(safe)),
        ("A* (plain)", lambda: astar(plain, name="A* (plain)")),
        ("A* (crop-safe)", lambda: astar(safe, name="A* (crop-safe)")),
    ]

    rows = []
    for name, run in offline:
        r = run()
        rows.append({
            "Algorithm": name,
            "Reached goal": r.found,
            "Moves": r.path_length if r.found else None,
            "Crop-safe cost": safe.path_cost(r.path) if r.found else None,
            "Cells next to crops": safe.cells_near_crops(r.path) if r.found else None,
            "Nodes expanded": r.nodes_expanded,
            "Time (ms)": median_time_ms(run, repeats),
            "Replans": 0,
            "Hits hidden obstacle": any(c in hidden for c in r.path),
            "path": r.path,
        })

    nav = navigate(field, crop_safe=True)
    nav_times = [navigate(field, crop_safe=True).planning_time_ms for _ in range(repeats)]
    rows.append({
        "Algorithm": "Online A* + replanning",
        "Reached goal": nav.reached,
        "Moves": nav.moves if nav.reached else None,
        "Crop-safe cost": safe.path_cost(nav.trail) if nav.reached else None,
        "Cells next to crops": safe.cells_near_crops(nav.trail) if nav.reached else None,
        "Nodes expanded": nav.nodes_expanded,
        "Time (ms)": statistics.median(nav_times),     # A* planning time, all runs summed
        "Replans": nav.replans,
        "Hits hidden obstacle": any(c in hidden for c in nav.trail),   # measured on the route driven             # it senses and drives around it
        "path": nav.trail,
    })
    return rows


def random_field(rng, extra_obstacles=20, hidden_obstacles=3):
    """The default field plus random known and hidden obstacles."""
    f = build_default_field()
    f.hidden_obstacles = set()
    free = [(r, c) for r in range(f.rows) for c in range(f.cols)
            if f.grid[r, c] == SAFE and (r, c) not in (f.start, f.goal)]
    picks = rng.sample(free, extra_obstacles + hidden_obstacles)
    for cell in picks[:extra_obstacles]:
        f.grid[cell] = OBSTACLE
    for cell in picks[extra_obstacles:]:
        f.hidden_obstacles.add(cell)
    return f


def evaluate_random(n=RANDOM_FIELDS, seed=RANDOM_SEED):
    """Average each metric over n random layouts (route metrics over successful runs only)."""
    rng = random.Random(seed)
    per_algo = {name: [] for name in ORDER}
    for _ in range(n):
        for row in evaluate(random_field(rng), repeats=3):
            per_algo[row["Algorithm"]].append(row)

    def mean(rows, key):
        values = [r[key] for r in rows if r[key] is not None]
        return statistics.mean(values) if values else None

    summary = []
    for name in ORDER:
        rows = per_algo[name]
        ok = [r for r in rows if r["Reached goal"]]
        real = [r for r in ok if not r["Hits hidden obstacle"]]   # the robot could really drive it
        summary.append({
            "Algorithm": name,
            "Reached goal on known map (%)": 100 * len(ok) / len(rows),
            "Reached goal for real (%)": 100 * len(real) / len(rows),
            "Moves": mean(ok, "Moves"),
            "Crop-safe cost": mean(ok, "Crop-safe cost"),
            "Nodes expanded": mean(rows, "Nodes expanded"),
            "Time (ms)": mean(rows, "Time (ms)"),
            "Replans": mean(rows, "Replans"),
        })
    return summary


# ----------------------------------------------------------------------
# Formatting
# ----------------------------------------------------------------------
MAIN_COLUMNS = ["Algorithm", "Reached goal", "Moves", "Crop-safe cost", "Cells next to crops",
                "Nodes expanded", "Time (ms)", "Replans", "Hits hidden obstacle"]
RANDOM_COLUMNS = ["Algorithm", "Reached goal on known map (%)", "Reached goal for real (%)", "Moves",
                  "Crop-safe cost", "Nodes expanded", "Time (ms)", "Replans"]
FOOTNOTE = ("Offline planners plan once on the known map: 'Reached goal' means their plan reaches the goal on that map.\n"
            "'Hits hidden obstacle' shows whether the real robot would drive into the unknown obstacle "
            "('for real' = the plan reaches the goal without hitting it).")


def fmt(value, key):
    if value is None:
        return "-"
    if isinstance(value, bool):
        return "yes" if value else "no"
    if key == "Time (ms)":
        return f"{value:.3f}"
    if isinstance(value, float):
        return f"{value:.1f}" if key != "Crop-safe cost" or not value.is_integer() else f"{value:.0f}"
    return str(value)


def as_text_rows(rows, columns):
    return [[fmt(row[c], c) for c in columns] for row in rows]


def print_table(title, rows, columns):
    data = as_text_rows(rows, columns)
    widths = [max(len(c), *(len(r[i]) for r in data)) for i, c in enumerate(columns)]
    print(f"\n{title}")
    print(" | ".join(c.ljust(w) if i == 0 else c.rjust(w) for i, (c, w) in enumerate(zip(columns, widths))))
    print("-+-".join("-" * w for w in widths))
    for r in data:
        print(" | ".join(v.ljust(w) if i == 0 else v.rjust(w) for i, (v, w) in enumerate(zip(r, widths))))


def markdown_table(rows, columns):
    data = as_text_rows(rows, columns)
    lines = ["| " + " | ".join(columns) + " |",
             "|" + "|".join(["---"] + ["---:"] * (len(columns) - 1)) + "|"]
    lines += ["| " + " | ".join(r) + " |" for r in data]
    return "\n".join(lines)


def observations(rows, random_rows):
    """Plain-language findings computed from the tables."""
    by = {r["Algorithm"]: r for r in rows}
    bfs_r, dfs_r, ucs_r, gr, ap, asafe, online = (by[n] for n in ORDER)
    most = max(rows, key=lambda r: r["Nodes expanded"])
    hit = [r["Algorithm"] for r in rows if r["Hits hidden obstacle"]]
    missed = [r["Algorithm"] for r in rows[:-1] if not r["Hits hidden obstacle"]]
    rnd = {r["Algorithm"]: r for r in random_rows}
    obs = [
        f"BFS finds the shortest route ({bfs_r['Moves']} moves) but expands {bfs_r['Nodes expanded']} nodes"
        + (" - more than any other planner." if most is bfs_r else "."),
        f"DFS reaches the goal but its route is {dfs_r['Moves']} moves long, "
        f"{dfs_r['Moves'] / bfs_r['Moves']:.1f}x the shortest route, with {dfs_r['Cells next to crops']} "
        f"cells next to crops. DFS is not optimal.",
        f"Greedy expands only {gr['Nodes expanded']} nodes, the fewest, but its route is {gr['Moves']} moves "
        f"(shortest is {bfs_r['Moves']}): fast but not optimal.",
        f"Plain A* matches BFS's {ap['Moves']} moves while expanding {ap['Nodes expanded']} nodes, "
        f"{100 * (1 - ap['Nodes expanded'] / bfs_r['Nodes expanded']):.0f}% fewer than BFS.",
                f"BFS is optimal only when every move costs the same: under the crop-safe cost its route costs "
        f"{bfs_r['Crop-safe cost']:.0f}, while UCS finds the cheapest ({ucs_r['Crop-safe cost']:.0f}). "
        f"Crop-safe A* finds the same cost as UCS with {asafe['Nodes expanded']} nodes instead of "
        f"{ucs_r['Nodes expanded']}, {100 * (1 - asafe['Nodes expanded'] / ucs_r['Nodes expanded']):.0f}% fewer: "
        f"the heuristic still helps when moves have different costs.",
        f"Crop-safe A* has the lowest crop-safe cost ({asafe['Crop-safe cost']:.0f}"
        + (", tied with UCS" if asafe['Crop-safe cost'] == ucs_r['Crop-safe cost'] else "")
        + f", vs "
        f"{ap['Crop-safe cost']:.0f} for plain A*) and passes {asafe['Cells next to crops']} cells next to "
        f"crops instead of {ap['Cells next to crops']}, "
        + (f"at the same {asafe['Moves']} moves. " if asafe['Moves'] == ap['Moves'] else
           f"for {asafe['Moves'] - ap['Moves']} extra moves ({asafe['Moves']} instead of {ap['Moves']}). ")
        + f"The price is also more search: {asafe['Nodes expanded']} nodes instead of {ap['Nodes expanded']}.",
        f"{', '.join(hit)} plan a route through the hidden obstacle, because they only know the known map"
        + (f" ({' and '.join(missed)} {'misses' if len(missed) == 1 else 'miss'} it by chance)." if missed else ".")
        + f" Online A* senses it, replans {online['Replans']} time(s) and reaches the goal in {online['Moves']} moves.",
        f"Over {RANDOM_FIELDS} random layouts, offline crop-safe A* would really reach the goal in only "
        f"{rnd['A* (crop-safe)']['Reached goal for real (%)']:.0f}% of them, because hidden obstacles block "
        f"its plan. Online A* with replanning really reaches it in "
        f"{rnd['Online A* + replanning']['Reached goal for real (%)']:.0f}% - every layout where the goal "
        f"is reachable at all.",
    ]
    return obs


def save_results(rows, random_rows, obs, md_path="results/results_table.md", csv_path="results/results.csv"):
    os.makedirs(os.path.dirname(md_path), exist_ok=True)
    with open(md_path, "w", encoding="utf-8") as f:
        f.write("## Performance comparison (default field)\n\n")
        f.write(f"All routes scored with the crop-safe cost. Time = median of {REPEATS} runs.\n\n")
        f.write(markdown_table(rows, MAIN_COLUMNS) + "\n\n")
        f.write(f"_{FOOTNOTE}_\n\n")
        f.write(f"## Average over {RANDOM_FIELDS} random field layouts\n\n")
        f.write("Each layout adds 20 random known obstacles and 3 random hidden obstacles. "
                "Moves and cost are averaged over successful runs.\n\n")
        f.write(markdown_table(random_rows, RANDOM_COLUMNS) + "\n\n")
        f.write("## Observations\n\n")
        f.write("\n".join(f"- {o}" for o in obs) + "\n")
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(MAIN_COLUMNS)
        for row in rows:
            writer.writerow([row[c] for c in MAIN_COLUMNS])


# ----------------------------------------------------------------------
# Figures
# ----------------------------------------------------------------------
HEADER_GREEN = "#3f7d20"
BAR_BLUE = "#2a78d6"
INK = "#0b0b0b"
INK_MUTED = "#52514e"


def style_table(table, n_cols, highlight_rows=()):
    table.auto_set_font_size(False)
    table.set_fontsize(10)
    for col in range(n_cols):
        table[0, col].set_facecolor(HEADER_GREEN)
        table[0, col].set_text_props(color="white", fontweight="bold")
        for r in highlight_rows:
            table[r, col].set_facecolor("#e8f4e0")
    for (r, c), cell in table.get_celld().items():
        cell.set_edgecolor("#bbbbbb")
        if c == 0 and r > 0:
            cell.set_text_props(ha="left")
            cell.PAD = 0.03


HEADER_LABELS = {      # two-line column headers for the figure
    "Reached goal": "Reached\ngoal",
    "Crop-safe cost": "Crop-safe\ncost",
    "Cells next to crops": "Cells next\nto crops",
    "Nodes expanded": "Nodes\nexpanded",
    "Time (ms)": "Time\n(ms)",
    "Hits hidden obstacle": "Hits hidden\nobstacle",
    "Reached goal on known map (%)": "Reached goal\nknown map %",
    "Reached goal for real (%)": "Reached goal\nfor real %",
}


def wrap_header(name):
    return HEADER_LABELS.get(name, name)


def draw_tables(rows, random_rows, save_path="screenshots/08_evaluation_table.png"):
    fig = plt.figure(figsize=(16, 9), layout="constrained")
    grid = fig.add_gridspec(2, 1, height_ratios=[1, 1])
    for ax, data, columns, title in (
            (fig.add_subplot(grid[0]), rows, MAIN_COLUMNS,
             f"Default field - all routes scored with the crop-safe cost (time = median of {REPEATS} runs)"),
            (fig.add_subplot(grid[1]), random_rows, RANDOM_COLUMNS,
             f"Average over {RANDOM_FIELDS} random layouts (20 random obstacles + 3 hidden obstacles each)")):
        ax.axis("off")
        widths = [0.22] + [(1 - 0.22) / (len(columns) - 1)] * (len(columns) - 1)
        table = ax.table(cellText=as_text_rows(data, columns), colLabels=[wrap_header(c) for c in columns],
                         cellLoc="center", bbox=[0, 0, 1, 0.92], colWidths=widths)
        style_table(table, len(columns), highlight_rows=(6, 7))
        ax.set_title(title, fontsize=11, fontweight="bold", loc="left")
    fig.suptitle("Task 6 - Performance evaluation", fontsize=14, fontweight="bold")
    fig.supxlabel(FOOTNOTE, fontsize=9, color=INK_MUTED, x=0.0, ha="left")   # reserved space under the tables
    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches="tight")
    return fig


def draw_charts(rows, save_path="screenshots/08_evaluation_charts.png"):
    """Three small bar charts - one metric each, same algorithm order in every panel."""
    metrics = [("Nodes expanded", "Nodes expanded (lower = less search)", "{:.0f}"),
               ("Moves", "Path length in moves (lower = shorter)", "{:.0f}"),
               ("Crop-safe cost", "Crop-safe cost (lower = safer + shorter)", "{:.0f}")]
    names = [r["Algorithm"] for r in rows]
    fig, axes = plt.subplots(1, 3, figsize=(18, 6), layout="constrained", sharey=True)
    for ax, (key, title, pattern) in zip(axes, metrics):
        values = [r[key] or 0 for r in rows]
        y = range(len(names))
        ax.barh(y, values, height=0.62, color=BAR_BLUE, edgecolor="white", linewidth=2)
        for yi, v in zip(y, values):
            ax.text(v, yi, "  " + pattern.format(v), va="center", ha="left", fontsize=10, color=INK)
        ax.set_yticks(list(y), names, fontsize=10, color=INK)
        ax.invert_yaxis()                                    # first algorithm at the top
        ax.set_title(title, fontsize=11, fontweight="bold", loc="left", color=INK)
        ax.set_xlim(0, max(values) * 1.18)
        ax.grid(axis="x", color="#e5e5e5", linewidth=0.8)
        ax.set_axisbelow(True)
        for side in ("top", "right", "left"):
            ax.spines[side].set_visible(False)
        ax.spines["bottom"].set_color("#bbbbbb")
        ax.tick_params(axis="x", colors=INK_MUTED, labelsize=9)
        ax.tick_params(axis="y", length=0)
    fig.suptitle("Task 6 - How the planners compare on the default field", fontsize=14, fontweight="bold")
    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches="tight")
    return fig


if __name__ == "__main__":
    field = build_default_field()
    print("Evaluating the default field...")
    rows = evaluate(field)
    print(f"Evaluating {RANDOM_FIELDS} random layouts (a few seconds)...")
    random_rows = evaluate_random()
    obs = observations(rows, random_rows)

    print_table("PERFORMANCE COMPARISON - default field (crop-safe cost, median time)", rows, MAIN_COLUMNS)
    print_table(f"AVERAGE OVER {RANDOM_FIELDS} RANDOM LAYOUTS", random_rows, RANDOM_COLUMNS)
    print("\nOBSERVATIONS")
    for o in obs:
        print(f"  - {o}")

    save_results(rows, random_rows, obs)
    draw_tables(rows, random_rows)
    draw_charts(rows)
    print("\nSaved: results/results_table.md, results/results.csv, "
          "screenshots/08_evaluation_table.png, screenshots/08_evaluation_charts.png")
    plt.show()