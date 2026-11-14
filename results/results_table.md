## Performance comparison (default field)

All routes scored with the crop-safe cost. Time = median of 30 runs.

| Algorithm | Reached goal | Moves | Crop-safe cost | Cells next to crops | Nodes expanded | Time (ms) | Replans | Hits hidden obstacle |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| BFS | yes | 25 | 62 | 6 | 215 | 0.391 | 0 | yes |
| DFS | yes | 91 | 245 | 40 | 152 | 0.361 | 0 | no |
| UCS | yes | 25 | 56 | 3 | 213 | 0.629 | 0 | yes |
| Greedy | yes | 27 | 60 | 3 | 28 | 0.085 | 0 | yes |
| A* (plain) | yes | 25 | 62 | 6 | 31 | 0.109 | 0 | yes |
| A* (crop-safe) | yes | 25 | 56 | 3 | 182 | 0.584 | 0 | yes |
| Online A* + replanning | yes | 25 | 60 | 5 | 203 | 0.694 | 1 | no |

_Offline planners plan once on the known map: 'Reached goal' means their plan reaches the goal on that map.
'Hits hidden obstacle' shows whether the real robot would drive into the unknown obstacle ('for real' = the plan reaches the goal without hitting it)._

## Average over 100 random field layouts

Each layout adds 20 random known obstacles and 3 random hidden obstacles. Moves and cost are averaged over successful runs.

| Algorithm | Reached goal on known map (%) | Reached goal for real (%) | Moves | Crop-safe cost | Nodes expanded | Time (ms) | Replans |
|---|---:|---:|---:|---:|---:|---:|---:|
| BFS | 96.0 | 58.0 | 25.7 | 74.3 | 178.6 | 0.340 | 0 |
| DFS | 96.0 | 20.0 | 86.1 | 257.1 | 135.1 | 0.321 | 0 |
| UCS | 96.0 | 58.0 | 26.8 | 64.2 | 174.9 | 0.501 | 0 |
| Greedy | 96.0 | 55.0 | 28.0 | 74.7 | 31.8 | 0.096 | 0 |
| A* (plain) | 96.0 | 54.0 | 25.7 | 73.2 | 48.2 | 0.153 | 0 |
| A* (crop-safe) | 96.0 | 57.0 | 26.9 | 64.2 | 152.0 | 0.472 | 0 |
| Online A* + replanning | 94.0 | 94.0 | 27.5 | 67.1 | 183.2 | 0.562 | 0.5 |

## Observations

- BFS finds the shortest route (25 moves) but expands 215 nodes - more than any other planner.
- DFS reaches the goal but its route is 91 moves long, 3.6x the shortest route, with 40 cells next to crops. DFS is not optimal.
- Greedy expands only 28 nodes, the fewest, but its route is 27 moves (shortest is 25): fast but not optimal.
- Plain A* matches BFS's 25 moves while expanding 31 nodes, 86% fewer than BFS.
- BFS is optimal only when every move costs the same: under the crop-safe cost its route costs 62, while UCS finds the cheapest (56). Crop-safe A* finds the same cost as UCS with 182 nodes instead of 213, 15% fewer: the heuristic still helps when moves have different costs.
- Crop-safe A* has the lowest crop-safe cost (56, tied with UCS, vs 62 for plain A*) and passes 3 cells next to crops instead of 6, at the same 25 moves. The price is more search: 182 nodes instead of 31.
- BFS, UCS, Greedy, A* (plain), A* (crop-safe) plan a route through the hidden obstacle, because they only know the known map (DFS misses it by chance). Online A* senses it, replans 1 time(s) and reaches the goal in 25 moves.
- Over 100 random layouts, offline crop-safe A* would really reach the goal in only 57% of them, because hidden obstacles block its plan. Online A* with replanning really reaches it in 94% - every layout where the goal is reachable at all.
