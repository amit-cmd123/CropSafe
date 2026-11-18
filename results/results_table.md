## Performance comparison (default field)

All routes scored with the crop-safe cost. Time = median of 30 runs.

| Algorithm | Reached goal | Moves | Crop-safe cost | Cells next to crops | Nodes expanded | Time (ms) | Replans | Hits hidden obstacle |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| BFS | yes | 25 | 62 | 6 | 209 | 0.857 | 0 | yes |
| DFS | yes | 87 | 225 | 35 | 147 | 0.609 | 0 | no |
| UCS | yes | 27 | 60 | 3 | 206 | 1.068 | 0 | yes |
| Greedy | yes | 27 | 60 | 3 | 28 | 0.147 | 0 | yes |
| A* (plain) | yes | 25 | 62 | 6 | 31 | 0.184 | 0 | yes |
| A* (crop-safe) | yes | 27 | 60 | 3 | 175 | 0.981 | 0 | yes |
| Online A* + replanning | yes | 29 | 70 | 6 | 201 | 1.164 | 1 | no |

_Offline planners plan once on the known map: 'Reached goal' means their plan reaches the goal on that map.
'Hits hidden obstacle' shows whether the real robot would drive into the unknown obstacle ('for real' = the plan reaches the goal without hitting it)._

## Average over 100 random field layouts

Each layout adds 20 random known obstacles and 3 random hidden obstacles. Moves and cost are averaged over successful runs.

| Algorithm | Reached goal on known map (%) | Reached goal for real (%) | Moves | Crop-safe cost | Nodes expanded | Time (ms) | Replans |
|---|---:|---:|---:|---:|---:|---:|---:|
| BFS | 99.0 | 63.0 | 26.1 | 77.4 | 173.2 | 0.657 | 0 |
| DFS | 99.0 | 29.0 | 75.7 | 223.6 | 122.1 | 0.558 | 0 |
| UCS | 99.0 | 64.0 | 27.8 | 70.7 | 175.5 | 0.986 | 0 |
| Greedy | 99.0 | 65.0 | 28.9 | 77.7 | 35.5 | 0.200 | 0 |
| A* (plain) | 99.0 | 61.0 | 26.1 | 76.8 | 47.6 | 0.283 | 0 |
| A* (crop-safe) | 99.0 | 67.0 | 28.3 | 70.7 | 155.7 | 0.934 | 0 |
| Online A* + replanning | 97.0 | 97.0 | 30.0 | 75.3 | 186.5 | 1.141 | 0.4 |

## Observations

- BFS finds the shortest route (25 moves) but expands 209 nodes - more than any other planner.
- DFS reaches the goal but its route is 87 moves long, 3.5x the shortest route, with 35 cells next to crops. DFS is not optimal.
- Greedy expands only 28 nodes, the fewest, but its route is 27 moves (shortest is 25): fast but not optimal.
- Plain A* matches BFS's 25 moves while expanding 31 nodes, 85% fewer than BFS.
- BFS is optimal only when every move costs the same: under the crop-safe cost its route costs 62, while UCS finds the cheapest (60). Crop-safe A* finds the same cost as UCS with 175 nodes instead of 206, 15% fewer: the heuristic still helps when moves have different costs.
- Crop-safe A* has the lowest crop-safe cost (60, tied with UCS, vs 62 for plain A*) and passes 3 cells next to crops instead of 6, for 2 extra moves (27 instead of 25). The price is also more search: 175 nodes instead of 31.
- BFS, UCS, Greedy, A* (plain), A* (crop-safe) plan a route through the hidden obstacle, because they only know the known map (DFS misses it by chance). Online A* senses it, replans 1 time(s) and reaches the goal in 29 moves.
- Over 100 random layouts, offline crop-safe A* would really reach the goal in only 67% of them, because hidden obstacles block its plan. Online A* with replanning really reaches it in 97% - every layout where the goal is reachable at all.
