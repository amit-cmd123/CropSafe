"""
search/common.py  -  shared pieces used by every search algorithm.
"""

from dataclasses import dataclass, field as dc_field


@dataclass
class SearchResult:
    algorithm: str
    found: bool                      # did we reach the goal?
    path: list                       # [start, ..., goal] or [] if not found
    cost: float                      # path cost (from problem.path_cost)
    nodes_expanded: int              # how many nodes were taken off the frontier and expanded
    time_ms: float                   # execution time in milliseconds
    expanded: list = dc_field(default_factory=list)   # expansion order (for drawing)

    @property
    def path_length(self):
        """Number of moves (cells in path minus one)."""
        return max(len(self.path) - 1, 0)

    def report(self):
        status = "reached goal" if self.found else "FAILED"
        print(f"{self.algorithm:<15} | {status:<12} | moves {self.path_length:>3} | "
              f"cost {self.cost:>6.1f} | expanded {self.nodes_expanded:>4} | {self.time_ms:.3f} ms")


def reconstruct_path(parent, goal):
    """Follow parent links back from the goal to the start, then reverse."""
    path = [goal]
    while parent[path[-1]] is not None:
        path.append(parent[path[-1]])
    path.reverse()
    return path