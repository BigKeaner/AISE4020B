Handles shortest path computation between nodes.
```python
import networkx as nx

def compute_shortest_route(graph, start, end):
    """Computes the shortest path between two nodes using Dijkstra's algorithm."""
    return nx.shortest_path(graph, source=start, target=end, weight='weight')
```
