
## Language: Python 3.8.5
1. Creating a maze by
- drawing and erasing walls by clicking on the grid
- placing walls randomly according to desired wall density
- generating a circular maze
- generating carved out maze
2. Finding the shortest path through the maze using one of two algorithms and visualizing the search process with desired speed of animation
- A\* Pathfinding Algorithm
- Breadth-First Pathfinding Algorithm
3. Reset the grid if necessary

## overview
A pathfinding algorithm is a computational method used to find the shortest path between two points in a graph or a grid. This is a common problem in computer science and has applications in various fields such as robotics, video games, network routing, and more. Several pathfinding algorithms exist, and each has its own strengths and weaknesses. Here are a few notable ones:

1 Dijkstra's Algorithm:
- Dijkstra's algorithm is a popular and widely used algorithm for finding the shortest path between nodes in a graph.
- It works by maintaining a set of nodes whose shortest distance from the source is known and continually expands this set until the destination is reached.
- This algorithm guarantees the shortest path, but it may not be efficient for large graphs.

2 A Algorithm:*
- A* (pronounced "A star") is an extension of Dijkstra's algorithm that incorporates heuristics to improve efficiency.
- It uses a combination of the actual cost to reach a node (g), the estimated cost from the current node to the destination (h), and a weight factor (usually denoted as w) to determine the next node to explore.
- A* is widely used in robotics and video games due to its efficiency and optimality.

3 Breadth-First Search (BFS):
- BFS is not explicitly a pathfinding algorithm, but it can be used to find the shortest path in an unweighted graph.
- It explores nodes level by level, guaranteeing the shortest path in an unweighted graph.
- It's simple and easy to implement but may not be as efficient as other algorithms for weighted graphs.

4 Depth-First Search (DFS):
- Similar to BFS, DFS is not designed for pathfinding but can be adapted to find paths.
- DFS explores as far as possible along each branch before backtracking.
- It may not guarantee the shortest path and is more suitable for certain types of graphs.

5 Bellman-Ford Algorithm:
- The Bellman-Ford algorithm is capable of handling graphs with negative edge weights.
- It iterates over all edges multiple times, relaxing the distances until it converges to the correct shortest path.
- It's less efficient than D

## references
1. https://www.w3schools.com/
2. https://discord.gg/python
3. https://stackoverflow.com/

## demo  

### Dijkstra's Algorithm: code
```
import heapq

def dijkstra(graph, start):
    # Create a priority queue to store vertices and their distances
    priority_queue = [(0, start)]
    # Dictionary to store the shortest distance to each vertex
    distance = {vertex: float('infinity') for vertex in graph}
    # Set the distance to the start vertex as 0
    distance[start] = 0

    while priority_queue:
        # Get the vertex with the smallest distance
        current_distance, current_vertex = heapq.heappop(priority_queue)

        # Check if the current distance is already greater than the known distance
        if current_distance > distance[current_vertex]:
            continue

        # Update the distance for neighboring vertices
        for neighbor, weight in graph[current_vertex].items():
            distance_to_neighbor = current_distance + weight

            # If a shorter path is found, update the distance
            if distance_to_neighbor < distance[neighbor]:
                distance[neighbor] = distance_to_neighbor
                heapq.heappush(priority_queue, (distance_to_neighbor, neighbor))

    return distance

# Example usage:
graph = {
    'A': {'B': 1, 'C': 4},
    'B': {'A': 1, 'C': 2, 'D': 5},
    'C': {'A': 4, 'B': 2, 'D': 1},
    'D': {'B': 5, 'C': 1}
}

start_vertex = 'A'
result = dijkstra(graph, start_vertex)
print(f"Shortest distances from {start_vertex}: {result}")
```
### A Algorithm:* code
```
import heapq

def astar(graph, start, goal):
    # Priority queue to store nodes and their estimated total cost
    priority_queue = [(0, start)]
    # Dictionary to store the cost from the start to each node
    cost_so_far = {start: 0}

    while priority_queue:
        current_cost, current_node = heapq.heappop(priority_queue)

        if current_node == goal:
            break

        for neighbor, weight in graph[current_node].items():
            new_cost = cost_so_far[current_node] + weight

            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + heuristic(goal, neighbor)
                heapq.heappush(priority_queue, (priority, neighbor))

    return cost_so_far

def heuristic(goal, neighbor):
    # This is a simple heuristic function, you may need to customize it based on your problem
    # For example, you can use Euclidean distance for a 2D grid
    return 0

# Example usage:
graph = {
    'A': {'B': 1, 'C': 4},
    'B': {'A': 1, 'C': 2, 'D': 5},
    'C': {'A': 4, 'B': 2, 'D': 1},
    'D': {'B': 5, 'C': 1}
}

start_node = 'A'
goal_node = 'D'
result = astar(graph, start_node, goal_node)
print(f"Cost from {start_node} to {goal_node}: {result[goal_node]}")
```
### Breadth-First Search (BFS) code
```
from collections import deque

def bfs(graph, start):
    visited = set()  # Set to keep track of visited nodes
    queue = deque([start])  # Queue for BFS traversal

    while queue:
        node = queue.popleft()  # Dequeue a node from the front of the queue
        if node not in visited:
            print(node, end=' ')  # Process the node (you can customize this part)
            visited.add(node)  # Mark the node as visited

            # Enqueue all adjacent nodes that haven't been visited yet
            queue.extend(neighbor for neighbor in graph[node] if neighbor not in visited)

# Example usage:
graph = {
    'A': ['B', 'C'],
    'B': ['A', 'D', 'E'],
    'C': ['A', 'F', 'G'],
    'D': ['B'],
    'E': ['B', 'H'],
    'F': ['C'],
    'G': ['C'],
    'H': ['E']
}

start_node = 'A'
print("BFS Traversal starting from node", start_node)
bfs(graph, start_node)
```
### Depth-First Search (DFS) code
```
def dfs(graph, start, visited=None):
    if visited is None:
        visited = set()

    visited.add(start)
    print(start, end=' ')  # Process the node (you can customize this part)

    for neighbor in graph[start]:
        if neighbor not in visited:
            dfs(graph, neighbor, visited)

# Example usage:
graph = {
    'A': ['B', 'C'],
    'B': ['A', 'D', 'E'],
    'C': ['A', 'F', 'G'],
    'D': ['B'],
    'E': ['B', 'H'],
    'F': ['C'],
    'G': ['C'],
    'H': ['E']
}

start_node = 'A'
print("DFS Traversal starting from node", start_node)
dfs(graph, start_node)
```
### Bellman-Ford Algorithm: code
```
class Graph:
    def __init__(self, vertices):
        self.V = vertices
        self.graph = []

    def add_edge(self, u, v, w):
        self.graph.append([u, v, w])

    def bellman_ford(self, src):
        # Step 1: Initialize distances
        distance = [float("inf")] * self.V
        distance[src] = 0

        # Step 2: Relax all edges V-1 times
        for _ in range(self.V - 1):
            for u, v, w in self.graph:
                if distance[u] != float("inf") and distance[u] + w < distance[v]:
                    distance[v] = distance[u] + w

        # Step 3: Check for negative weight cycles
        for u, v, w in self.graph:
            if distance[u] != float("inf") and distance[u] + w < distance[v]:
                print("Graph contains negative weight cycle")
                return

        # Print the distances
        print("Vertex Distance from Source:")
        for i in range(self.V):
            print(f"{i} \t\t {distance[i]}")

# Example usage:
g = Graph(5)
g.add_edge(0, 1, -1)
g.add_edge(0, 2, 4)
g.add_edge(1, 2, 3)
g.add_edge(1, 3, 2)
g.add_edge(1, 4, 2)
g.add_edge(3, 2, 5)
g.add_edge(3, 1, 1)
g.add_edge(4, 3, -3)

source_vertex = 0
g.bellman_ford(source_vertex)
```
