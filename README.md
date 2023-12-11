
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
