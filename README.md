# Routing with dijkstra
[Dijkstra's algorithm](https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm) is an algorithm for finding the shortest paths between nodes in a graph.

## Node
Each Node represents a point in a network or something similar.\
Node structre:
* name: string
* neighbors: list of tuples, each tuple includes the name of one neighbor and its distance
Example: NodeA = ('A', [('B',5), ('C',3), ('D',6)])

## Graph
The graph is a directory with every node and their neighbors.

## Dijkstra
To calculate the distance to every node the algorithm needs a starting node.\
The dijkstra algorithm is a greedy-algorithm and its runtime increases exonential with the number of nodes.

### Main functions
* *self.calc_new_graph(start_node)*
Calculates the shortest route from start node to every node.
* *get_shortest_route_to(destination_node)*
Prints the shortest route from the start node to the destination node. The runtime of this method is very low. As long as the starting node is the same the calculated graph is also the same.

## RasterNodes modul
Convert each pixel from a PIL image into node which can be used in a graph for the dijkstra algorithm.

## Main
The main function creates a random PIL image, converts it into graph with nodes for each pixel. The pixel size was set to 5 meter. The image represents a digital terrain model. Each pixel value represents the hight in this digital terrain model.\
The dijkstra algorithm calculates the shortest path from the starting node (0,0) to the destination node (n,m) and returns the path and an image with a red colorized path.