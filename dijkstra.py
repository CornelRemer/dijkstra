"""Dijkstra Algorithm

This module calculates the shortest way from a
starting node to any other nodes of a graph.
"""
import copy

from mydecorators import measure_time

INFINITY = float('inf')

class Node:
    """A point from a network or something similar

    The __init__ takes two arguments:
        name (str): Name of a node in human readable string
        neighbors (list): List of tuples with connected node and distance

    Attributes:
        label_previous_node (str): name of previous node
        label_distance (int): Shortest distance from starting node
        visited (boolean): Value True if node has been visited bevor

    
    Example:
        nodeA = Node('A',[('B',4), ('E',3), ('J',4)])

    """

    def __init__(self, name, neighbors):
        self.name = name
        self.neighbors = [(name,dist) for name,dist in neighbors]
        self.visited = False
        self._change_node_label()
    
    def set_label_distance(self, distance):
        self.label_distance = distance
    
    def get_label_distance(self):
        return self.label_distance
    
    def set_label_previous_node(self, node_name):
        self.label_previous_node = node_name
    
    def get_label_previous_node(self):
        return self.label_previous_node

    def _change_node_label(self, previous_node=None, distance=INFINITY):
        self.set_label_previous_node(previous_node)
        self.set_label_distance(distance)
    
    def get_name(self):
        return self.name

    def set_start_node(self, start_distance=0):
        self._change_node_label(None, start_distance)
        return self

    def get_neighbors(self):
        """Returns a list of tupels.
        
        Each tupel contains a neighbors name (str) and it's distance (int):
            (name, distance)
        
        """
        return self.neighbors

    def check_infinity(self):
        return self.label_distance < INFINITY
    
    def set_visited(self):
        self.visited = True


class Graph:
    """A Graph consists of nodes.

    Attributes:
        graph (dir): Contains all nodes
    """
    def __init__(self):
        self.graph = {}
    
    def add_one_node(self,node):
        """Add single node

        Args:
            node (obj): Node object
        """
        self.graph[node.name] = node

    def add_nodes(self, nodes):
        """Add multiple nodes at ones

        Args:
            nodes (list): Node objects
        """
        for node in nodes:
            self.add_one_node(node)
    
    def del_one_node(self, node):
        """Deletes one node from the graoh

        Args:
            node (obj): Node object
        """
        try:
            del self.graph[node.get_name()]
        except KeyError:
            print('KeyError: Could not find Node %s in Graph' % (node.get_name()))
        else:
            print('deleted node %s' % (node.get_name()))
    
    def del_nodes(self, nodes):
        for node in nodes:
            self.del_one_node(node)

    def count_nodes(self):
        """Returns number of nodes in graph"""
        return len(self.graph.keys())

    def get_nodes(self):
        """Returns list with all key values of a graph"""
        return self.graph.keys()
    
    def print_graph(self):
        """Print graph in human readable string"""
        print('\n---- created graph ----')
        for key in self.graph:
            print('node: %s (dist: %s), from: %s' % (
                key,
                self.graph[key].label_distance,
                self.graph[key].label_previous_node))

    def _validate_node(self, node):
        return not self.graph[node].visited and self.graph[node].check_infinity()
    
    def _min_node_distance(self, nodes):
        closest_node = nodes[0]
        for node in nodes:
            if node.label_distance < closest_node.label_distance:
                closest_node = node
        
        return closest_node

    def get_closest_node(self):
        """Returns node with lowest labeled distance"""
        nodes = [self.graph[node] for node in self.get_nodes() if self._validate_node(node)]
        return self._min_node_distance(nodes)
    
    def get_node(self, name):
        """Returns a node by its name"""
        return self.graph[name]

class Dijkstra:
    """Algorithm to calculate the shortest path from a starting node
    to every node or to a specific destination node

    The __init__ takes one arguments:
        graph (obj): Graph object which contains all connected nodes

    Example:
        d = Dijkstra(graph)

    """
    def __init__(self, graph):
        self.graph = copy.deepcopy(graph)

        self._vitited_nodes = []
        self._current_node = None
    
    def _set_node_visited(self,node):
        node.set_visited()
        self._vitited_nodes.append(node.get_name())
    
    def _set_start_node(self, node):
        self._start_node = node.set_start_node()
    
    def _get_start_node(self):
        return self._start_node
    
    def _set_destination_node(self, node):
        self._destination_node = self.graph.get_node(node.get_name())
    
    def _all_nodes_visited(self):
        return len(self._vitited_nodes) == self.graph.count_nodes()
    
    def _calc_new_distance(self, distance):
        return self._current_node.get_label_distance() + distance

    def _set_new_shortest_distance(self, node):
        self.graph.get_node(node[0]).set_label_distance(
            self._calc_new_distance(node[1]))
        
        self.graph.get_node(node[0]).set_label_previous_node(self._current_node.get_name())

    @measure_time
    def calc_new_graph(self, start_node):
        """Calculates the shortest route from start node to every node

        CAUTION:
            A large number of nodes can cause a significant  amount of time to calculat!

        Args:
            start_node (obj): Node object from which the distance
            to all other nodes has to be calculated.
        """
        self._set_start_node(start_node)
        self._current_node = self._get_start_node()

        while not self._all_nodes_visited():
            # get a list of tupels with every neighbors name and it's distance
            self.neighbors = self._current_node.get_neighbors()

            for neighbor in self.neighbors:
                self.old_dist = self.graph.get_node(neighbor[0]).get_label_distance()
                self.new_dist = self._calc_new_distance(neighbor[1])

                if self.old_dist > self.new_dist:
                    # change nodes distance label
                    self._set_new_shortest_distance(neighbor)

            self._set_node_visited(self._current_node)

            if self._all_nodes_visited():
                break
            else:
                self._current_node = self.graph.get_closest_node()

    @measure_time
    def get_shortest_route_to(self, destination_node):
        """Prints the shortest route from the start node to the destination node

        Args:
            destination_node (obj): Node object that you want to be routed to.
        """
        self.shortest_way= []
        self._set_destination_node(destination_node)

        if self._destination_node.get_name() is self._start_node.get_name():
            print('\nStart node equals destination node. No travel necessary.')
            return None
        
        print('\nShortest route from %s to %s:\nDistance to travel: %.2f m' % (
            self._start_node.get_name(),
            self._destination_node.get_name(),
            self.graph.get_node(self._destination_node.get_name()).get_label_distance()))

        # add destination node to route
        self.shortest_way.append(self._destination_node.get_name())

        self._latest_node = self._destination_node

        # go backwards from destination node to start node to find the shortest route
        while self.shortest_way[-1] is not self._start_node.get_name():
            self.shortest_way.append(self.graph.get_node(self._latest_node.get_name()).get_label_previous_node())
            self._latest_node = self.graph.get_node(self._latest_node.get_label_previous_node())
            
        self.shortest_way.reverse()
        #print('%s\n' % (self.shortest_way))
        
        return self.shortest_way

if __name__=="__main__":
    ### NODES ###
    nodeA = Node('A', [('B',4), ('C',3)])
    nodeB = Node('B', [('A',4), ('C',1), ('D',3)])
    nodeC = Node('C', [('B',1), ('A',3), ('D',1), ('E',2)])
    nodeD = Node('D', [('C',1), ('B',3), ('E',1)])
    nodeE = Node('E', [('D',3), ('C',2), ('D', 1)])

    nodes = [nodeA, nodeB, nodeC, nodeD, nodeE]

    ### CREATE GRAPH AND ADD NODES ###
    mygraph = Graph()
    mygraph.add_nodes(nodes)

    #mygraph.print_graph()
    
    ### INITIATE DIJKSTRA AND CALCULATE NEW GRAPH ###
    d = Dijkstra(mygraph)
    d.calc_new_graph(nodeA)

    #d.graph.print_graph()

    ### CALCULATE SHORTEST ROUTE TO DESTINATION ###
    d.get_shortest_route_to(nodeE)