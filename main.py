from PIL import Image
import random
import time
import pickle

from dijkstra import Node, Graph, Dijkstra
from transformer import RasterNodes
from mydecorators import measure_time

WIDTH = 75

# square images only (for now!!)
HEIGHT = WIDTH

PIXEL_VALUE = 256
PIXEL_SIZE = 5

#@measure_time
def create_random_image(save_image=False):
    """Create a random image"""
    img = Image.new('L', (WIDTH, HEIGHT), PIXEL_VALUE)
    pixels = img.load()
    # random werte eintragen
    for i in range(WIDTH):
        for j in range(HEIGHT):
            pixels[i,j] = random.randint(0,256)
    
    if save_image:
        img.save('calculated_images/%s_%s_start_image.jpg' % (WIDTH,HEIGHT))

    return img

#@measure_time
def colorize_route(img, route, save_image=False):
    """Creates an image with the calculated route in red
    
    Args:
        img (obj): Image object from the PIL module
        route (list): A list including all nodes from the start to the destination point
    
    Return:
        An PIL Image object including the shortest way from start to destination colorized in red
    """
    img = img.convert('RGB')
    pixels = img.load()

    for _ in route:
        x,y = _.split(',')
        pixels[int(x),int(y)] = (255,0,0)

    if save_image:
        img.save('calculated_images/%s_%s_route.jpg' % (WIDTH, HEIGHT))
    
    return img


# Create Data
"""nodeA = Node('A',[('B',4), ('E',3), ('J',4)])
nodeB = Node('B',[('A',4), ('C',4), ('D',1), ('G',8)])
nodeC = Node('C',[('B',4), ('D',2), ('G',3), ('H',5), ('F',3)])
nodeD = Node('D',[('C',2), ('B',1), ('E',3)])
nodeE = Node('E',[('A',3), ('D',3), ('J',2), ('F', 3)])
nodeF = Node('F',[('E',3), ('C',3), ('H',2), ('I',2)])
nodeG = Node('G',[('B',8), ('C',3), ('H', 2)])
nodeH = Node('H',[('G',2), ('C',5), ('F',2), ('I',3)])
nodeI = Node('I',[('H',3), ('F',2), ('J',5)])
nodeJ = Node('J',[('A',4), ('E',2), ('I',5)])

nodes = [nodeA, nodeB, nodeC, nodeD, nodeE, nodeF, nodeG, nodeH, nodeI, nodeJ]
"""

img = create_random_image()

# Convert pixel into nodes
myraster_nodes = RasterNodes(img, PIXEL_SIZE)
nodes = myraster_nodes.transform_to_nodes()

# Create graph and add nodes
mygraph = Graph()
mygraph.add_nodes(nodes)

# Load pickle graph
#with open('./dumped_graphs/dijkstra_graph_strt_node_0,0.pickle', 'rb') as r:
#    d = pickle.load(r)

d = Dijkstra(mygraph)

# Calculate distance to every node
start_node = nodes[0]
d.calc_new_graph(start_node)

# Save calculated graph with pickle
#with open('./dumped_graphs/dijkstra_graph_strt_node_%s.pickle' % (start_node.get_name()), 'wb') as w:
#    pickle.dump(d, w)

#mygraph.print_graph()
#d.graph.print_graph()

destination = nodes[-1]
route = d.get_shortest_route_to(destination)

# Create image with colorized route
img_route = colorize_route(img, route)
img_route.show()




