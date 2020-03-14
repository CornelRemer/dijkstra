"""Convert a PIL image into a graph, suitable for the dijkstra algorithm
"""

import numpy as np
import math

from dijkstra import Node

class RasterNodes():
    """Graph of nodes created from an image like a Digital Terrain Model (DTM)

    The __init__ takes two arguments:
        image (obj): Image object from the PIL module
        pixel_size (int): Complies the size of on pixel (in meter)
    
    Example:
        graph = RasterNodes(img, PIXEL_SIZE)

    """
    def __init__(self, image, pixel_size = 5):
        self._width, self._height = image.size
        self._image = image
        self._pixel_size = pixel_size

        # create a None-value frame
        self._matrix = np.full(
            (self._width + 2, self._height + 2),
            None)
        # fill pixel values inside the None-value frame
        self._fill_matrix()

        self._straight_neighbors = [
            (-1,0),
            (1,0),
            (0,-1),
            (0,1)]

        self._diagonal_neighbors =[
            (-1,-1),
            (-1,1),
            (1,-1),
            (1,1)]

    
    def _fill_matrix(self):
        self._pixel_values = self._image.load()
        for i in range(self._width):
            for j in range(self._height):
                self._matrix[i+1, j+1] = self._pixel_values[i, j]
    
    def _determine_neighbors(self, neighbor_coordinates):
        for i,j in neighbor_coordinates:
            self._x, self._y = self._coloumn+i, self._row+j

            if self._matrix[self._x, self._y] is not None:
                self._pixel_value = self._matrix[self._x, self._y]

                # get direct neighbors
                if neighbor_coordinates == self._straight_neighbors:
                    self.distance = math.sqrt(self._pixel_size**2 + (self._matrix[self._row, self._coloumn] - self._pixel_value)**2)

                # get diagonal neighbors
                elif neighbor_coordinates == self._diagonal_neighbors:
                    self.distance = math.sqrt((math.sqrt(2 * self._pixel_size**2))**2 + (self._matrix[self._row, self._coloumn] - self._pixel_value)**2)

                self._neighbors.append(
                    ('%s,%s' % (self._x - 1, self._y - 1), self.distance))
    
    def get_pixel_neighbors(self, pixel_coordinates):
        """Returns all neighbors of a specific pixel
        
        Args:
            pixel_coordinates (tuple): coordinates (integer) of a specific pixel in (row, coloumn)
        """
        self._coloumn, self._row = [_ + 1 for _ in pixel_coordinates]
        self._neighbors = []
        self._determine_neighbors(self._diagonal_neighbors)
        self._determine_neighbors(self._straight_neighbors)
        return self._neighbors
    
    def transform_to_nodes(self):
        """Converts every pixel from the image into a Node.

        Returns all Node-objects in a list
        """
        self.nodes = []
        for i in range(self._width):
            for j in range(self._height):
                self.nodes.append(Node(
                    '%s,%s' % (i,j),
                    self.get_pixel_neighbors((i,j))
                ))
        return self.nodes

if __name__=="__main__":
    from PIL import Image

    WIDTH = 3
    HEIGHT = 3

    PIXEL_VALUE = 15

    PIXEL_SIZE = 5

    img = Image.new('L', (WIDTH, HEIGHT), PIXEL_VALUE)

    myraster_nodes = RasterNodes(img, PIXEL_SIZE)
    nodes = myraster_nodes.transform_to_nodes()

    print(nodes)