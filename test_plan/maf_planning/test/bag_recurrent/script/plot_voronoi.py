import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import numpy as np

if __name__ == "__main__":
    file_path = "f_voroni_face.log"
    coordinates_list = []
    coordinates_list_2 = []
    with open(file_path, 'r') as file:
        lines = file.readlines()
        for line in lines:
            coordinates = list(map(float, line.strip().split()))
            coordinates_list.append(coordinates)
    with open("f_voroni_center.log", 'r') as file:
        lines = file.readlines()
        for line in lines:
            coordinates = list(map(float, line.strip().split()))
            coordinates_list_2.append(coordinates)
    

    min_x = 0
    min_y = 0
    max_x = 0
    max_y = 0
    fig, ax = plt.subplots()
    for coordinates in coordinates_list:
        coordinates_ = []
        length = int(len(coordinates) / 2)

        for i in range(length):
            coordinate =  (coordinates[i * 2], coordinates[2 * i + 1])
            if(min_x > coordinates[i * 2]):
                min_x = coordinates[i * 2]
            if(max_x < coordinates[i * 2]):
                max_x = coordinates[i * 2]
            if(min_y > coordinates[2 * i + 1]):
                min_y = coordinates[2 * i + 1]
            if(max_y < coordinates[2 * i + 1]):
                max_y = coordinates[2 * i + 1]
            # coordinates = np.array(coordinates).reshape(-1, 2)
            coordinates_.append(coordinate)
        print(coordinates_)
        print("=========================")
        polygon = Polygon(coordinates_, edgecolor='c', facecolor='none')
        ax.add_patch(polygon)
    
    for coordinates in coordinates_list_2:
        point = plt.Circle((coordinates[0], coordinates[1]), 0.5, color = 'r')
        ax.add_patch(point)

    ax.set_aspect('equal', adjustable='box')
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    # Get the overall range of coordinates
    # Set plot limits based on the range of coordinates
    ax.set_xlim(min_x - 1, max_x + 1)
    ax.set_ylim(min_y - 1, max_y + 1)
    plt.title('Voronoi Polygons')

    plt.show()
