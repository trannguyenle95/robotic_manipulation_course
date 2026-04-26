import os
from shapely.geometry import Polygon, Point, LineString
from shapely.affinity import rotate

import numpy as np

import math
import matplotlib.pyplot as plt

from random import randrange

dirpath = os.getcwd()
print("current directory is : " + dirpath)
foldername = os.path.basename(dirpath)
print("Directory name is : " + foldername)
assert foldername == "exercise5", "You must be in the exercise5 directory to run this script."

def read_vertices(filename="data/vertices.txt"):
    with open(filename) as f:
        lines = f.readlines()
    vertices = [] # list with all vertices as tuples of (x, y)
    for line in lines:
        x, y = line.split()
        vertices.append((float(x), float(y)))
    return vertices

def calculate_normal(vertex, grasp_point, polygon, length):
    ab = LineString([vertex, grasp_point])
    left = ab.parallel_offset(0.1, 'left')
    point_left = left.boundary[1]
    # right = ab.parallel_offset(0.1, 'right')
    # point_right = right.boundary[0]
    correct_side = 'left' if polygon.contains(point_left) else 'right'

    internal_parallel = ab.parallel_offset(length, correct_side)
    return internal_parallel.boundary[1] if correct_side == 'left' else internal_parallel.boundary[0]

def contruct_friction_cone(vertex, grasp_point, polygon, alpha=45, length=2):
    normal = calculate_normal(vertex, grasp_point, polygon, length=length)
    friction_cone = []
    friction_cone.append(rotate(normal, angle=alpha, origin=grasp_point))
    friction_cone.append(rotate(normal, angle=-alpha, origin=grasp_point))
    return friction_cone, normal

def get_optimal_pair_of_grasps(stable_grasp_points_id_pairs, grasp_points_dict, polygon):
    opt_pair = None
    min_distance = 1000000
    for pair in stable_grasp_points_id_pairs:
        id_a, id_b = pair
        total_dist_to_cent = polygon.centroid.distance(grasp_points_dict[id_a]['point']) +\
            polygon.centroid.distance(grasp_points_dict[id_b]['point'])
        if total_dist_to_cent < min_distance:
            opt_pair = pair
            min_distance = total_dist_to_cent
    return opt_pair

def get_approach_angles(grasp_pair, grasp_points_dict):
    approach_angles_dict = dict()
    for id in grasp_pair:
        grasp_point = grasp_points_dict[id]['point']
        normal = grasp_points_dict[id]['normal']
        dx = normal.x - grasp_point.x
        dy = normal.y - grasp_point.y
        external_perp = Point(grasp_point.x - dx, grasp_point.y - dy)
        angle = math.degrees(math.atan2(dy, dx)) % 360
        approach_angles_dict[id] = {
            'angle': angle,
            'external_perp': external_perp,
        }
    return approach_angles_dict

def linspace_points(p1, p2, num=30):
    return list(zip(np.linspace(p1.x, p2.x, num + 1), np.linspace(p1.y, p2.y, num + 1)))

def plot_polygon(polygon, format=None):
    x_coors, y_coors = polygon.exterior.xy
    plt.plot(x_coors, y_coors)

def plot_point(point, format='r*'):
    plt.plot(point.x, point.y, format)

def plot_line(points, format='y-'):
    x_coors = [p.x for p in points]
    y_coors = [p.y for p in points]
    plt.plot(x_coors, y_coors, format)

def plot_friction_cone(grasp_point, friction_cone, format='b'):
    plt.plot([grasp_point.x, friction_cone[0].x], [grasp_point.y, friction_cone[0].y], format)
    plt.plot([grasp_point.x, friction_cone[1].x], [grasp_point.y, friction_cone[1].y], format)

def write_output(grasp_pair_dict, approach_angles_dict, filename="data/grasp_data.txt"):
    try:
        os.remove(filename)
    except OSError:
        pass

    with open(filename, 'w') as f:
        for id in grasp_pair_dict:
            point = grasp_pair_dict[id]['point']
            angle = int(approach_angles_dict[id]['angle'])
            f.write("{0:f}\t{1:f}\t{2:d}\n".format(round(point.x * 0.1, 5), round(point.y * 0.1, 5), round(angle)))


def plan_grasp():
    vertices = read_vertices()
    polygon = Polygon(vertices)
    num_vertices = len(vertices)

    print("Step 1: Constructing friction cone at each grasp point.")
    grasp_points_dict = dict()
    # Iterate over each pair of correlative vertices
    for i in range(num_vertices):
        vertex_a = Point(vertices[i])
        vertex_b = Point(vertices[(i+1) % num_vertices])

        # Get evenly spaced points over the line they produce
        points = linspace_points(vertex_a, vertex_b)
        points = [Point(p) for p in points]

        # Contruct friction cone at every one of these possible grasp points
        for j, point in enumerate(points):
            if point == vertex_a:
                continue
            friction_cone, _ = contruct_friction_cone(vertex_a, point, polygon, alpha=45, length=polygon.length*2)
            friction_cone_to_plot, normal = contruct_friction_cone(vertex_a, point, polygon, alpha=45, length=1)
            cone_span = Polygon(friction_cone + [point])
            id = str(i) + '_' + str(j)
            grasp_points_dict[id] = {
                'point': point, 
                'cone_span': cone_span, 
                'friction_cone': friction_cone_to_plot,
                'normal': normal,
            }

    print("Step 2: Check all other grasping points that lie within the friction cone.")
    # Check all other grasping points that lie within the cone (a triangle)
    point_ids = grasp_points_dict.keys()
    all_points = [grasp_points_dict[id]['point'] for id in point_ids]
    for id in point_ids:
        # print("Computing id", id, "out of", len(point_ids))
        cone_span = grasp_points_dict[id]['cone_span']
        potential_pair = [p for p in all_points if cone_span.contains(p)]
        grasp_points_dict[id]['potential_pair'] = potential_pair
        # for p in compatible_points:
        #     plot_point(p)
    
    print("Step 3: Get all stable pairs of grasp points.")
    stable_grasp_points_id_pairs = []
    for i, id1 in enumerate(point_ids):
        print("Computing id number", i, "out of", len(point_ids))
        for id2 in point_ids:
            if grasp_points_dict[id1]['point'] in grasp_points_dict[id2]['potential_pair'] \
                and grasp_points_dict[id2]['point'] in grasp_points_dict[id1]['potential_pair']:
                stable_grasp_points_id_pairs.append((id1, id2))

    print("Step 4: Select optimal pair of grasp points.")
    grasp_pair = get_optimal_pair_of_grasps(stable_grasp_points_id_pairs, grasp_points_dict, polygon)

    print("Step 5: Compute approach angle for those points.")
    approach_angles_dict = get_approach_angles(grasp_pair, grasp_points_dict)

    print("Step 6: Plot all.")
    plot_polygon(polygon)

    id_a, id_b = grasp_pair
    grasp_point_a = grasp_points_dict[id_a]['point']
    grasp_point_b = grasp_points_dict[id_b]['point']

    # Plot first grasp point, its friction cone and approach angle
    plot_point(grasp_point_a, format='g*')
    plot_friction_cone(grasp_point_a, grasp_points_dict[id_a]['friction_cone'], format='g')
    plot_line([grasp_point_a, approach_angles_dict[id_a]['external_perp']], format='g-.')

    # Plot second grasp point, its friction cone and approach angle
    plot_point(grasp_point_b, format='r*')
    plot_friction_cone(grasp_point_b, grasp_points_dict[id_b]['friction_cone'], format='r')
    plot_line([grasp_point_b, approach_angles_dict[id_b]['external_perp']], format='r-.')

    # Plot line connecting both grasp points
    plot_line([grasp_point_a, grasp_point_b], format='y--')

    plt.savefig('data/grasp_data.png')
    # plt.show()

    print("Step 7: Write solution in a file.")
    grasp_pair_dict = dict((id, grasp_points_dict[id]) for id in grasp_pair)
    write_output(grasp_pair_dict, approach_angles_dict)

if __name__ == "__main__":
    plan_grasp()