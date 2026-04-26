import numpy as np
import matplotlib.pyplot as plt
import math
from shapely.geometry import Point, Polygon

SAMPLING_NUM = 5
# def centroid(vertexes):
#      _x_list = [vertex [0] for vertex in vertexes]
#      _y_list = [vertex [1] for vertex in vertexes]
#      _len = len(vertexes)
#      _x = sum(_x_list) / _len
#      _y = sum(_y_list) / _len
#      return(_x, _y)

def allPairs(arr1,arr2,x):
    print ([(x-k,k) for k in arr2 if (x-k) in arr1]) 

def check_is_online(point1, point2, poly_c, index):
    # num = (point2[1]-point1[1])
    # den = (point2[0]-point1[0])
    # slope = num / den
    
    # if cross == 0:
    #     print(index)
    #     return index

    dxc = int(poly_c.centroid.x - point1[0])
    dyc = int(poly_c.centroid.y - point1[1])

    dxl = int(point2[0] - point1[0])
    dyl = int(point2[1]- point1[1])

    cross = dxc * dyl - dyc * dxl

    if cross == 0:
        print(index)
        return index

def cal_slope(point1, point2):
    num = (point2[1]-point1[1])
    den = (point2[0]-point1[0])
    slope = num / den
    return slope

# def cal_distance(point1, point2):
#     """point1, 2 is np.array 1x2 a
#        return the distance scalar"""
#     return np.linalg.norm(point1.reshape((2, 1)) - point2.reshape((2, 1)))

# def cal_angle(point1, point2):
#     """point1, 2 is np.array 1x2 a
#        return the angle in rad"""
#     lx, ly = np.linalg.norm(point1.reshape((2, 1))), np.linalg.norm(point2.reshape((2, 1)))
#     cos_angle = point1.dot(point2) / (lx * ly)
#     angle = np.arccos(cos_angle)
#     if angle > np.pi / 2:
#         return np.pi - angle
#     else:
#         return angle

def sample_polygon_edge(p1, p2, numofpoints):
    return list(zip(np.linspace(p1[0], p2[0], numofpoints),
               np.linspace(p1[1], p2[1], numofpoints)))

def draw_perpendicular(point, angle, length, polygon):
    x, y = point
    plt.plot(x, y, 'g*')

    endx_cone = x + length * math.cos(math.radians(angle))
    endy_cone = y + length * math.sin(math.radians(angle))
    _endx_cone = x + length * -math.cos(math.radians(angle))
    _endy_cone = y + length * -math.sin(math.radians(angle))
   
    # ROI
    roi_x = x + 0.01 * math.cos(math.radians(angle))
    roi_y = y + 0.01 * math.sin(math.radians(angle))
    _point = Point(roi_x, roi_y)

    if polygon.contains(_point):
        plt.plot([x, endx_cone], [y, endy_cone], 'g--')
    else:
        plt.plot([x, _endx_cone], [y, _endy_cone], 'g--')

def draw_friction_cone(point, angle, length, polygon, index):
    x, y = point

    endx_cone = x + length * math.cos(math.radians(angle))
    endy_cone = y + length * math.sin(math.radians(angle))
    _endx_cone = x + length * -math.cos(math.radians(angle))
    _endy_cone = y + length * -math.sin(math.radians(angle))
    
    # ROI
    roi_x = x + 0.01 * math.cos(math.radians(angle))
    roi_y = y + 0.01 * math.sin(math.radians(angle))
    _point = Point(roi_x, roi_y)

    if polygon.contains(_point):
        plt.plot([x, endx_cone], [y, endy_cone], 'b-')
        return index, float(angle)
    else:
        plt.plot([x, _endx_cone], [y, _endy_cone], 'b-')
        return index, float(angle + 180)

def main():
    plt.subplots(figsize=(5,5))

    geometry_data = np.loadtxt('data/vertices.txt')
    x = geometry_data[:, 0]
    y = geometry_data[:, 1]

    polygon_data = []
    for coordinates in geometry_data:
        polygon_data.append(list(coordinates))
    polygon_data.append(list(geometry_data[0]))
    # print(polygon_data)
    
    # Sampling on each edge
    polygon_data_sample = []
    # convert a list into an array
    polygon_data_arr = np.asarray(polygon_data)
    for index in range(len(polygon_data_arr)-1):
        polygon_data_sample.extend(sample_polygon_edge(polygon_data_arr[index], polygon_data_arr[index+1], SAMPLING_NUM))
    # print(polygon_data_sample)
    
    # Create Polygon
    poly_sample = Polygon(polygon_data_sample)
    
    # Start Point
    start_x, start_y = polygon_data_sample[0]
    plt.plot(start_x, start_y, 'g*')

    grasp_position_x = []
    grasp_position_y = []
    grasp_angle = []
    # friction_coefficient = 1 -> friction cone = 45 deg
    # cone_rad = np.arctan(1)
    # cone_deg = cone_rad * 180/np.pi
    for index in range(len(polygon_data_sample)-1):
        # slope = np.cross(polygon_data_sample[index], polygon_data_sample[index+1])
        slope = cal_slope(polygon_data_sample[index], polygon_data_sample[index+1])
        # stable_index = check_is_online(polygon_data_sample[index], polygon_data_sample[index+1], poly_sample, index)
        # if math.isnan(slope) != True and str(slope) != 'inf':
        # if (point_oneline(1)):
        #     print("ha")

        if index % SAMPLING_NUM != 0 and math.isnan(slope) != True:
            grasp_index, angle = draw_friction_cone(polygon_data_sample[index], np.arctan(-1/slope)*180/np.pi + 45, 0.5, poly_sample, index)        
            draw_friction_cone(polygon_data_sample[index], np.arctan(-1/slope)*180/np.pi - 45, 0.5, poly_sample, index)
            draw_perpendicular(polygon_data_sample[index], np.arctan(-1/slope)*180/np.pi, 1, poly_sample)

            grasp_position_x.append(polygon_data_sample[grasp_index][0]/10.0)
            grasp_position_y.append(polygon_data_sample[grasp_index][1]/10.0)
            grasp_angle.append(angle)

    with open('data/test.txt', 'w') as f:
        for f1, f2, f3 in zip(grasp_position_x, grasp_position_y, grasp_angle):
            print(format(f1, '.6f'), format(f2, '.6f'), int(f3), file=f)

    # geometry_data visualization
    x_p, y_p = poly_sample.exterior.xy
    plt.plot(x_p, y_p, 'black')
    plt.plot(poly_sample.centroid.x, poly_sample.centroid.y, 'bo')
    # plt.fill(x, y)

    # grasp_data visualization
    grasp_data = np.loadtxt('data/grasp_data.txt')
    scale = 10
    draw_length = 0.5
    for contacts in grasp_data:
        contact_x, contact_y, angle = contacts
        plt.plot(contact_x*scale, contact_y*scale, 'ro')
   
    # Show
    plt.show()


if __name__ == "__main__":
    main()