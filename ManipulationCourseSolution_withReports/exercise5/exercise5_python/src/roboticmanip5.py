import matplotlib.pyplot as plt
from shapely.geometry import Polygon, Point
import numpy as np
import math, re, sys, argparse, warnings
from scipy.spatial import distance

parser = argparse.ArgumentParser( )
parser.add_argument('--input' , help = 'Input path to file containing polygon vertices' )
parser.add_argument('--output' , help = 'Output path to file for grasping points and angles' )
args = parser.parse_args( )

read_file = args.input # "data_vertices.txt"
write_file = args.output # "grasp_data.txt"


def read_polygon_from_file ( file_path , verbose = False ) :
    '''
    Read and return coordinates from specified file <file_path>
    '''
    coord = [ ]
    with open ( file_path ) as f :
        for line in f :
            line = re.sub ( r'\s' , ',' , line )
            x = float ( line.split ( ',' ) [ 0 ] )
            y = float ( line.split ( ',' ) [ 1 ] )
            coord.append ( [ x , y ] )

    coord.append ( coord [ 0 ] )
    xs , ys = zip ( *coord )
    if verbose :
        print ( coord )
    return xs , ys , coord


def write_to_file ( min_point1 , min_point2 , angle1 , angle2 ) :
    '''
    Write the grasping positions (x,y) and their angles wrt x axis into file
    For mujoco, 0.1 scaling is applied before.
    '''
    angle1 += 180
    while angle1 >= 360 :
        angle1 -= 360
    angle2 += 180
    while angle2 >= 360 :
        angle2 -= 360
    str1 = " ".join ( str ( x ) for x in [ min_point1 [ 0 ] * 0.1 , min_point1 [ 1 ] * 0.1 , int ( angle1 ) ] )
    str2 = "\n"
    str3 = " ".join ( str ( x ) for x in [ min_point2 [ 0 ] * 0.1 , min_point2 [ 1 ] * 0.1 , int ( angle2 ) ] )
    file1 = open ( write_file , "w+" )
    file1.write ( str1 )
    file1.write ( str2 )
    file1.write ( str3 )
    file1.close ( )


def plot_polygon_with_center ( x , y , center ) :
    '''
    Plot polygon and the center of the polygon
    '''
    plt.figure ( )
    plt.plot ( x , y )
    plt.scatter ( center [ 0 ] , center [ 1 ] )


def centroid ( vertexes , verbose = False ) :
    '''
    Return the center of a polygon specified by the points in <vertexes>
    '''
    centroid = list ( Polygon ( vertexes ).centroid.coords )
    polygon = Polygon ( vertexes )
    length = Polygon ( vertexes ).length
    if verbose :
        print ( centroid )
    _centroid = [ centroid [ 0 ] [ 0 ] , centroid [ 0 ] [ 1 ] ]
    return (_centroid , polygon , length)


def getPointsInFrictionCone ( start_point , point_fc1 , point_fc2 , coordinates ) :
    '''
    Inputs are 3 points: grasping point and 2 points that are on the lines
    of the friction cones. They make up a triangle.
    Return a dictionary, where the key is the grasping point and
    the values are the points on the polygon's vertex, that are inside
    the grasping point's friction cone
    '''
    inside_points = [ ]
    triangle = Polygon ( [ start_point , point_fc1 , point_fc2 ] )
    for point in range ( 0 , len ( coordinates ) ) :
        _point = Point ( coordinates [ point ] )
        if triangle.contains ( _point ) :
            inside_points.append ( coordinates [ point ] )
    point_pairs = { start_point : inside_points }
    return point_pairs


def getEquidistantPoints ( p1 , p2 , number_of_points ) :
    '''
    Return a list of evenly spaced numbers <number_of_points> over a specified interval <p1, p2>.
    '''
    return list ( zip ( np.linspace ( p1 [ 0 ] , p2 [ 0 ] , number_of_points + 1 ) ,
                        np.linspace ( p1 [ 1 ] , p2 [ 1 ] , number_of_points + 1 ) ) )


def get_friction_cone_point ( point , angle , length , polygon , plot = False ) :
    '''
    point - grasping point (x,y)
    angle - Angle of friction cone you want your end point at in degrees.
    length - Length of the friction cone
    Returns a point on the line of the friction cone, which is <length> away from <point>
    '''
    x , y = point
    endx = x + 0.01 * math.cos ( math.radians ( angle ) )
    endy = y + 0.01 * math.sin ( math.radians ( angle ) )
    endx_triangle = x + length * math.cos ( math.radians ( angle ) )
    endy_triangle = y + length * math.sin ( math.radians ( angle ) )
    endx_alter_triangle = x + length * math.cos ( math.radians ( angle + 180 ) )
    endy_alter_triangle = y + length * math.sin ( math.radians ( angle + 180 ) )
    _point = Point ( endx , endy )
    if polygon.contains ( _point ) :
        if plot :
            plt.plot ( [ x , endx_triangle ] , [ y , endy_triangle ] )
        return [ endx_triangle , endy_triangle ]
    else :
        if plot :
            plt.plot ( [ x , endx_alter_triangle ] , [ y , endy_alter_triangle ] )
        return [ endx_alter_triangle , endy_alter_triangle ]


def point_perpendicular ( point , angle , length , polygon , plot = False ) :
    '''
    point - grasping point (x,y)
    angle - Angle of friction cone you want your end point at in degrees.
    length - Length of the friction cone
    Plots a perpendicular approach line on the grasping point with <length> away from <point>
    '''
    x , y = point
    endx = x + 0.01 * math.cos ( math.radians ( angle ) )
    endy = y + 0.01 * math.sin ( math.radians ( angle ) )
    endx_triangle = x + length * math.cos ( math.radians ( angle ) )
    endy_triangle = y + length * math.sin ( math.radians ( angle ) )
    endx_alter_triangle = x + length * math.cos ( math.radians ( angle + 180 ) )
    endy_alter_triangle = y + length * math.sin ( math.radians ( angle + 180 ) )
    _point = Point ( endx , endy )
    if not polygon.contains ( _point ) :
        if plot :
            plt.plot ( [ x , endx_triangle ] , [ y , endy_triangle ] )
        return angle
    else :
        if plot :
            plt.plot ( [ x , endx_alter_triangle ] , [ y , endy_alter_triangle ] )
        return (angle + 180)


def get_points_in_friction_cone ( coordinates , length , polygon , all_coordinates_of_polygon ) :
    '''
    returns 2 dictionaries:
     1st - dictonary{point: corresponding perpendicular line's degree}
     2nd - dictionary {grasp_point:inside_points} to get the points inside the friction cone
    '''
    point_pairs = { }
    point_data = { }
    for point in range ( 0 , len ( coordinates ) - 1 ) :
        points_on_a_side = getEquidistantPoints ( coordinates [ point ] , coordinates [ point + 1 ] ,
                                                  50 )  # get points on the sides of the polygon
        for i in range ( 0 , len ( points_on_a_side ) - 1 ) :
            slope = (points_on_a_side [ i + 1 ] [ 1 ] - points_on_a_side [ i ] [ 1 ]) / (
                    points_on_a_side [ i + 1 ] [ 0 ] - points_on_a_side [ i ] [
                0 ])  # calculate the slope of the sides of the polygon
            perpend_slope = -1 / slope  # slope of the perpedicular line on the grasping point (on the side of the polygon)
            perpend_angle = math.degrees ( math.atan ( perpend_slope ) )
            point_data.update (
                { points_on_a_side [ i ] : perpend_angle
                  } )  # dictonary{point: corresponding perpendicular line's degree}
            fc1 = get_friction_cone_point ( points_on_a_side [ i ] , perpend_angle + 45 ,
                                            length , polygon , False )  # get a point on one of the friction cones
            fc2 = get_friction_cone_point ( points_on_a_side [ i ] , perpend_angle - 45 ,
                                            length , polygon , False )  # get a point on the other friction cone
            point_pairs.update ( getPointsInFrictionCone ( points_on_a_side [ i ] , fc1 , fc2 ,
                                                           all_coordinates_of_polygon ) )  # dictionary {grasp_point:inside_points} to get the points inside the friction cone
    return point_data, point_pairs


def get_sorted_coordinate_list ( coordinates ) :
    '''
    Return a 1D list with all the <coordinates> of the polygon's sides
    '''
    all_points = [ ]
    all_points_sorted = [ ]
    for point in range ( 0 , len ( coordinates ) - 1 ) :
        all_points.append ( getEquidistantPoints ( coordinates [ point ] , coordinates [ point + 1 ] , 50 ) )
    for point in range ( 0 , len ( all_points ) ) :
        for i in range ( 0 , len ( all_points [ point ] ) ) :
            all_points_sorted.append ( all_points [ point ] [ i ] )
    return all_points_sorted

def main( ) :
    warnings.filterwarnings ( "ignore" , message = "divide by zero encountered in double_scalars" )
    x , y , coordinates = read_polygon_from_file ( read_file , verbose = False )  # read coordinates from file
    center , polygon , length = centroid ( coordinates , verbose = False )  # calculate the center of polygon and the length of sum(len(sides))
    plot_polygon_with_center ( x , y , center )  # plot the polygon with its center
    all_coordinates_of_polygon = get_sorted_coordinate_list ( coordinates )  # get all the boundary coordinates of the polygon

    # dictionary point_pairs {grasp_point:inside_points} to get the points inside the friction cone
    point_data, point_pairs = get_points_in_friction_cone( coordinates , length, polygon , all_coordinates_of_polygon )

    # Get all the stable grasp points by crearting grasping point pairs
    grasp_points_list = [ ]  # store the stable grasping point pairs
    for key , value in point_pairs.items ( ) :
        for key2 , value2 in point_pairs.items ( ) :
            if key in value2 and key2 in value :
                grasp_points_list.append ( key )
                grasp_points_list.append ( key2 )

    # Get the distances between the stable grasping points and the center of the polygon
    grasp_points_distance = [ ]  # store the stable grasping point pairs with their distances from the center of the polygon
    for i in range ( 0 , len ( grasp_points_list ) - 1 ) :
        dist1 = distance.euclidean ( grasp_points_list [ i ] , center )
        dist2 = distance.euclidean ( grasp_points_list [ i + 1 ] , center )
        dist = dist1 + dist2
        grasp_points_distance.append ( dist )
        grasp_points_distance.append ( grasp_points_list [ i ] )
        grasp_points_distance.append ( grasp_points_list [ i + 1 ] )

    # Get the minimum distance between stable grasp points and the center of the polygon --> optimal grasp (force closure)
    # (if the polygon is symmetric, then the first optimal)
    min = float ( math.inf )
    for j in range ( 0 , len ( grasp_points_distance ) , 3 ) :
        if grasp_points_distance [ j ] < min :
            min = grasp_points_distance [ j ]
            min_point1 = grasp_points_distance [ j + 1 ]
            min_point2 = grasp_points_distance [ j + 2 ]

    # Visualize the calculated optimal grasping points
    plt.scatter ( min_point1 [ 0 ] , min_point1 [ 1 ] )
    plt.scatter ( min_point2 [ 0 ] , min_point2 [ 1 ] )

    # plot perpendicular line on grasping point with length = average polygon side's length
    perp_angle1 = point_perpendicular ( min_point1 , point_data [ min_point1 ] , length / len ( coordinates ) , polygon , True )
    perp_angle2 = point_perpendicular ( min_point2 , point_data [ min_point2 ] , length / len ( coordinates ) , polygon , True )
    get_friction_cone_point ( min_point1 , point_data [ min_point1 ] + 45 , length / len ( coordinates ) , polygon , True )
    get_friction_cone_point ( min_point1 , point_data [ min_point1 ] - 45 , length / len ( coordinates ) , polygon , True )
    get_friction_cone_point ( min_point2 , point_data [ min_point2 ] + 45 , length / len ( coordinates ) , polygon , True )
    get_friction_cone_point ( min_point2 , point_data [ min_point2 ] - 45 , length / len ( coordinates ) , polygon , True )

    # Write data to file
    write_to_file ( min_point1 , min_point2 , perp_angle1 , perp_angle2 )

    plt.show ( )

if __name__== "__main__":
    main()