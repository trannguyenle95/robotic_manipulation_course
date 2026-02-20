#imports
import numpy as np
import matplotlib.pyplot as plt

#Load vertices
vertices = np.loadtxt("data/vertices.txt")

#global variables
mu = 1
alpha_mu = np.arctan(mu) #friction angle

def rotate_vector(v, angle):
    R = np.array([[np.cos(angle), -np.sin(angle)],[np.sin(angle), np.cos(angle)]])
    return np.dot(R,v)

def get_edges(vertices):
    edges = []
    #stack edges in 2x2 np arrays so edge[:,0] are the x-coordinates and edge[:,1] are the y-coordinates
    for i in range(len(vertices)):
        if i != len(vertices)-1: #last vertex is handled differently
            edges.append(np.vstack((vertices[i],vertices[i+1])))
        else:
            edges.append(np.vstack((vertices[i],vertices[0])))
    return edges

def plot_polygon(vertices):
    v_list = vertices.tolist()
    v_list.append(v_list[0])
    vertices = np.array(v_list)
    plt.fill(vertices[:,0], vertices[:,1], alpha = 0.4, label="polygon")

def get_curve_orientation(vertices):
    #Curve Orientation calculated with method described here https://en.wikipedia.org/wiki/Curve_orientation

    #Get point on convex hull - it must be in a corner so search for point with smallest x value (and smallest y 
    # value among these if there are multiple points with same smallest x value).
    #Call point on convex hull "B", so that the corresponding corner is "ABC" like in the link above.
    x_min = vertices[0,0]
    y_min = vertices[0,1]
    B_i = 0 #initialize index for point "B" on convex hull
    for i, point in enumerate(vertices):
        if point[0] < x_min:
            x_min = point[0]
            y_min = point[1]
            B_i = i
        elif point[0] == x_min:
            if point[1] < y_min:
                y_min = point[1]
                B_i = i

    #Get the points forming a corner "ABC" with "B" on the convex hull
    B = vertices[B_i] #point on convex hull
    #get previous point, call it A
    A = vertices[B_i-1]
    #get next point, call it C
    if B_i != len(vertices)-1:
        C = vertices[B_i+1]
    else: #loop back to first vertex if "B" was the last one
        C = vertices[0]
    
    #Plot points to visualize how orientation is determied
    plt.plot(A[0], A[1], marker="*", color="darkorange", label="A")
    plt.plot(B[0], B[1], marker="*", color="deeppink", label="B")
    plt.plot(C[0], C[1], marker="*", color="blue", label="C")

    #Calculate determinant using points "A". "B" and "C" to determine curve orientation like described in link above
    det = (B[0]-A[0])*(C[1]-A[1])-(C[0]-A[0])*(B[1]-A[1])
    if det > 0:
        return "ccw"
    else:
        return "cw"
    
def sample_point(edges):
    edge = edges[np.random.randint(0,len(edges))]
    x1 = edge[0,0]
    x2 = edge[1,0]
    y1 = edge[0,1]
    y2 = edge[1,1]

    if x1 == x2: #vertical line
        x = x1
        if y1 < y2: 
            y = np.random.uniform(y1, y2)
        else:
            y = np.random.uniform(y2, y1)
    else:
        if x1 < x2: 
            x = np.random.uniform(x1, x2)
        else:
            x = np.random.uniform(x2, x1)
        k = (y2-y1)/(x2-x1)
        y = k*(x-x1)+y1

    point = np.array([x,y])
    return edge, point

def get_values(edge, point, orientation):
    # This method generates points defining the legs of the friction cone and vectors
    # pointing inwards and outwards from the polygon at the contact point.
    # Values "p_FC_leg1", "p_FC_leg2" and "v_out" are used for plotting, the friction cone and the approach
    # direction. Value "v_in" is used for calculating the angle between vector "v_in",
    # which is at the center of the friction cone, and the line connecting the contact points.
    # This is used for calculating the performance metric in method "evaluate_grasp".
    # This method (get_values) also calculates the approach angle and returns it as "alpha_approach".

    x1 = edge[0,0]
    x2 = edge[1,0]
    y1 = edge[0,1]
    y2 = edge[1,1]

    dx = x2-x1
    dy = y2-y1

    #Get normal vector pointing to the interior of the object.
    #Depending on if edges are defined in clockwise or counterclockwise orientation the vector pointing to the
    #interor of the polygon will be either on the left side or right side of the edge vector.
    if orientation == "ccw":
        v_in = np.array([-dy,dx])
        v_in = (v_in/np.linalg.norm(v_in))*0.25 #set length to 0.25 for easier plotting
        v_out = np.array([dy,-dx])
        v_out = (v_out/np.linalg.norm(v_out))*0.25 #set length to 0.25 for easier plotting
    else:
        v_out = np.array([-dy,dx])
        v_out = (v_out/np.linalg.norm(v_out))*0.25 #set length to 0.25 for easier plotting
        v_in = np.array([dy,-dx])
        v_in = (v_in/np.linalg.norm(v_in))*0.25 #set length to 0.25 for easier plotting

    p_v_out = point + v_out  #vector from origin to endpoint of translated vector

    v_approach = point - p_v_out #reverse so it points from outside towards contact point, i.e. approach direction

    #get approach angle
    alpha_approach = np.rad2deg(np.arctan2(v_approach[1],v_approach[0]))

    #Need to rotate when it is in origin and only later translate to point
    FC_leg1 = rotate_vector(v_in, alpha_mu)
    FC_leg2 = rotate_vector(v_in, -alpha_mu)
    p_FC_leg1 = FC_leg1 + point #vector from origin to endpoint of translated vector
    p_FC_leg2 = FC_leg2 + point #vector from origin to endpoint of translated vector

    return p_FC_leg1, p_FC_leg2, v_in, v_out, alpha_approach

def plot_point(edge, point, order, orientation):
    if order == 1:
        color = "r"
        label_p = "grasping point 1, with approach direction and friction cone"
    else:
        color = "g"
        label_p = "grasping point 2, with approach direction and friction cone"

    plt.plot(point[0],point[1], "*"+color, label=label_p) #plot contact point
    p_FC_leg1, p_FC_leg2, v_in, v_out, alpha = get_values(edge,point, orientation)
    p_v_out = point + v_out  #vector from origin to endpoint of translated vector
    plt.plot(p_v_out[0], p_v_out[1], color) #plot point at start of arrow so figure scales to include start of arrows
    plt.arrow(p_v_out[0], p_v_out[1], point[0]-p_v_out[0], point[1]-p_v_out[1], length_includes_head = True, head_width = 0.05, color=color)
    plt.plot( (point[0],p_FC_leg1[0]), (point[1], p_FC_leg1[1]), color)
    plt.plot( (point[0],p_FC_leg2[0]), (point[1], p_FC_leg2[1]), color)

    
def evaluate_grasp(p1, p1_v_in, p2, p2_v_in):
    #If angle angle from middle of friction cone to line connecting contact points is larger than the angle 
    # of the friction cone "alpha_mu", then the line goes outside the cone, and the grasp is not stable.

    #We know angle inside friction cone is alpha_mu = 45 degrees thus, we figure out if the angle between 
    #the inwards pointing normal (middle of friction cone) and line connectig contact points is larger than 45 degrees 
    #for either of the contact points. If this is the case the grasp is unstable
    #and we reject these points an continue samplig points.

    #angle between middle of friction cone and line connecting points
    v_p1_p2 = p2-p1 #vector between the points
    angle_p1 = np.arccos(np.dot(p1_v_in, v_p1_p2)/(np.linalg.norm(p1_v_in)*np.linalg.norm(v_p1_p2)))

    #angle between middle of friction cone and line connecting points
    v_p2_p1 = p1-p2 #vector between the points
    angle_p2 = np.arccos(np.dot(p2_v_in, v_p2_p1)/(np.linalg.norm(p2_v_in)*np.linalg.norm(v_p2_p1)))

    #The metric compares how large the larger angle between friction cone middle and line connecting points is, compared
    #to the friction angle "alpha_mu". This is because a smaller angle is desired so that the grasping points are further from being unstable, 
    #that is, connecting line going outside either friction cone. The values for stable grasps are then scaled between 0-1, corresponding to how close the
    #larger angle is to the ideal angle "zero degrees". If the larger angle is greater than alpha_mu, then the metric will be negative, indicating that the
    #grasp is unstable and should be rejected.

    #use np.max because we want the worse angle to be as good as possible
    metric = (alpha_mu - np.max((angle_p1, angle_p2))) / (alpha_mu) #negative value means unstable grasp, stable grasps have value 0-1 with 1 being most robust
    return metric
    
def point_to_str(point, alpha):
    return "("+str(round(point[0],2))+","+str(round(point[1],2))+","+str(int(round(alpha)))+" deg"+")"


def main():
    orientation = get_curve_orientation(vertices)
    print("Curve ortientation: ", orientation) #curve orientation can also be seen from order of points "A", "B" and "C" in the generated plot

    edges = get_edges(vertices)
    plot_polygon(vertices)

    best_grasp_value = -1
    max_iter = 1000 #with more iterations even better grasps could be found
    iterations = 0
    #robustness metric from evaluate_grasp reutrns negative values for non-stable grasps, and a value between 0-1 for stable grasps with 1 being most stable
    while(best_grasp_value < 0 or iterations < max_iter):
        e1,p1 = sample_point(edges)
        p1_FC1, p1_FC2, p1_v_in, p1_v_out, p1_alpha = get_values(e1, p1, orientation)

        e2 = e1 #initialize as same edge
        while(np.array_equal(e1,e2)):
            e2,p2 = sample_point(edges) #sample until point on different edge is found
        p2_FC1, p2_FC2, p2_v_in, p2_v_out, p2_alpha = get_values(e2, p2, orientation)

        grasp_value = evaluate_grasp(p1,p1_v_in,p2,p2_v_in)
        if grasp_value > best_grasp_value:
            best_grasp_value = grasp_value
            best_p1 = p1
            best_p2 = p2
            best_e1 = e1
            best_e2 = e2
            best_p1_alpha = p1_alpha
            best_p2_alpha = p2_alpha

        iterations += 1

    #Now that the points with highest stability metric score in 1000 iterations have been found, do plotting
    plot_point(best_e1,best_p1,1, orientation)
    print("point 1 alpha (degrees):", best_p1_alpha)
    plot_point(best_e2,best_p2,2, orientation)
    print("point 2 alpha (degrees):", best_p2_alpha)
    plt.plot((best_p1[0],best_p2[0]),(best_p1[1],best_p2[1])) #plot connecting line

    #write to file
    f = open("data/grasp_data.txt", "w")
    #The robots in the simulation are spaced along the x-axis, so assign the grasp point with higher x-value first and then the grasp
    #point with lower x-value, to maximize chances of robots reaching the grasp points. There are still planned grasps that are valid as they are
    #stabile, but that the ROS simulation cannot execute as the robots do not reach so far. For example, when both grasp points are placed close 
    #to the middle between the robots and they are expected to grasp the object from the positive and negative direction of the y-axis,
    #they cannot reach these positions).
    if(best_p1[0] > best_p2[0]): #write higher x-value grasping point first, see explanation above
        f.write(str(best_p1[0]*0.1)+" "+str(best_p1[1]*0.1)+" "+str(best_p1_alpha)+"\n")
        f.write(str(best_p2[0]*0.1)+" "+str(best_p2[1]*0.1)+" "+str(best_p2_alpha))
    else:
        f.write(str(best_p2[0]*0.1)+" "+str(best_p2[1]*0.1)+" "+str(best_p2_alpha)+"\n")
        f.write(str(best_p1[0]*0.1)+" "+str(best_p1[1]*0.1)+" "+str(best_p1_alpha))
    f.close()

    #display plot
    plt.axis("equal") #make scaling for x and y axes same
    plt.legend(loc="upper left")
    plt.title("Grasping points are: "+point_to_str(best_p1,best_p1_alpha)+" , and "+point_to_str(best_p2,best_p2_alpha)+". Quality metric: "+str(round(best_grasp_value,2))+".")
    plt.show()

if __name__ == "__main__":
    main()