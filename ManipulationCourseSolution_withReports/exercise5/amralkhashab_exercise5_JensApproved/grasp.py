import matplotlib.pyplot as plt
import random
import numpy as np
import math

#metric angle and distance

vertices = []
xp = []
yp = []
line = []
with open('data/vertices.txt') as f:
   for line in f:
       x, y = line.split()
       vertices.append((float(x),float(y)))
       xp.append(float(x))
       yp.append(float(y))

x_origin = sum(xp) / len(xp)
y_origin = sum(yp) / len(yp)
origin = (x_origin, y_origin)

#constrains
## no two contacts on the same side
## the other contact is set arbitrarily to any the other sides, and at any location
# select the first point to at the center

#vector class helps in drawing and finding alpha##########################################

class vector:
    #vector A, B
    def __init__(self, point1, point2):
        self.A = point1
        self.B = point2
        self.AB = [x1 - x2 for (x1, x2) in zip(self.B, self.A)]
    def magnitude(self):
        sum = math.pow(self.AB[0],2) + math.pow(self.AB[1],2)
        return math.sqrt(sum)

    def dot(self, vector2):
        if len(self.AB) != len(vector2.AB):
            return 0
        return sum(i[0] * i[1] for i in zip(self.AB, vector2.AB))

    def alpha(self, vector2):
        DOT = self.dot(vector2)
        angle = math.acos(DOT/(self.magnitude() * vector2.magnitude())) * (180/math.pi)
        if self.AB[1] < 0:
            return 360 - angle
        else:
            return angle

    def normalize(self):
        return [float(x1)/sum([abs(number) for number in self.AB]) for x1 in self.AB]

    def rotation(self, angle):
        AB = np.array(self.AB)
        R = np.array([[math.cos(angle), -math.sin(angle)],[math.sin(angle), math.cos(angle)]])
        rotated = np.dot(R, np.transpose(AB))
        return [float(x1)/sum([abs(number) for number in rotated]) for x1 in rotated]

    def vector_lies_between(self, vector1, vector2, ox):
        angle = self.alpha(ox)
        angle2 = vector1.alpha(ox)
        angle3 = vector2.alpha(ox)

        print("anglesoriginal", angle, angle2, angle3)

        if abs(angle3 - angle2) > 100: #margine for errors
            if angle2 > 180:
                angle2 = 360 - angle2
            else:
                angle3 = 360 - angle3

        print("angles", angle, angle2, angle3)
        if (angle> angle2 and angle<= angle3) or (angle<= angle2 and angle >= angle3):
            return True
        else:
            return False




## Draw polygon #######################################


line_check = []
lines = []
for i in range(len(xp)):
    line_check.append(0)

for i in range(len(vertices) -1 ):
    x1 = vertices[i][0]
    y1 = vertices[i][1]
    x2 = vertices[i+1][0]
    y2 = vertices[i+1][1]
    lines.append((x1,y1,x2,y2))
    plt.plot([x1,x2],[y1,y2], 'r')

x1 = vertices[0][0]
y1 = vertices[0][1]
x2 = vertices[len(vertices)-1][0]
y2 = vertices[len(vertices)-1][1]
lines.append((x2,y2,x1,y1))
plt.plot([x1,x2],[y1,y2], 'r')

def get_points(lines, index):
    x1 = lines[index][0]
    x2 = lines[index][2]
    y1 = lines[index][1]
    y2 = lines[index][3]
    return x1,y1,x2,y2


####Not used#############################
def line_equation(lines, index, x ): #y = mx + b
    if (lines[index][2] - lines[index][0]) != 0:
        m = (lines[index][3] - lines[index][1])/(lines[index][2] - lines[index][0])
    else:
        return lines[index][3]
    b = lines[index][3] - m * lines[index][2]
    return m*x+b

###########Get Random points####################################################
def random_point(lines, index):
    x1,y1,x2,y2 = get_points(lines, index)
    x = random.uniform(x1, x2)
    if x1 - x2 == 0:
        return (x1, random.uniform(y1,y2))
    y = line_equation(lines, index, x)
    return (x,y)


########## calculate alpha of a contact point##############
def alpha(point, origin):
    p = point
    o = origin
    op = vector(o, p)
    xunitvector = (1, 0)
    ox = vector(o, xunitvector)
    return op.alpha(ox)


def draw_fc(lines, index, point):
    #vector start from contact point and parallel to the line
    _, _,x2,y2 = get_points(lines, index)
    v1 = vector(point, (x2,y2))
    Rv1 = v1.rotation(math.pi/2)
    Rv2 = v1.rotation(math.pi/4)
    Rv3 = v1.rotation((math.pi/2) + (math.pi/4))
    plt.quiver(point[0], point[1], Rv1[0], Rv1[1], color=['g'], scale = 8)
    plt.quiver(point[0], point[1], Rv2[0], Rv2[1], color=['b'], scale = 10)
    plt.quiver(point[0], point[1], Rv3[0], Rv3[1], color=['b'], scale = 10)

def fc(lines, index, point):
    _, _,x2,y2 = get_points(lines, index)
    v1 = vector(point, (x2,y2))
    Rv1 = v1.rotation(math.pi / 2)
    Rv2 = v1.rotation(math.pi/4)
    Rv3 = v1.rotation((math.pi/2) + (math.pi/4))

    p1 = [x1 + x2 for (x1, x2) in zip(Rv1, list(point))]
    p2 = [x1 + x2 for (x1, x2) in zip(Rv2, list(point))]
    p3 = [x1 + x2 for (x1, x2) in zip(Rv3, list(point))]
    rv2 = vector(point, p2)
    rv3 = vector(point, p3)
    rv1 = vector(point, p1)

    return rv2, rv3, rv1



iteration = 0
while 1:
    iteration = iteration + 1

    # assign first contact at any side (randomly).
    line = random.randint(0, len(xp) - 1)
    firstpoint = random_point(lines, line)
    line_check[line] = 1

    #choose different line
    while 1:
        line2 = random.randint(0,len(xp)-1)
        if line2 != line:
            break

    #choose random point
    secondpoint = random_point(lines, line2)

    #unit x vector
    ox = vector((0,0), (1,0))

    #contact one
    fc1, fc2, norm = fc(lines, line, firstpoint)
    vec1 = vector(firstpoint, secondpoint)
    check = vec1.vector_lies_between(fc1, fc2, ox)

    #contact two
    fc3, fc4, norm2 = fc(lines,line2, secondpoint)
    vec2 = vector(secondpoint, firstpoint)
    check2 = vec2.vector_lies_between(fc3, fc4, ox)

    if(check and check2):
        draw_fc(lines, line, firstpoint)
        draw_fc(lines, line2, secondpoint)
        plt.scatter(firstpoint[0], firstpoint[1], s=40)
        plt.scatter(secondpoint[0], secondpoint[1], s=40)
        plt.plot([firstpoint[0], secondpoint[0]], [firstpoint[1], secondpoint[1]], '--k')
        print("Iteration", iteration)

        #write file
        angle1 = norm.alpha(ox)
        angle2 = norm2.alpha(ox)
        dataFile = open('data/grasp_data.txt', "w")
        if firstpoint[0]>0:
            dataFile.write(str(firstpoint[0]*0.1) + ' ' + str(firstpoint[1]*0.1)  + " " + str(angle1) + "\n")
            dataFile.write(str(secondpoint[0]*0.1) + ' ' + str(secondpoint[1]*0.1) + " " + str(angle2) + '\n')
        else:
            dataFile.write(str(secondpoint[0] * 0.1) + ' ' + str(secondpoint[1] * 0.1) + " " + str(angle1) + '\n')
            dataFile.write(str(firstpoint[0] * 0.1) + ' ' + str(firstpoint[1] * 0.1) + " " + str(angle2) + "\n")

        dataFile.close()
        break

    line_check[line] = 0



plt.scatter(x_origin, y_origin, s=10)


plt.axis('equal')
plt.show()


