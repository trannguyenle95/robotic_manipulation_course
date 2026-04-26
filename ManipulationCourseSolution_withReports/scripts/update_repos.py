import numpy as np
import sys
import os
import subprocess
import re


def createDir(directory):
    if not os.path.exists(directory):
        os.makedirs(directory)
    return directory

if __name__ == '__main__':
    if len(sys.argv) != 2:
        sys.exit('Need at least one argument naming the git repo to dowload such as exercise1')
    repo = sys.argv[1]
    usernames = np.genfromtxt('username.txt', dtype='str')
    cloneStr = "git@version.aalto.fi:"
    # git@version.aalto.fi:aitov1_robotic_manipulation/exercise1.git
    for username in usernames:
        username_nodot = username.replace('.','')
        curDir = createDir("ros_ws/src/"+username_nodot+"_"+repo)+"/"
# git@version.aalto.fi:apparvi2_robotic_manipulation/exercise1.git
        clone = "git clone --progress git@version.aalto.fi:" + "robotic_manipulation_students_projects_2021/" + username + "/"+repo+".git "+curDir
        os.system(clone+" 2>> git_output.txt")
        if len(os.listdir("/home/trannguyenle/RemoteWorkingStation/phd_aalto/Course/ManipulationCourseSolution/scripts/ros_ws/src/"+username_nodot+"_"+repo+"/")) != 0:
            xml = open("/home/trannguyenle/RemoteWorkingStation/phd_aalto/Course/ManipulationCourseSolution/scripts/ros_ws/src/"+username_nodot+"_"+repo+"/"+"package.xml", "rt")
            xmldata = xml.read()
            xmldata = xmldata.replace(repo, username_nodot+"_"+repo)
            xml.close()
            xml = open("/home/trannguyenle/RemoteWorkingStation/phd_aalto/Course/ManipulationCourseSolution/scripts/ros_ws/src/"+username_nodot+"_"+repo+"/"+"package.xml", "wt")
            xml.write(xmldata)
            xml.close()
            if (repo == "exercise1"):
                CMake = open("/home/trannguyenle/RemoteWorkingStation/phd_aalto/Course/ManipulationCourseSolution/scripts/ros_ws/src/"+username_nodot+"_"+repo+"/"+"CMakeLists.txt", "rt")
                CMakedata = CMake.read()
                CMakedata = CMakedata.replace(repo, username_nodot+"_"+repo)
                CMakedata = CMakedata.replace('frame_publisher', username_nodot+"_"+'frame_publisher')
                CMake.close()
                CMake = open("/home/trannguyenle/RemoteWorkingStation/phd_aalto/Course/ManipulationCourseSolution/scripts/ros_ws/src/"+username_nodot+"_"+repo+"/"+"CMakeLists.txt", "wt")
                CMake.write(CMakedata)
                CMake.close()
            if (repo == 'exercise2'):
                os.rename(r"/home/trannguyenle/RemoteWorkingStation/phd_aalto/Course/ManipulationCourseSolution/scripts/ros_ws/src/"+username_nodot+"_"+repo+"/"+"src/plan.cpp", r"/home/trannguyenle/RemoteWorkingStation/phd_aalto/Course/ManipulationCourseSolution/scripts/ros_ws/src/"+username_nodot+"_"+repo+"/"+"src/"+ username_nodot +"_plan.cpp")
                CMake = open("/home/trannguyenle/RemoteWorkingStation/phd_aalto/Course/ManipulationCourseSolution/scripts/ros_ws/src/" + username_nodot+"_"+repo+"/"+"CMakeLists.txt", "rt")
                CMakedata = CMake.read()
                CMakedata = CMakedata.replace(repo, username_nodot+"_"+repo)
                # CMakedata = CMakedata.replace('plan', username_nodot+"_"+'plan')

                CMakedata = re.sub(r"\bplan\b", username_nodot+"_"+'plan',CMakedata)

                CMake.close()
                CMake = open("/home/trannguyenle/RemoteWorkingStation/phd_aalto/Course/ManipulationCourseSolution/scripts/ros_ws/src/"+username_nodot+"_"+repo+"/"+"CMakeLists.txt", "wt")
                CMake.write(CMakedata)
                CMake.close()
            if (repo == 'exercise3'):
                os.rename(r"/home/trannguyenle/RemoteWorkingStation/phd_aalto/Course/ManipulationCourseSolution/scripts/ros_ws/src/"+username_nodot+"_"+repo+"/"+"src/pick_and_place.cpp", r"/home/trannguyenle/RemoteWorkingStation/phd_aalto/Course/ManipulationCourseSolution/scripts/ros_ws/src/"+username_nodot+"_"+repo+"/"+"src/"+ username_nodot +"_pick_and_place.cpp")
                CMake = open("/home/trannguyenle/RemoteWorkingStation/phd_aalto/Course/ManipulationCourseSolution/scripts/ros_ws/src/" + username_nodot+"_"+repo+"/"+"CMakeLists.txt", "rt")
                CMakedata = CMake.read()
                CMakedata = CMakedata.replace(repo, username_nodot+"_"+repo)
                # CMakedata = CMakedata.replace('plan', username_nodot+"_"+'plan')

                CMakedata = re.sub(r"\bpick_and_place\b", username_nodot+"_"+'pick_and_place',CMakedata)

                CMake.close()
                CMake = open("/home/trannguyenle/RemoteWorkingStation/phd_aalto/Course/ManipulationCourseSolution/scripts/ros_ws/src/"+username_nodot+"_"+repo+"/"+"CMakeLists.txt", "wt")
                CMake.write(CMakedata)
                CMake.close()
	else:
            print(username +" is empty")
