import os

if __name__ == "__main__":
    for i in next(os.walk('ros_ws'))[1]:
        os.chdir('ros_ws/' + str(i))
        os.system("catkin build 2> build.txt")
        os.chdir("../..")
