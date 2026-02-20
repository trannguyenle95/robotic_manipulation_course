
## Recommended for all users
- Visual Studio Code for your original OS
    - allows you to easily code within the container if you want

## Two versions for the GUI
There are two versions for how to run things based on the GUI backend. The first version is the "native" version which works on Windows 11 and should also work on Linux. This offers better FPS and allows the GUI applications to simply open in another window. Linux users should also have the benefit of using their GPU for hardware acceleration of OpenGL drawing calls.

The other version is to use the GUI applications via an internet browser. This method offers lower fps and is only recommended if you cannot get the native version to work (Windows 10 or older, Mac users...).

To use the browser version, you should add "-f browser.yaml" after "docker compose" commands, or alternatively, remove/rename the compose.yaml file and rename the browser.yaml file to compose.yaml and you can directly use the commands presented here. Note that the browser version requires running another container for which the image takes around 608MB space from your storage.

# Building the image
There are some Ros packages that need to be downloaded from the Aalto GitLab that require login, thus it is not automated to avoid accidentally sharing login credentials.

Depending on your computer, the build is expected to last from 3 to 5 minutes assuming there is a sufficiently fast internet connection.

1. Go to the robotic manipulation git lab page and log in
    - https://version.aalto.fi/gitlab/robotic_manipulation_2023 was for 2023 course

2. Open the projects "franka_ros", "lumi_testbed" and "mujoco_ros_control"

3. Download franka_ros, lumi_tesbed and mujoco_ros_control projects as "tar.gz" files (the down arrow on the left to Clone button) from the following branches:
    - franka_ros: manipulation_course
    - lumi_testbed: master and two_robots
    - mujoco_ros_control: master 

4. Place all the "tar.gz" files in this folder and check on the top of Dockerfile that the names match without the .tar.gz endings.
    - During the 2023 course we started with the master branch for lumi_testbed and then moved to the two_robots branch on Exercises 5 and 6.
    - If you need to switch branches, change the file name in the Dockerfile and re-build the image with docker compose build.

5. Open up your terminal (PowerShell or WSL terminal both should work for Windows) and navigate to this folder.

6. Type in "docker compose build" without the " symbols. If the build finishes without any errors, everything is ready and you have a working image.
    - If you are using the browser compose file, the novnc image will be automatically build for you as well.

# Configuring the container
These are all done within the compose.yaml file. Everything you change here will be in effect anytime you start a new container or restart your current one.

## Required steps
There are many ways of achieving things with Docker but this is the recommended way for running and editing code within a container that you want to also access using your host machine.

1. In the compose.yaml file, edit the path to the exercise folders on your host computer under the volumes parameters. For the format you can use any of the following
    - /absolute/path/to/exercise/folder/on/host:/root/catkin_ws/src/funny_task
    - ./relative/path/also/works:/root/catkin_ws/src/joyful_task
    - ../relative/can/also/traverse/up:/root/catkin_ws/src/playful_task
    - NOTICE: you must always mount the exercise files to the container under /root/catkin_ws/src/ so that they are found by ros!!!

For Linux users:

2. Linux users may also need to configure at least the "DISPLAY" environment variable to point to the X11 socket that accepts connections to support the GUI applicatons.
    - To assign a value to an environment variable that will not be inherited from the host, use the format "- DISPLAY=some.value.here"

3. Linux users may also need to edit or remove the /mnt/wslg entry under the volumes.

## Optional steps

1. Configuring bash_aliases and tmux.conf
    - just edit the bash_aliases.sh and tmux.conf files and source bash_aliases / launch a new tmux seesion

2. If you want to use GPU acceleration with an NVIDIA GPU, you can append the following entries into the compose.yaml files under ros. However, OpenGL GPU acceleration currently only works for Linux.

```
services:
    ros:
        environment:
            - NVIDIA_VISIBLE_DEVICES=all
            - NVIDIA_DRIVER_CAPABILITIES=all

        deploy:
            resources:
                reservations:
                    devices:
                    - driver: nvidia
                        count: all
                        capabilities: [gpu]
```

3. If you want the ros container to automatically restart itself, make sure Docker Desktop is automatically running on launch and then add the following to compose.yaml file

```
services:
    ros:
        restart: unless-stopped
```
The options are "unless-stopped", "on-failure" and "always", pick what you want.

# Running the container and working with code

You have two options to work with the container. Either edit the code on your host or edit the code within the container. Editing on the host is more lightweight but editing from the container for example using Visual Studio Code allows you to search for some function definitions and autocompletion that exist in the installed packages should you need that.

1. Open a Terminal, navigate to this folder and type in "docker compose up -d"
    - This will start roscore for you in the container if you did not edit the entrypoint.sh file.
    - If you need to see what's happening, enter the command without -d option. You can then use CTRL+C to stop the container.
    - If you did enable auto restart, you only need to type this command once

2. To edit the exercise code, you have two options

    1. Edit the code directly on your host computer with the method of your choice.

    2. Edit the code within a container.
        - Open Visual Studio Code.
        - Open command palette (CTRL+SHIFT+P in Win11), type in "attach" without quotes.
        - Select ros and wait for the Visual Studio Code to be ready.
        - open the exercise folder with the open folder button

3. To execute code within the container you need a terminal to the container and you have two options to do it:

    1. Use a terminal window on your host.
        - You can type "docker exec -it ros bash" from anywhere to start an interactive terminal on the ros container that runs bash

    2. Use the Visual Studio Code that is connected to the container.
        - Simply open a terminal window (CTRL+SHIFT+U and select the Terminal tab on Win11).

4. Navigate to ~/catkin_ws once you have a terminal to the container.
    - Type in "catkin_make" and hit enter. This will compile the exercise code for you. Do this anytime you make any changes or when you have started a fresh container from the image and need to run the code.

5. In case you are using the browser version, you should also have another container named theasp/novnc automatically running, this is a container running noVNC allowing you to have a virtual desktop for the ros container in your browser:
    - Connect to localhost:8080 and select vnc.html or connect directly to localhost:8080/vnc.html.
    - Click connect on the button.
    - if you have trouble seeing all the things on the desktop, hover over to the left and select the cog wheel, change scaling mode to Local Scaling

6. After you are done, type in "docker compose down" on the host machine terminal. This should take around 10 seconds as the container will have to be forcibly stopped to exit the roscore.
    - you can skip this step if you enabled automatic restart unless you want to make it stop

Don't forget to push the solved exercises to Aalto GitLab using the host machine!!!

# Pro tips for advanced or curious users
The default Dockerfile installs tmux and the default tmux.conf file changes the splitting commands to be "CTRL+B, V" for vertical split and "CTRL+B, H" for horizontal splitting. Very handy instead of using multiple terminals. Use "CTRL+B, arrow-key" or the mouse to navigate around.

- For curious users, tmux is a 'terminal multiplexer' application and allows you to run multiple terminal windows or 'panes' within a single terminal. You can navigate between them easily or even send some to run tasks on the background so that you can come at any point and see how the process is doing. Recommended practice is to use "tmux new -s session_name" to start a new named session. You can always type "exit" which will close a pane, or the session if only one pane is left. You can also detach a session and come back to it later, for more information type "man tmux".

You can also add in bash aliases in the bash_aliases file on this folder either from the host or within the container for example with vim ~/.bash_aliases. Useful commands are for example:
```
alias sim="tmux new -ds sim 'roslaunch exercise6 sim.launch'"
alias killsim="tmux kill-session -t sim"
alias run="tmux new -ds run 'rosrun exercise6 sim.launch'"
alias killrun="tmux kill-session -t run"
alias data="mv /tmp/exercise6_robot1.csv ~/catkin_ws/src/ex/report/figures/exercise6_robot1.csv && mv /tmp/exercise6_robot2.csv ~/catkin_ws/src/ex/report/figures/exercise6_robot2.csv"
```

Where the sim/killsim run/killrun can be used to launch the background processes and to run the exercise 6 task as a detached tmux sessions as well as to forcibly shut them down. The data alias in the above example is handy if you need to move recorded data into another folder for plotting. These can be edited as needed, just remember to source ~/.bash_alias after editing the file from host or within the container.

To streamline starting the container, you can add for example the following to your host machine's ~/.bash_aliases file:

```
alias ros-start="cd /path/to/this/folder && docker compose up -d && cd $HOME && docker exec -it ros bash"
alias ros-end="cd /path/to/this/folder && docker compose down && cd $HOME
```
Remember to source your .bash_aliases file or open a new terminal window for the aliases to work!
