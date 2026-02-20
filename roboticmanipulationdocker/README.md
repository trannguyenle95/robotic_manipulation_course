# Running Exercises in Docker

At the start of the course you will need to clone this repository to get the "robotmanipulationdocker" directory on your local machine.
For each exercise, these are the commands that are needed to run the Docker container. While in the "robotmanipulationdocker" open a terminal and run the following commands:

0) Git clone the directory for an exercise to the same parent directory that contains "robotmanipulationdocker". Edit "compose.yaml" to set the correct exercise number on row 24 (two changes on the line: "../exercise[insertnumberhere]:/root/catkin_ws/src/ex[insertnumberhere]).
1) Run "docker compose up".
2) Open a new terminal and run "docker exec -it ros /bin/bash". This acts as a new terminal in the container which was launched in step 1). If you need multiple terminals, just open more terminals on the host machine and run the same command again.
3) To access GUI windows opened by the code, access address "localhost:8080/vnc.html" in a web browser and click connect.
4) You can do updates to the code in the exercise folder on the host machine, and it will automatically get synchronized in the container. However, you must run "catkin_make" in the container for ROS to be able to use the updated code.
5) Once finished working through Docker, run "docker compose down" to close the container. The code changes will be saved on the local system.
6) Exercises 5 and 6 use two robot arms, and for this configuration you need to change the Dockerfile by uncommenting row 10 and commenting row 9 away. If you go back to exercises 1-4 you need to undo this change. After updating the Dockerfile you need to run "docker compose up --build" when launching a container the first time after making changes to the Docker file, otherwise the changes will not get added to the image and thus activated in the corresponding containers.


# Why Docker?
Docker is using things called containers, which include everything an application needs in order to run from operating system calls to middleware. That is, they offer a standardized environment at the cost of additional storage memory.

This makes docker a much more light weight option to for example VM Box, which always virtualizes a complete machine. Furthermore, Docker containers are built with Dockerfiles, so you are able to see exactly what is in the container and even easily modify the content to your needs.

## How does it work?
Dockerfiles are like recipes and they are used to build images. An image is a collection of software and a state, which we can use to launch a container that always begins in that same state. A container is the actual process that runs the code. No matter what changes you do in the container, unless you commit the changes to a new image, you will always start fresh from the same point when launching a new container from a given image.

This means that any files you modify within the container will not be saved, as they reside within temporary files on your host machine, unless you mount files or folders from any other location on your host machine. Changes made to mounted volumes will stay, as they are part of your host machine, not the container.

# Requirements
Assuming you already have a Docker engine (and WSL on Windows), the final ros environment image will take around 3.06GB space so at least that amount of storage memory should be needed. However, after you are done with the image, simply removing it with "docker rm image_name:tag_name" will get rid of all the installed apps from your system! No dangling registry entries, no hidden files here and there.

## Requirements for Windows users
- WSL2 installed
    - open PowerShell
    - type in "wsl --install" without the quotes
    - it will default to wsl2 mode

- a Linux distro running on the WSL2
    - open Microsoft Store and download for example Ubuntu 22.04.1LTS

- Docker Desktop
    - install Docker Desktop https://www.docker.com/products/docker-desktop/
    - in the settings make sure it is using WSL2 backend, should be on by default

## Requirements for Linux
- Docker Engine installed https://docs.docker.com/engine/install/
- alternatively Docker Desktop may be installed, as it will include Docker Engine https://www.docker.com/products/docker-desktop/

# Requirements for Mac users
- install Docker Desktop https://www.docker.com/products/docker-desktop/
- when it asks to access the folder, answer yes