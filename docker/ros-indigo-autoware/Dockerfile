FROM ros:indigo-ros-base
##### ref: https://hub.docker.com/_/ros/
##### To build docker image
##### docker build . -t ros:ros-indigo-autoware

### Install dependencies for building Autowire https://github.com/CPFL/Autoware
RUN apt-get update && apt-get install -y \
    ros-indigo-pcl-ros \
    ros-indigo-roslint \
    ros-indigo-theora-image-transport \
    libopencv-dev \
    libnlopt-dev freeglut3-dev qt5-default \
    libqt5opengl5-dev libssh2-1-dev libarmadillo-dev \
    libpcap-dev gksu libgl1-mesa-dev \
    ros-indigo-sound-play \
    ros-indigo-nmea-navsat-driver \
    ros-indigo-desktop-full=1.1.4-0* \
    && rm -rf /var/lib/apt/lists/

### Install other tools (optional)
RUN apt-get update && apt-get install -y \
   git \
   openssh-server \
   && rm -rf /var/lib/apt/lists/
   
### Git clone the Autowire and make
