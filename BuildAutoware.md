## Using Dockerfile

### Build
```
cd docker/ros-indigo-autoware
docker build . -t ai-world-car:ros-indigo-autoware
```
### Run
```
docker run -it --rm \
   --net host \
   --env="DISPLAY" \
   -v "$HOME/Projects:/Projects" \
   -v "$HOME/.Xauthority:/root/.Xauthority:rw" \
   ai-world-car:ros-indigo-autoware /bin/bash
```
### TODO https://github.com/CPFL/Autoware
```
cd $HOME
$ git clone https://github.com/CPFL/Autoware.git
$ cd ~/Autoware/ros/src
$ catkin_init_workspace
$ cd ../
$ ./catkin_make_release
```
