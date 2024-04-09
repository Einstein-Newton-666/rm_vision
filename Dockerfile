FROM ros:humble-ros-base		

# clone projects
RUN git clone https://gitlab.com/Einstein-Newton-666/rm_vision.git

# create workspace
WORKDIR /rm_vision/ 

# install dependencies and some tools
RUN  sed '1,$d' /etc/apt/sources.list.d/ros2-latest.list && \
     sed -i 's/ports.ubuntu.com/mirrors.tuna.tsinghua.edu.cn/g' /etc/apt/sources.list &&\
     sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'  &&\
     gpg --keyserver keyserver.ubuntu.com --recv F42ED6FBAB17C654 &&apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654  &&\
     apt-get update && rosdep install --from-paths src --ignore-src -r -y && \
     apt-get install ros-humble-foxglove-bridge wget htop vim  libboost-all-dev unzip bash-completion -y 

# COPY abseil-cpp ../usr/local
# COPY protobuf-cpp-3.21.11.zip ../usr/local

#install protobuf   
# RUN cd ../usr/local && unzip protobuf-cpp-3.21.11.zip && cd protobuf-3.21.11 && \
#     ./autogen.sh && ./configure && make && \
#     sudo make install && ldconfig 

RUN  apt-get install libprotobuf-dev -y && \
     apt-get install protobuf-compiler -y

# build
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install \
     --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 1


# setup .zshrc
RUN echo 'export TERM=xterm-256color\n\
     source /rm_vision/install/setup.sh\n\
     source /opt/ros/humble/setup.sh\n\
     eval "$(register-python-argcomplete3 ros2)"\n\
     eval "$(register-python-argcomplete3 colcon)"\n'\
     >> /root/.bashrc


# source entrypoint setup
RUN sed --in-place --expression \
     '$isource "/rm_vision/install/setup.bash"' \
     /ros_entrypoint.sh


