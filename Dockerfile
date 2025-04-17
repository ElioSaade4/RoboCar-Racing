FROM ros:iron

SHELL ["/bin/bash", "-c"]

# dependencies
RUN apt-get update --fix-missing && \
    apt-get install -y git \
                       nano \
                       vim \
                       python3-pip \
                       libeigen3-dev \
                       tmux \
                       ros-iron-rviz2 \
                       ros-iron-foxglove-bridge
                       
RUN apt-get -y dist-upgrade
RUN pip3 install transforms3d

# f1tenth gym
RUN git clone https://github.com/f1tenth/f1tenth_gym -4
RUN cd f1tenth_gym && \
    pip3 install -e .

# copy source files to docker container
RUN mkdir -p sim_ws  
COPY ./sim_ws/src/ ./sim_ws/src/

# ros2 gym bridge
RUN source /opt/ros/iron/setup.bash && \
    cd sim_ws/ && \
    apt-get update --fix-missing && \
    rosdep install -i --from-path src --rosdistro iron -y && \
    colcon build

RUN cd sim_ws/ && \
    pip install pandas==1.5.0

WORKDIR '/sim_ws'
ENTRYPOINT ["/bin/bash"]
