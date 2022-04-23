FROM nvidia/cuda:11.2.0-cudnn8-devel-ubuntu20.04 AS CUDA

FROM osrf/ros:noetic-desktop-full

MAINTAINER Chenhui Zhang
LABEL version="2.0"
LABEL description="Docker Image for GEM Simulator"

COPY --from=CUDA /usr/local/cuda-11.2/lib64/ /usr/local/cuda-11.2/lib64/
COPY --from=CUDA /usr/local/cuda-11.2/include/ /usr/local/cuda-11.2/include/
COPY --from=CUDA /lib/x86_64-linux-gnu/ /lib/x86_64-linux-gnu/

# Install packages without prompting the user to answer any questions
ENV DEBIAN_FRONTEND noninteractive

RUN apt update && apt install -y curl wget git vim htop xauth psmisc zsh tmux python3-pip python3-tk

ARG USER=ros
ARG PASS=ros

RUN rm /etc/ros/rosdep/sources.list.d/20-default.list && apt update -y && \
    apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential && \
    rosdep init
RUN apt update -y && apt install -y ros-noetic-ackermann-msgs ros-noetic-geometry2 ros-noetic-hector-gazebo \
    ros-noetic-hector-models ros-noetic-jsk-rviz-plugins ros-noetic-ros-control ros-noetic-ros-controllers \
    ros-noetic-velodyne-simulator ros-noetic-joy ros-noetic-gps-common libgeos++-dev libpcap-dev && apt clean

RUN useradd -rm -s /bin/zsh -G sudo -u 1000 $USER && echo $USER:$PASS | chpasswd
USER $USER
WORKDIR /home/$USER
SHELL ["/bin/zsh", "-ec"]

RUN rosdep update
RUN wget https://github.com/robbyrussell/oh-my-zsh/raw/master/tools/install.sh -O - | zsh || true
RUN sed -i '/plugins=(/c\plugins=(git git-flow adb pyenv tmux)' /home/$USER/.zshrc
RUN echo "source /opt/ros/noetic/setup.zsh" >> ~/.zshrc

RUN pip3 install tensorflow-gpu==2.5.0 torch==1.7.1 torchvision==0.8.2
RUN pip3 install matplotlib==3.5.1 fonttools==4.33.2 requests==2.27.1 Pillow==9.1.0 scipy==1.8.0 pandas==1.4.2 \
    pytz==2022.1 python-dateutil==2.8.2 tqdm seaborn sklearn charset-normalizer
ENV LD_LIBRARY_PATH=/usr/local/cuda-11.2/lib64/:/lib/x86_64-linux-gnu/:$LD_LIBRARY_PATH
WORKDIR /home/$USER/gem

ENTRYPOINT ["/ros_entrypoint.sh", "/bin/zsh"]
