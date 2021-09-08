ARG cuda_version=11.4.1
ARG ubuntu_version=18.04

FROM nvidia/cudagl:${cuda_version}-devel-ubuntu${ubuntu_version}

#RUN rm /etc/apt/sources.list.d/nvidia-ml.list
RUN apt-get clean && apt-get update
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
  apt-utils \
  build-essential \
  gcc \
  g++ \
  gdb \
  gdbserver \
  openssh-server \
  clang \
  cmake \
  rsync \
  git \
  pkg-config \
  mesa-utils \
  libgl1-mesa-glx \
  libgl1-mesa-dev \
  libglew-dev \
  libpython2.7-dev \
  libegl1-mesa-dev \
  libwayland-dev \
  libxkbcommon-dev \
  wayland-protocols \
  libglu1-mesa-dev \
  freeglut3-dev \
  mesa-common-dev \
  libgtk-3-dev \
  libavcodec-dev \
  libavformat-dev \
  libswscale-dev \
  libv4l-dev \
  libxvidcore-dev \
  libx264-dev \
  libjpeg-dev \
  libpng-dev \
  libtiff-dev \
  gfortran \
  openexr \
  libatlas-base-dev \
  python3-dev \
  python3-numpy \
  libtbb2 \
  libtbb-dev \
  libdc1394-22-dev \
  libpcl-dev \
  libgoogle-glog-dev \
  libgflags-dev \
  libsuitesparse-dev

# Permit root user to login by ssh
RUN mkdir /var/run/sshd
RUN echo 'root:root' | chpasswd
RUN sed -i 's/PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config

# SSH login fix. Otherwise user is kicked off after login
RUN sed 's@session\s*required\s*pam_loginuid.so@session optional pam_loginuid.so@g' -i /etc/pam.d/sshd

ENV NOTVISIBLE "in users profile"
RUN echo "export VISIBLE=now" >> /etc/profile

# 22 for ssh server. 7777 for gdb server.
EXPOSE 22 7777

# Create dev user with password 'dev'
RUN useradd -ms /bin/bash dev
RUN echo 'dev:dev' | chpasswd

# generate pangolin
#WORKDIR /home/dev
#RUN git clone https://github.com/stevenlovegrove/Pangolin.git
#WORKDIR /home/dev/Pangolin/build
#RUN cmake ..
#RUN make
#RUN make install

# install opencv
WORKDIR /home/dev/
RUN git clone https://github.com/opencv/opencv.git
RUN git clone https://github.com/opencv/opencv_contrib.git
WORKDIR /home/dev/opencv/build
RUN cmake -DOPENCV_EXTRA_MODULES_PATH=/home/dev/opencv_contrib/modules ..
RUN make
RUN make install

RUN apt-get install libxmu-dev libxi-dev

#install eigen
WORKDIR /home/dev/
RUN git clone https://gitlab.com/libeigen/eigen
WORKDIR /home/dev/eigen
RUN git checkout 3.3.0
WORKDIR /home/dev/eigen/build
RUN cmake ..
RUN make
RUN make install

# install sophos
WORKDIR /home/dev/
RUN git clone https://github.com/strasdat/Sophus
WORKDIR /home/dev/Sophus/build
RUN cmake -DUSE_BASIC_LOGGING=ON ..
RUN make
RUN make install

# install ceres
WORKDIR /home/dev/
RUN wget http://ceres-solver.org/ceres-solver-2.0.0.tar.gz
RUN tar zxf ceres-solver-2.0.0.tar.gz
WORKDIR /home/dev/ceres-solver-2.0.0/build
RUN cmake ..
RUN make
RUN make install

# install g2o
WORKDIR /home/dev/
RUN git clone https://github.com/RainerKuemmerle/g2o
WORKDIR /home/dev/g2o/build
RUN cmake -std=c++14 ..
RUN make
RUN make install

# install fmt
WORKDIR /home/dev/
RUN git clone https://github.com/fmtlib/fmt
WORKDIR /home/dev/fmt/build
RUN cmake ..
RUN make
RUN make install

# X11 forwarding
ENV DISPLAY :0
RUN export LIBGL_ALWAYS_INDIRECT=1

WORKDIR /home/dev/

# Upon start, run ssh daemon
CMD ["/usr/sbin/sshd", "-D"]
