FROM ubuntu:18.04

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
  libeigen3-dev \
  libgl1-mesa-dev \
  libglew-dev \
  libpython2.7-dev \
  libegl1-mesa-dev \
  libwayland-dev \
  libxkbcommon-dev \
  wayland-protocols \
  qt5-default

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

# get pangolin
WORKDIR /home/dev
RUN git clone https://github.com/stevenlovegrove/Pangolin.git

# install pangolin
WORKDIR /home/dev/Pangolin/build
RUN cmake ..
RUN cmake --build .
RUN cmake --install . 

# Upon start, run ssh daemon
CMD ["/usr/sbin/sshd", "-D"]
