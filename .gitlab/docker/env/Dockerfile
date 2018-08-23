ARG UBUNTU_DISTRO

FROM ubuntu:${UBUNTU_DISTRO}
ARG UBUNTU_DISTRO
ARG ROS_DISTRO

RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections
RUN apt-get update
RUN apt-get install -y \
      wget \
      sudo \
      apt-utils \
      git \
      ninja-build \
      sed \
      python-bloom \
      devscripts \
      debhelper


WORKDIR /home
COPY setup.sh setup.sh
COPY package.xml package.xml
RUN ./setup.sh ${UBUNTU_DISTRO} ${ROS_DISTRO}
