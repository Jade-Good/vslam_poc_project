FROM ubuntu:focal

# MAINTAINER changh95
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update -y && apt-get upgrade -y

RUN apt-get install build-essential -y && \
# Related to build...
apt-get install cmake -y && \
apt-get install git -y && \
apt-get install sudo -y && \
apt-get install wget -y && \
apt-get install ninja-build -y && \
apt-get install software-properties-common -y && \
apt-get install python3 -y && \
apt-get install python3-pip -y && \
# Related to JetBrains CLion Docker develpoment...
apt-get install -y ssh && \
apt-get install -y gcc && \
apt-get install -y g++ && \
apt-get install -y gdb && \
apt-get install -y clang && \
apt-get install -y cmake && \
apt-get install -y rsync && \
apt-get install -y tar && \
apt-get install -y mesa-utils && \
# Related to X11 remote display
apt-get install -y libgl1-mesa-glx && \
apt-get install -y libglu1-mesa-dev && \
apt-get install -y mesa-common-dev && \
apt-get install -y x11-utils && \
apt-get install -y x11-apps && \
apt-get clean

RUN pip3 install pyyaml
RUN pip3 install gitpython

RUN apt-get autoclean

RUN mkdir slam && cd slam && \
    git clone https://github.com/changh95/programmers_slam_project_template.git &&\
    cd programmers_slam_project_template && ./buildDeps.py --d --system