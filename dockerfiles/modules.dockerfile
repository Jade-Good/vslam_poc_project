FROM celinachild/orbslam2:latest

ARG BRANCH=development
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update -y && apt-get upgrade -y

RUN useradd -m user && yes password | passwd user

RUN echo "== Start Debug build == " &&\
cd /slam/programmers_slam_project_template && \
git remote update && \
git fetch --all && \
git checkout ${BRANCH} && \
git pull && \
git branch && \
mkdir build_debug && cd build_debug && \
cmake -DCMAKE_BUILD_TYPE=Debug -GNinja .. && ninja

RUN echo "== Start Release build == " && \
cd /slam/programmers_slam_project_template && \
git remote update && \
git fetch --all && \
git checkout ${BRANCH} && \
git pull && \
git branch && \
mkdir build_release && cd build_release && \
cmake -DCMAKE_BUILD_TYPE=Release -GNinja .. && ninja
