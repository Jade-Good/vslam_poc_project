FROM kangsm423/orb2_u18.04_2:base

ARG BRANCH=feature/CI
ARG DEBIAN_FRONTEND=noninteractive
RUN sudo apt-key del F42ED6FBAB17C654 && sudo -E apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN sudo apt-get update -y && sudo apt-get upgrade -y

RUN useradd -m user && yes password | passwd user

RUN echo "== Start Debug build == " && \
cd home/vslam_poc_project && \
git remote update && \
git fetch --all && \
git checkout ${BRANCH} && \
git pull && \
git branch && \
mkdir build_debug && cd build_debug && \
cmake -DCMAKE_BUILD_TYPE=Debug -GNinja .. && ninja

RUN echo "== Start Release build == " && \
cd home/vslam_poc_project && \
git remote update && \
git fetch --all && \
git checkout ${BRANCH} && \
git pull && \
git branch && \
mkdir build_release && cd build_release && \
cmake -DCMAKE_BUILD_TYPE=Release -GNinja .. && ninja