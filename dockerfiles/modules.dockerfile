FROM kangsm423/orb2_u18.04:base

ARG BRANCH=feature/CI
ARG DEBIAN_FRONTEND=noninteractive

RUN sudo apt-get update -y && sudo apt-get upgrade -y

RUN useradd -m user && yes password | pawssd user

RUN echo "== Start Debug build == " && \
cd home/ORB_SLAM2 && \
git remote update && \
git fetch --all && \
git checkout ${BRANCH} && \
git pull && \
git branch && \
mkdir build_debug && cd build_debug && \
cmake -DCMAKE_BUILD_TYPE=Debug -GNinja .. && ninja

RUN echo "== Start Release build == " && \
cd home/ORB_SLAM2 && \
git remote update && \
git fetch --all && \
git checkout ${BRANCH} && \
git pull && \
git branch && \
mkdir build_release && cd build_release && \
cmake -DCMAKE_BUILD_TYPE=Release -GNinja .. && ninja
