FROM arm64v8/ros:foxy

USER root

RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA
RUN apt-get update && apt-get -y install sudo 
RUN apt-get install -y apt-utils 
RUN apt-get install -y ca-certificates
RUN apt-get install -y git vim 
RUN apt-get install dialog apt-utils -y
RUN apt-get install net-tools -y
RUN apt-get install inetutils-ping -y 
RUN echo 'debconf debconf/frontend select Noninteractive' | sudo debconf-set-selections
RUN sudo apt-get install -y -q
RUN apt-get install --fix-missing
RUN apt-get install -y gcc g++ cmake
RUN apt-get update
RUN apt-get install -y python3-pip
RUN apt-get install -y curl
RUN apt-get install -y software-properties-common
RUN apt-get install -y build-essential cmake
RUN apt-get install -y libeigen3-dev libjsoncpp-dev libtins-dev libpcap-dev
RUN apt-get install -y libcurl4-openssl-dev
RUN apt-get install -y libglfw3-dev libglew-dev libspdlog-dev 
RUN apt-get install -y libpng-dev libflatbuffers-dev

RUN useradd -ms /bin/bash xcraft
RUN echo "xcraft ALL=(ALL) NOPASSWD: ALL" | sudo tee -a /etc/sudoers >/dev/null

RUN python3 -m pip install pip -U
RUN python3 -m pip install grpcio
RUN python3 -m pip install mavsdk
RUN python3 -m pip install pymavlink
RUN python3 -m pip install pyserial
RUN python3 -m pip install ouster-sdk
RUN python3 -m pip install kiss-icp
RUN python3 -m pip install PyYAML --ignore-installed
RUN python3 -m pip install open3d
RUN python3 -m pip install laspy
