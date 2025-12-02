#!/bin/bash

VOLUMES="-v $(pwd)/../voxl-os1-server:/home/voxl-os1-server -v /media/ouster:/media/ouster -v /tmp:/tmp -v /etc/modalai:/etc/modalai -v /dev:/dev"

docker run -it --entrypoint "/bin/bash" --rm --privileged --net=host --ipc=host ${VOLUMES} voxl-os1-server -c "source /opt/ros/foxy/setup.bash && HOME=/home && cd ~/voxl-os1-server && colcon build && echo 'source install/setup.bash' >> ~/.bashrc && /bin/bash"
