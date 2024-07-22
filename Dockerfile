FROM mxwilliam/mxck:mxck-noetic-perception-l4t-35.2.1

# Update system and install ROS packages
RUN apt install --yes \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-geometry-msgs
    
# Upgrade pip and install Python packages
RUN python3 -m pip install \
    gdown \
    qpsolvers \ 
    clarabel \ 
    highspy \ 
    transformations \
    statistics \
    scipy \ 
    typing

   
# Replace ros_entrypoint.sh
COPY ./ros_entrypoint.sh /ros_entrypoint.sh

# Copy autorun script
COPY ./autorun.sh /

# Set the entrypoint and default command
ENTRYPOINT ["/autorun.sh"]
CMD ["false"]

# Set working directory
WORKDIR /noetic_ws
