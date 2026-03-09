FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# Instalar dependências
RUN apt-get update && apt-get install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-simulations \
    && rm -rf /var/lib/apt/lists/*

# Variáveis de ambiente
ENV TURTLEBOT3_MODEL=waffle
ENV GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models

WORKDIR /workspace

# Copiar apenas source
COPY src /workspace/src

# Build workspace (skip tests)
RUN bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF"

# Entrypoint
RUN echo '#!/bin/bash\nset -e\nsource /opt/ros/humble/setup.bash\nsource /workspace/install/setup.bash\nexec "$@"' > /entrypoint.sh && \
    chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
