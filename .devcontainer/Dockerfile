FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

RUN mkdir -p /tmp/packages
COPY ./src/param_inspector/package.xml /tmp/packages/package.xml

RUN apt-get update && \
    apt-get install -y \
        python3-colcon-common-extensions \
    && \
    rosdep update && \
    rosdep install --from-paths /tmp/packages --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc

ENV DEBIAN_FRONTEND=