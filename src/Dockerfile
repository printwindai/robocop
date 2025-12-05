# Choose one at build time: jazzy (24.04), humble (22.04), iron (22.04)
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

# Set noninteractive for clean installs
ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Etc/UTC \
    ROS_DISTRO=${ROS_DISTRO} \
    COLCON_HOME=/root/.colcon

# Common dev tools + colcon + rosdep + vcs
RUN apt-get update && apt-get install -y --no-install-recommends \
      bash-completion \
      build-essential \
      git \
      python3-colcon-common-extensions \
      python3-rosdep \
      python3-vcstool \
      python3-pip \
      less \
      nano \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep (needed to install package dependencies)
RUN rosdep init || true && rosdep update

# Create workspace
WORKDIR /ws

# 1) Copy metadata first for better Docker layer caching
# If you have a ros2.repos file (vcs), copy it first:
# COPY ros2.repos /ws/
# RUN vcs import src < ros2.repos

# If your packages have package.xml, this helps cache deps
# (Optional) Copy only package manifests to improve caching:
# RUN mkdir -p src
# COPY src/*/package.xml src/

# Install system/ROS dependencies declared in your packages
# (If you copied only manifests, run this now; otherwise this step will run after copying all sources)
# RUN rosdep install --from-paths src --rosdistro ${ROS_DISTRO} -y --ignore-src

# 2) Now copy your whole source tree
COPY src/ /ws/src/

# Install dependencies (safe to re-run; rosdep skips installed stuff)
RUN rosdep install --from-paths src --rosdistro ${ROS_DISTRO} -y --ignore-src

# Build
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --merge-install

# Auto-source workspace in every shell
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc && \
    echo "source /ws/install/setup.bash" >> /etc/bash.bashrc

# The official ROS image provides this entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
