FROM nvcr.io/nvidia/tensorrt:23.09-py3

# Update package lists and install necessary packages
RUN apt-get update \
    && apt-get install -q -y --no-install-recommends locales tzdata \
    && rm -rf /var/lib/apt/lists/*

# Generate and set the locale to en_US.UTF-8
RUN locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Set the timezone to Asia/Jakarta
RUN ln -fs /usr/share/zoneinfo/Asia/Jakarta /etc/localtime \
    && dpkg-reconfigure -f noninteractive tzdata

# --------------------------------------

# Update package lists and install necessary packages
RUN apt-get update \
    && apt-get install -q -y --no-install-recommends software-properties-common curl sudo \
    && rm -rf /var/lib/apt/lists/*

# Add the universe repository to the apt sources list
RUN add-apt-repository universe

# Download and add the ROS key to the keyring
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package lists and install ROS
RUN apt-get update \
    && apt-get install -y --no-install-recommends ros-humble-desktop ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# --------------------------------------

# Create a user and use it
ARG USERNAME=protanjung
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
USER $USERNAME

# --------------------------------------

# Add ROS to the environment
RUN source /opt/ros/humble/setup.bash \
    && echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Configure ROS_DOMAIN_ID
ENV ROS_DOMAIN_ID=0
RUN echo "export ROS_DOMAIN_ID=$ROS_DOMAIN_ID" >> ~/.bashrc

# Configure ROS_LOCALHOST_ONLY
ENV ROS_LOCALHOST_ONLY=1
RUN echo "export ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY" >> ~/.bashrc

# --------------------------------------

# Update package lists and install necessary packages
RUN sudo apt-get update \
    && sudo apt-get install -q -y --no-install-recommends \
    python3-colcon-common-extensions \
    && sudo rm -rf /var/lib/apt/lists/*

# Add colcon_argcomplete to the environment
RUN source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash \
    && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

# Add colcon_cd to the environment
RUN source /usr/share/colcon_cd/function/colcon_cd.sh \
    && export export _colcon_cd_root=/opt/ros/humble/ \
    && echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc \
    && echo "export _colcon_cd_root=/opt/ros/humble/" >> ~/.bashrc

# --------------------------------------

# Add current user to the 'video' and 'dialout' groups
RUN sudo usermod -aG video $USERNAME
RUN sudo usermod -aG dialout $USERNAME