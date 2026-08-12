FROM osrf/ros:kinetic-desktop
LABEL maintainer="Alexandros Philotheou <alefilot@auth.gr>"
# ------------------------------------------------------------------------------
# Build configuration
#
# EXACT_FP=ON          builds with -O2 -fno-fast-math -ffp-contract=off instead
#                      of -Ofast, so that results are reproducible across
#                      compilers at the cost of some speed.
# DETERMINISTIC_FFT=ON makes FFTW select its plan by heuristic instead of by
#                      timing itself, so that results are reproducible across
#                      runs.
#
# Both default to OFF, which is the configuration this package ships with.
# ------------------------------------------------------------------------------
ARG EXACT_FP=OFF
ARG DETERMINISTIC_FFT=OFF
# ------------------------------------------------------------------------------
RUN apt-get update                                                          && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    sudo \
    apt-utils \
    build-essential \
    g++ \
    git \
    curl \
    libfftw3-dev \
    libcgal-dev=4.7-4 \
    libcgal-qt5-dev=4.7-4 \
    libboost-random-dev \
    libgtest-dev \
    python-catkin-tools \
    ros-kinetic-tf2-geometry-msgs                                           && \
    rm -rf /var/lib/apt/lists/*
# ------------------------------------------------------------------------------
# Use bash and create user
# https://roboticseabass.com/2023/07/09/updated-guide-docker-and-ros2/
RUN rm /bin/sh && ln -s /bin/bash /bin/sh
ENV USER=fsmlo
RUN useradd --create-home ${USER}                                           && \
    echo ${USER} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USER}         && \
    chmod 0440 /etc/sudoers.d/${USER}                                       && \
    mkdir -p /home/${USER}
USER ${USER}
WORKDIR /home/${USER}
# ------------------------------------------------------------------------------
RUN mkdir -p /home/${USER}/catkin_ws/src
COPY --chown=${USER}:${USER} fsm_lo/ /home/${USER}/catkin_ws/src/fsm_lo/
# ------------------------------------------------------------------------------
RUN cd /home/${USER}/catkin_ws                                              && \
    source /opt/ros/kinetic/setup.bash                                      && \
    catkin build fsm_lo                                                        \
      --cmake-args                                                             \
        -DFSM_LO_EXACT_FP=${EXACT_FP}                                          \
        -DFSM_LO_DETERMINISTIC_FFT=${DETERMINISTIC_FFT}
# ------------------------------------------------------------------------------
# The next five lines + the entrypoint command will make sure that
# when the container is run the fsm node is roslaunched immediately
RUN echo "#!/bin/bash"                                      > /home/${USER}/fsm_launch.sh
RUN echo "source /opt/ros/kinetic/setup.bash"              >> /home/${USER}/fsm_launch.sh
RUN echo "source /home/${USER}/catkin_ws/devel/setup.bash" >> /home/${USER}/fsm_launch.sh
RUN echo "roslaunch fsm_lo avanti.launch"                  >> /home/${USER}/fsm_launch.sh
RUN chmod +x /home/${USER}/fsm_launch.sh
# ------------------------------------------------------------------------------
ENTRYPOINT bash /home/${USER}/fsm_launch.sh
