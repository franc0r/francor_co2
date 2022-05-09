ARG VARIANT="galactic"
FROM franc0r/ros2-francor-dev:${VARIANT}

RUN sudo apt-get update \
    && sudo apt-get install -y \
    python3-serial \
    && sudo apt-get clean