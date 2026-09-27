FROM osrf/ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    ROS_DISTRO=humble

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    git \
    ros-humble-ackermann-msgs \
 && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install --no-cache-dir --upgrade numpy scipy

WORKDIR /workspace

# Preinstall the package so container starts quickly on first run.
COPY unreal_fable /workspace/unreal_fable
WORKDIR /workspace/unreal_fable/python
RUN python3 -m pip install --no-cache-dir -e .

ENV PYTHONPATH=/workspace/unreal_fable/python:${PYTHONPATH}

WORKDIR /workspace

CMD ["python3", "-m", "fable.ros2_adapter", "--help"]
