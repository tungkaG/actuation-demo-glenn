# Copyright (c) 2022-2024, Arm Limited.
# SPDX-License-Identifier: Apache-2.0

From ghcr.io/autowarefoundation/autoware-universe:humble-20231115

ARG ZSDK_VERSION=0.16.3
ARG BUILD_ARCH=aarch64

# Install dependencies
RUN apt-get update && \
    apt install --no-install-recommends -y ninja-build unzip && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Install west
RUN pip3 install -U west

ENV ZEPHYR_SDK_INSTALL_DIR=/opt/zephyr-sdk-${ZSDK_VERSION}

# Install Zephyr SDK
RUN wget -q --show-progress --progress=bar:force:noscroll \
    https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v${ZSDK_VERSION}/zephyr-sdk-${ZSDK_VERSION}_linux-${BUILD_ARCH}.tar.xz && \
    tar xf zephyr-sdk-${ZSDK_VERSION}_linux-${BUILD_ARCH}.tar.xz -C /opt \
        zephyr-sdk-${ZSDK_VERSION}/arm-zephyr-eabi/ \
        zephyr-sdk-${ZSDK_VERSION}/setup.sh \
        zephyr-sdk-${ZSDK_VERSION}/cmake \
        zephyr-sdk-${ZSDK_VERSION}/sdk_toolchains \
        zephyr-sdk-${ZSDK_VERSION}/sdk_version && \
    ${ZEPHYR_SDK_INSTALL_DIR}/setup.sh -c -t arm-zephyr-eabi && \
    rm "zephyr-sdk-${ZSDK_VERSION}_linux-${BUILD_ARCH}.tar.xz"

# Ensure the default ROS middleware is used
RUN echo "unset RMW_IMPLEMENTATION" >> ~/.bashrc

# The gdown version installed in the base image has a breaking bug. Update to
# the next version that has the fix.
# See https://github.com/wkentaro/gdown/issues/291
RUN pip3 install gdown==v4.7.3
