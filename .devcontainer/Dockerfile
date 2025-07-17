FROM ghcr.io/zephyrproject-rtos/ci-base:v0.26-branch

ARG ZSDK_VERSION=0.16.9
ARG ZSDK_TOOLCHAINS="arm-zephyr-eabi"

ENV ZSDK_VERSION=${ZSDK_VERSION}
ENV ZEPHYR_TOOLCHAIN_VARIANT=zephyr
ENV ZEPHYR_SDK_INSTALL_DIR=/opt/zephyr-sdk-${ZSDK_VERSION}
ENV WGET_ARGS="-q --show-progress --progress=bar:force:noscroll"

# Install Zephyr SDK (includes ARM toolchain)
RUN mkdir -p /opt && \
    cd /opt && \
    wget ${WGET_ARGS} https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v${ZSDK_VERSION}/zephyr-sdk-${ZSDK_VERSION}_linux-x86_64_minimal.tar.xz && \
    tar xf zephyr-sdk-${ZSDK_VERSION}_linux-x86_64_minimal.tar.xz && \
    rm zephyr-sdk-${ZSDK_VERSION}_linux-x86_64_minimal.tar.xz && \
    /opt/zephyr-sdk-${ZSDK_VERSION}/setup.sh -t ${ZSDK_TOOLCHAINS} -c

# Clean up
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        usbutils \
        openocd \
        && \
    apt-get clean -y && \
    apt-get autoremove --purge -y && \
    rm -rf /var/lib/apt/lists/*

RUN ln -s /workdir/zephyrrc /root/.zephyrrc

# Setup workspace
WORKDIR /workdir
VOLUME /workdir

# Set the default command to run when the container starts
CMD ["/bin/bash"]
