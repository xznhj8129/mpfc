# syntax=docker/dockerfile:1.7

FROM python:3.11-slim-bookworm AS mavlink-router-builder

ARG MAVLINK_ROUTER_REF=v3

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    g++ \
    gcc \
    git \
    meson \
    ninja-build \
    pkg-config \
    systemd \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /tmp
RUN git clone --depth 1 --branch ${MAVLINK_ROUTER_REF} https://github.com/mavlink-router/mavlink-router.git

WORKDIR /tmp/mavlink-router
RUN git submodule update --init --recursive
RUN meson setup build --buildtype=release
RUN ninja -C build
RUN DESTDIR=/tmp/mavlink-router-root ninja -C build install


FROM python:3.11-slim-bookworm

ARG TARGETARCH
ARG TARGETVARIANT
ARG HIVEOS_INSTALL_MAVSDK=auto
ARG HIVEOS_INSTALL_YOLO=auto
ARG FROGPROTO_REF=21bfc55145125aad364491f8f7cdbf2a30e57879
ARG FROGCOT_REF=2db8352da397978db6e6d3e899e3d5892298ec6f
ARG FROGGEOLIB_REF=116f4491cd1bc2a6fe2dd4faaa931af7ef8e1e95
ARG FROGTASTIC_REF=8ed8c6fe97e8c93ab50bfb4ae41942f9b4234dc1
ARG HIVELINK_REF=9411f3c8b2eb96d8e34ddacb117439646f1e59d3
ARG MSPAPI2_REF=e458e8d87f3378b7871a9d807617c97a84687073

ENV PYTHONDONTWRITEBYTECODE=1
ENV PYTHONUNBUFFERED=1
ENV PIP_NO_CACHE_DIR=1
ENV MAIN_CONFIG=/opt/hiveos/flight_cores/test_core/config.yaml
ENV HIVEOS_START_MOSQUITTO=1
ENV HIVEOS_START_MAVLINK_ROUTER=1
ENV HIVEOS_WORKDIR=/opt/hiveos
ENV MOSQUITTO_CONFIG=/etc/mosquitto/mosquitto.conf
ENV MAVLINK_ROUTER_CONFIG=/etc/mavlink-router/main.conf

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash \
    build-essential \
    cargo \
    ca-certificates \
    git \
    gpiod \
    libatomic1 \
    libffi-dev \
    libgpiod2 \
    libstdc++6 \
    libsystemd0 \
    libusb-1.0-0 \
    mosquitto \
    mosquitto-clients \
    procps \
    python3-dev \
    rustc \
    tini \
 && rm -rf /var/lib/apt/lists/*

COPY --from=mavlink-router-builder /tmp/mavlink-router-root/usr/bin/mavlink-routerd /usr/local/bin/mavlink-routerd

RUN mkdir -p \
    /etc/mavlink-router \
    /opt/hiveos \
    /opt/vendor \
    /run/mosquitto \
    /var/log/hiveos \
    /var/log/mavlink-router \
    /var/log/mosquitto

WORKDIR /opt/hiveos

RUN python -m pip install --upgrade pip setuptools wheel
RUN python -m pip install \
    PyYAML \
    PyPubSub \
    crcmod \
    filterpy \
    geojson \
    geographiclib \
    meshtastic \
    mgrs \
    msgpack \
    numpy \
    paho-mqtt \
    pydantic \
    pymavlink \
    pytz \
    pyserial \
    simple-pid \
    xmltodict

RUN git clone https://github.com/xznhj8129/frogproto.git /opt/vendor/frogproto && \
    git -C /opt/vendor/frogproto checkout ${FROGPROTO_REF} && \
    git clone https://github.com/xznhj8129/frogcot.git /opt/vendor/frogcot && \
    git -C /opt/vendor/frogcot checkout ${FROGCOT_REF} && \
    git clone https://github.com/xznhj8129/froggeolib.git /opt/vendor/froggeolib && \
    git -C /opt/vendor/froggeolib checkout ${FROGGEOLIB_REF} && \
    git clone https://github.com/xznhj8129/frogtastic.git /opt/vendor/frogtastic && \
    git -C /opt/vendor/frogtastic checkout ${FROGTASTIC_REF} && \
    git clone https://github.com/xznhj8129/hivelink.git /opt/vendor/hivelink && \
    git -C /opt/vendor/hivelink checkout ${HIVELINK_REF} && \
    git clone https://github.com/xznhj8129/mspapi2.git /opt/vendor/mspapi2 && \
    git -C /opt/vendor/mspapi2 checkout ${MSPAPI2_REF}

RUN python -m pip install --no-deps \
    -e /opt/vendor/frogproto \
    -e /opt/vendor/frogcot \
    -e /opt/vendor/froggeolib \
    -e /opt/vendor/frogtastic \
    -e /opt/vendor/hivelink \
    -e /opt/vendor/mspapi2

RUN install_mavsdk=1 && \
    if [ "${HIVEOS_INSTALL_MAVSDK}" = "0" ] || [ "${HIVEOS_INSTALL_MAVSDK}" = "false" ]; then install_mavsdk=0; fi && \
    if [ "${TARGETARCH}" = "arm" ] && [ "${TARGETVARIANT}" = "v7" ] && [ "${HIVEOS_INSTALL_MAVSDK}" = "auto" ]; then install_mavsdk=0; fi && \
    if [ "${install_mavsdk}" = "1" ]; then python -m pip install grpcio mavsdk; fi

RUN install_yolo=1 && \
    if [ "${HIVEOS_INSTALL_YOLO}" = "0" ] || [ "${HIVEOS_INSTALL_YOLO}" = "false" ]; then install_yolo=0; fi && \
    if [ "${TARGETARCH}" = "arm" ] && [ "${TARGETVARIANT}" = "v7" ] && [ "${HIVEOS_INSTALL_YOLO}" = "auto" ]; then install_yolo=0; fi && \
    if [ "${install_yolo}" = "1" ]; then python -m pip install opencv-python-headless supervision ultralytics; fi

COPY . /opt/hiveos
COPY docker/entrypoint.sh /usr/local/bin/hiveos-entrypoint.sh
COPY config/mosquitto.conf /etc/mosquitto/mosquitto.conf
COPY config/mavlink-router/main.conf /etc/mavlink-router/main.conf

RUN chmod +x /usr/local/bin/hiveos-entrypoint.sh

EXPOSE 1883/tcp 5760/tcp 14540/udp 14550/udp 14551/udp 5550/udp 5555/udp

ENTRYPOINT ["/usr/bin/tini", "--", "/usr/local/bin/hiveos-entrypoint.sh"]
