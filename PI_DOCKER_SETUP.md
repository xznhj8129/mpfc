# HiveOS Docker Setup On Raspberry Pi

Use a 64-bit Raspberry Pi OS if possible. For both Raspberry Pi Zero 2 W and Raspberry Pi 4B, `linux/arm64` is the correct target. The `linux/arm/v7` image is only the fallback for a 32-bit Pi OS, and in that mode MAVSDK and YOLO are skipped by default.

## Pi Setup

On the Pi:

1. Install Docker.

```bash
curl -fsSL https://get.docker.com | sh
sudo usermod -aG docker "$USER"
```

Log out and back in after adding yourself to the `docker` group.

2. Enable the hardware interfaces you actually need.

In `raspi-config`:

- Serial Port: set `login shell = No`
- Serial Port: set `hardware serial = Yes`
- SPI/I2C/Camera: enable only if your hardware uses them
- GPIO is exposed through `/dev/gpiomem` and `/dev/gpiochip*`

3. Put the repo on the Pi, for example:

```bash
mkdir -p /opt
cd /opt
git clone <your-hiveos-repo-url> hiveos
cd /opt/hiveos
```

## Build

Build directly on the Pi:

```bash
cd /opt/hiveos
docker build -t hiveos:pi-arm64 .
```

On a Pi Zero 2 W, if you want the lighter image:

```bash
docker build --build-arg HIVEOS_INSTALL_YOLO=0 -t hiveos:pi-arm64 .
```

If you build on another machine instead, build and push an `arm64` image there, then `docker pull` it on the Pi.

## How Core Selection Works

HiveOS runtime selection works in three layers:

1. `MAIN_CONFIG` points to one YAML file.
2. That YAML file says which core Python module to run with `core.name`.
3. That same YAML file contains the core tuning and plugin wiring.

Example:

```yaml
my_name: uav1
core:
  name: test_takeoff_land
  cfg:
    id: px4_core
    mav_port: 14550
plugins:
- plugin: mavsdk_interface
  cfg:
    conn_str: udp://:<mav_port>
```

That means:

- `MAIN_CONFIG` chooses the file
- `core.name: test_takeoff_land` loads `flight_cores/test_takeoff_land/test_takeoff_land.py`
- `core.cfg` changes mission behavior for that core
- `plugins` chooses which interfaces/adapters are attached to that core

In practice:

- To launch a different existing core, change `MAIN_CONFIG` to point at a different YAML file
- To tune behavior of one core, edit that YAML file's `core.cfg`
- To change ports, serial devices, UDP endpoints, or interface behavior, edit that YAML file's `plugins[*].cfg`
- To change the actual mission logic, edit the Python file under `flight_cores/<core_name>/<core_name>.py`

Do not edit `/opt/hiveos` inside the container image. The container bind-mounts the repo from the host, so edit the files on the Pi host under `/opt/hiveos`, then restart the service.

## Host-Controlled Daemon Setup

The repo now includes a host-side launcher setup so you do not need to recreate the full `docker run` command every time.

Files:

- `/opt/hiveos/config/pi/hiveos.env`
- `/opt/hiveos/config/hiveos-docker.service`
- `/opt/hiveos/scripts/hiveos_docker_ctl.sh`
- `/opt/hiveos/scripts/install_pi_service.sh`

The small file you edit is:

```bash
/opt/hiveos/config/pi/hiveos.env
```

Default contents:

```bash
HIVEOS_IMAGE=hiveos:pi-arm64
HIVEOS_CONTAINER_NAME=hiveos
MAIN_CONFIG=/opt/hiveos/flight_cores/atak_example/config.yaml
HIVEOS_START_MOSQUITTO=1
HIVEOS_START_MAVLINK_ROUTER=1
```

To switch cores, edit only `MAIN_CONFIG`.

Example:

```bash
nano /opt/hiveos/config/pi/hiveos.env
```

Change:

```bash
MAIN_CONFIG=/opt/hiveos/flight_cores/atak_example/config.yaml
```

To:

```bash
MAIN_CONFIG=/opt/hiveos/flight_cores/test_takeoff_land/config.yaml
```

Then restart the service:

```bash
sudo systemctl restart hiveos-docker
```

## Install The Boot Service

Once the image exists on the Pi and the repo is at `/opt/hiveos`:

```bash
cd /opt/hiveos
chmod +x scripts/install_pi_service.sh scripts/hiveos_docker_ctl.sh
sudo ./scripts/install_pi_service.sh
```

That installs a host systemd unit which:

- starts HiveOS automatically on boot
- runs the container in the foreground so systemd supervises it
- restarts it if it exits
- reads `MAIN_CONFIG` from `/opt/hiveos/config/pi/hiveos.env`

## Daily Use

Edit the selected runtime:

```bash
nano /opt/hiveos/config/pi/hiveos.env
```

Edit the actual config or Python:

```bash
nano /opt/hiveos/flight_cores/test_takeoff_land/config.yaml
nano /opt/hiveos/flight_cores/test_takeoff_land/test_takeoff_land.py
nano /opt/hiveos/plugins/mavsdk_interface/mavsdk_interface.py
```

Then restart:

```bash
sudo systemctl restart hiveos-docker
```

Check status:

```bash
sudo systemctl status hiveos-docker
sudo journalctl -u hiveos-docker -f
```

## Device Handling

The host launcher script auto-adds common device nodes if they exist:

- `/dev/bus/usb`
- `/dev/ttyUSB*`
- `/dev/ttyACM*`
- `/dev/serial0`
- `/dev/serial1`
- `/dev/gpiomem`
- `/dev/gpiochip*`
- `/dev/video*`
- `/dev/i2c-*`
- `/dev/spidev*`

That covers the usual USB, serial, GPIO, camera, I2C, and SPI cases without rewriting the `docker run` command each time.

## Important Config Detail

The default router config in [config/mavlink-router/main.conf](config/mavlink-router/main.conf) forwards MAVLink to `127.0.0.1:14540`, so your MAVSDK plugin config should use:

```yaml
conn_str: udp://:14540
```

That applies to [plugins/mavsdk_interface/config_template.yaml](plugins/mavsdk_interface/config_template.yaml) or your runtime config override.

## Useful Commands

```bash
sudo systemctl restart hiveos-docker
sudo systemctl stop hiveos-docker
sudo systemctl status hiveos-docker
sudo journalctl -u hiveos-docker -f
docker exec -it hiveos bash
```

## Notes

- Prefer `linux/arm64` on Pi Zero 2 W and Pi 4B.
- `linux/arm/v7` is only the 32-bit fallback.
- In `arm/v7` mode, MAVSDK and YOLO are intentionally skipped by default.
- `MAIN_CONFIG` is the normal switch for choosing which core config to launch.
- `core.name` inside the YAML is the switch for which Python core module that config uses.
