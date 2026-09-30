# MPFC Raspberry Pi appliance

This directory builds the MPFC companion-computer appliance for a Raspberry Pi
Zero 2 W class target. The same raw image can also be booted under QEMU for ARM
qualification; normal MPFC development uses the x86 KVM runtime managed by
Sigmac3.

The core image contains MPFC, installed OCCID and HiveLink packages, a Python
runtime, loopback-only Mosquitto, SSH, and the MPFC systemd runtime. Optional
physical-node capabilities are composed into that core image only when selected.
The appliance is independent of Sigma.

## Build model

```text
build-image
    |
    +--> build-image-core
    |      pinned Raspberry Pi OS
    |      MPFC + OCCID + HiveLink + MAVSDK + Mosquitto
    |
    +--> selected deploy/rpi/components/*
    |
    +--> provisioning directory -> /home/mpfc/keys
    |
    +--> final image + manifest
```

`build-image-core` retains the existing cached system/Python build:

1. A pinned Raspberry Pi OS Lite ARM64 image is expanded and provisioned with
   the small system/runtime package set. This prepared OS layer is cached by the
   base-image hash, image size, and `install-rootfs` content.
2. MPFC/HiveLink Python dependencies that have ARM64 wheels are resolved and
   installed by host x86 Python/pip using explicit target platform, Python, and
   ABI selectors. The resulting target `site-packages` layer is cached by the
   resolved requirements inputs.
3. `crcmod` is supplied by the Debian ARM64 `python3-crcmod` package because it
   contains a target-native extension.
4. OCCID and HiveLink are installed into the target virtualenv as normal Python
   packages from the selected source checkouts. Their source trees are not
   copied into the appliance and no runtime path override is used.
5. MPFC application source, service files, identity, password, network settings,
   and runtime configuration are host-side overlays onto the prepared image.

`build-image` then applies only enabled component installers and provisioning.
Component definitions are build inputs and are removed from `/opt/mpfc` before
emitting the image, so an unselected component contributes no package, source,
driver, service, or dependency to the appliance.

`MPFC_BUILD_CACHE` may point at a persistent cache outside the source checkout.
Sigmac3 defaults this to `~/.cache/sigmac3/mpfc`.

## Image configuration

The authoritative non-interactive interface is an INI file:

```ini
[image]
hostname=mpfc
target=pi-zero-2w

[components]
wfb-ng=yes

[wfb-ng]
driver=rtl8812eu
wifi_channel=161
wifi_region=CA

[provisioning]
directory=/path/to/provisioning
```

See `image.conf.example`.

Build it directly:

```bash
sudo ./deploy/rpi/build-image --config ./deploy/rpi/image.conf
```

If `deploy/rpi/image.conf` exists, `build-image` uses it automatically. The file
is ignored by Git so local deployment choices do not become repository state.

### Terminal configurator

`configure` is only a small writer for the same INI contract. It contains no
image-build implementation:

```bash
./deploy/rpi/configure
```

It writes `deploy/rpi/image.conf`, asks whether WFB-NG is wanted, asks for the
WFB radio driver and basic radio settings when selected, and can invoke
`build-image` after writing the file.

New optional capabilities belong under:

```text
deploy/rpi/components/<component>/install-rootfs
```

and are enabled through `[components]`. An enabled component without an
installer is a hard error; there are no placeholder profiles or silent fallbacks.

## WFB-NG component

WFB-NG is the first optional image component. It follows the upstream WFB-NG
Setup HOWTO rather than treating a WFB image as the base operating system.

Supported driver choices:

```text
rtl8812au
rtl8812eu
```

The component:

- installs WFB-NG into the existing MPFC Raspberry Pi OS appliance;
- installs matching Raspberry Pi `linux-image-rpi-v8` and
  `linux-headers-rpi-v8` packages before building the patched driver;
- builds the selected patched Realtek driver through DKMS for the target Pi v8
  kernel, explicitly rather than using the build host's `uname`;
- writes `/etc/wifibroadcast.cfg` from the selected channel/region;
- enables `wifibroadcast@drone.service`;
- keeps WFB-NG away from the FC serial device: MPFC remains the owner of the
  physical MAVLink endpoint. WFB's drone MAVLink profile listens on local UDP
  `127.0.0.1:14550` for an explicit bridge/router if one is later desired;
- listens for drone video input on UDP `0.0.0.0:5602`;
- uses upstream NIC autodetection rather than baking a developer-specific
  `wlanX` name into the image.

The component pins its WFB package/driver inputs in its installer and records the
actual WFB version, driver, driver revision, target kernel, channel, and region in
the final image manifest.

WFB-NG expects the drone key at `/etc/drone.key`. The appliance keeps the
canonical provisioning boundary at `/home/mpfc/keys`, so the component installs:

```text
/etc/drone.key -> /home/mpfc/keys/drone.key
```

A WFB-enabled image may be built without provisioning, but the WFB service is
not operational until a matching `drone.key` is present.

## Provisioning

Deployment-specific keys, certificates, identities, and small configuration
files are supplied as ordinary data:

```bash
sudo ./deploy/rpi/build-image \
  --config ./deploy/rpi/image.conf \
  --provision /path/to/provisioning
```

`--provision` overrides `[provisioning] directory=...` from the INI file.
The directory is copied verbatim as data to:

```text
/home/mpfc/keys/
```

with deliberately simple permissions:

```text
/home/mpfc/keys      mpfc:mpfc 0700
regular files        mpfc:mpfc 0600
subdirectories       mpfc:mpfc 0700
```

The builder never executes provisioning content. Components adapt upstream paths
back to `/home/mpfc/keys`; deployment material does not create another discovery
mechanism.

`provisioning.example/` documents a non-secret example structure. Keep real
provisioning outside Git.

The image manifest records only relative provisioned filenames and a deterministic
hash of the provisioning tree. It never copies secret contents into the manifest.

## Core runtime configuration

Standalone core defaults live in `deploy/rpi/defaults.env`. They define node
identity, control/guest addresses, network prefix, HiveLink port, bridge/tap
names, physical/VM MAVLink connections, and the appliance password.

Default login:

```text
user:     mpfc
password: mpfc
```

Override `MPFC_PI_PASSWORD` before building/booting when another appliance
password is desired. Password authentication is deliberate; the appliance does
not depend on a developer's personal SSH key.

At runtime:

- `/etc/mpfc/runtime.env` contains physical/default appliance settings;
- QEMU supplies VM-only settings on the removable `MPFC_CONFIG` disk;
- `run-mpfc` renders `/run/mpfc/config.yaml` immediately before MPFC starts.

OCCID and HiveLink are ordinary installed packages in `/opt/mpfc/.venv`. There
is no `OCCID_PATH`, `HIVELINK_PATH`, source `.pth`, `/opt/occid`, or
`/opt/hivelink` runtime import mechanism.

## Host prerequisites

On Debian, Ubuntu, or Mint:

```bash
sudo apt update
sudo apt install -y \
  git curl python3 python3-venv python3-pip openssl \
  rsync xz-utils parted e2fsprogs \
  qemu-user-static binfmt-support qemu-system-arm \
  dosfstools mtools device-tree-compiler \
  iproute2 openssh-client sshpass iputils-ping
```

The first prepared-system cache miss and optional component installation use
`qemu-aarch64-static`. ARM pip is not used for the core MPFC Python layer.

## Build the image

Keep MPFC, OCCID, and HiveLink as sibling checkouts, then run either:

```bash
sudo ./deploy/rpi/build-image
```

or:

```bash
sudo ./deploy/rpi/build-image --config my-drone.conf
```

The OCCID/HiveLink checkout paths are build inputs only. `--occid-path` and
`--hivelink-path` may select other checkouts without creating runtime path
coupling in the resulting image.

Outputs under `deploy/rpi/dist/`:

```text
mpfc-rpi-zero2w.img
mpfc-rpi-zero2w.img.sha256
mpfc-rpi-zero2w.img.manifest
mpfc-rpi-zero2w.img.kernel8.img
mpfc-rpi-zero2w.img.raspi3ap.dtb
```

The final manifest records the core base/cache/Python/source information plus
image target/hostname, component selection and component-owned version data,
and provisioning filename/hash evidence.

## Physical Pi

Write the raw image with Raspberry Pi Imager or another raw image writer, for
example:

```bash
sudo dd if=deploy/rpi/dist/mpfc-rpi-zero2w.img \
  of=/dev/sdX bs=4M status=progress conv=fsync
```

After boot, configure the actual FC serial endpoint:

```bash
sudo /opt/mpfc/deploy/rpi/configure-fc /dev/ttyAMA0 921600
```

or:

```bash
sudo /opt/mpfc/deploy/rpi/configure-fc /dev/ttyUSB0 460800
```

`configure-fc` changes only `MPFC_MAVLINK_CONNECTION`.

## Virtual appliance

Foreground:

```bash
./deploy/rpi/pi-vm up
```

Background:

```bash
./deploy/rpi/pi-vm start
```

The host bridge owns only its `/32` control address and one explicit `/32` route
to the guest, so it never claims the physical LAN prefix.

QEMU uses a disposable DTB copy that disables the BCM2835 PM/watchdog provider
which current Raspberry Pi kernels probe but QEMU's Pi 3 PM/ASB model does not
fully implement. The physical image and original DTB remain unchanged.

The VM uses snapshot mode, so guest writes are discarded on stop. The serial
console is retained at `deploy/rpi/.vm/console.log`.

Useful commands:

```bash
./deploy/rpi/pi-vm ssh
./deploy/rpi/pi-vm logs
./deploy/rpi/pi-vm deploy
./deploy/rpi/pi-vm stop
```

`deploy` is only for MPFC application-source iteration. OCCID, HiveLink, or other
Python dependency changes require an image rebuild so the installed environment
remains authoritative.

## Guest services

Core image:

```text
ssh.service
mosquitto.service
mpfc-runtime-config.service
mpfc.service
```

WFB-NG image additionally enables:

```text
wifibroadcast@drone.service
```

Useful diagnostics:

```bash
journalctl -fu mpfc.service
mosquitto_sub -h 127.0.0.1 -p 1883 -v -t 'mpfc/#'
journalctl -fu wifibroadcast@drone.service
```

## Validation boundary

The VM validates the ARM64 userspace, cross-staged Python/MAVSDK runtime,
systemd services, private local IPC, installed OCCID/HiveLink packages,
independent IP networking, and PX4 interaction. It does not validate physical
Zero 2 W GPIO, RF, electrical behavior, USB WiFi injection, driver RF behavior,
or serial timing. WFB-NG radio acceptance therefore requires the physical Pi and
selected radio hardware.
