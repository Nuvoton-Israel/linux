# Nuvoton OBMF on OpenBMC

This directory documents the OBMF-ICP USB class driver in the Nuvoton Linux kernel. The OBMF-related `linux-nuvoton` patches are already integrated in the [NPCM-6.1-OBMF-dev](https://github.com/Nuvoton-Israel/linux/tree/NPCM-6.1-OBMF-dev) and [NPCM-6.12-OBMF-dev](https://github.com/Nuvoton-Israel/linux/tree/NPCM-6.12-OBMF-dev) branches; an OpenBMC build should use the matching kernel branch.

This README covers bringing up the environment, building the image, identifying the exposed devices on the BMC, and running basic channel tests.

## Table of Contents

- [What This Repository Provides](#what-this-repository-provides)
- [Kernel Config Matrix](#kernel-config-matrix)
- [Host Build Environment](#host-build-environment)
- [How to Integrate and Build](#how-to-integrate-and-build)
- [USB Topology and Hierarchical DTS Design](#usb-topology-and-hierarchical-dts-design)
  - [1. Where the DTS lives and how it is applied](#1-where-the-dts-lives-and-how-it-is-applied)
  - [2. How the driver matches a USB device to a DT node](#2-how-the-driver-matches-a-usb-device-to-a-dt-node)
  - [3. Determining the port numbers](#3-determining-the-port-numbers)
  - [4. Case A: device directly on the root port (no hub)](#4-case-a-device-directly-on-the-root-port-no-hub)
  - [5. Case B: device behind one hub](#5-case-b-device-behind-one-hub)
  - [6. Case C: device behind two hubs](#6-case-c-device-behind-two-hubs)
  - [7. Rules and pitfalls](#7-rules-and-pitfalls)
  - [8. Verifying the topology](#8-verifying-the-topology)
- [Channel Type to Linux Device Mapping](#channel-type-to-linux-device-mapping)
- [What You Should See on the BMC](#what-you-should-see-on-the-bmc)
- [Channel Test Guide](#channel-test-guide)
  - [1. Base bring-up test](#1-base-bring-up-test)
  - [2. GPIO test](#2-gpio-test)
  - [3. IPMI channel test](#3-ipmi-channel-test)
  - [4. SPI flash test](#4-spi-flash-test)
  - [5. Memory Device test](#5-memory-device-test)
  - [6. UART / Serial test](#6-uart--serial-test)
  - [7. Port 80 / POST code test](#7-port-80--post-code-test)
  - [8. I2C Controller test](#8-i2c-controller-test)
  - [9. I2C Target test](#9-i2c-target-test)
  - [10. I3C Controller test](#10-i3c-controller-test)

## What This Repository Provides

The two OBMF development branches provide the kernel-side implementation:

- The OBMF transport and discovery implementation under `drivers/usb/class/obmf/`, including MMIO, IO, GPIO, Serial, IPMI, SPI, Memory Device, I2C, and I3C channel handlers.
- `CONFIG_USB_OBMF` and channel switches in `drivers/usb/class/Kconfig`, with build integration in `drivers/usb/class/Makefile` and `drivers/usb/class/obmf/Makefile`.
- Board-specific USB topology and resource descriptions in the kernel Device Tree when needed.

No separate OBMF driver patch bundle, `linux-nuvoton/` directory, or bbappend is shipped alongside this README. The OpenBMC layer still needs to select the matching kernel branch and enable the required kernel options for its board.

## Kernel Config Matrix

Check the effective kernel configuration before following the [Channel Test Guide](#channel-test-guide). The OBMF channel switches default to `y` only when their dependencies and `CONFIG_USB_OBMF` are enabled; your OpenBMC machine configuration may override them. A disabled handler does not prevent channel discovery, but it does prevent that channel's Linux device from being registered.

| Feature you want to use | Required kernel config | Configuration note |
|---|---|---|
| OBMF core, discovery, MMIO and IO channels | `CONFIG_USB_OBMF=y` | Select built-in for boot-time bring-up, or `m` for a module. |
| GPIO channel | `CONFIG_USB_OBMF_GPIO=y`, `CONFIG_GPIOLIB=y` | OBMF switch defaults to `y` if dependencies are met. |
| Serial channel | `CONFIG_USB_OBMF_SERIAL=y`, `CONFIG_TTY=y` | OBMF switch defaults to `y` if dependencies are met. |
| IPMI channel | `CONFIG_USB_OBMF_IPMI=y` | OBMF switch defaults to `y` if core is enabled. |
| SPI Controller channel | `CONFIG_USB_OBMF_SPI=y`, `CONFIG_MTD=y` | OBMF switch defaults to `y` if dependencies are met. |
| Memory Device channel | `CONFIG_USB_OBMF_MEM_DEV=y`, `CONFIG_MTD=y` | OBMF switch defaults to `y` if dependencies are met. |
| eSPI-backed flash behind the Memory Device channel | `CONFIG_SPI_NPCM_FLASH=y` | Enable separately if the board uses NPCM eSPI flash as the backing MTD device. |
| I2C Controller channel | `CONFIG_USB_OBMF_I2C=y`, `CONFIG_I2C=y` | OBMF switch defaults to `y` if dependencies are met. |
| I2C Target channel | `CONFIG_USB_OBMF_I2C=y`, `CONFIG_I2C_SLAVE=y` | I2C target support is configured separately. |
| I3C Controller channel | `CONFIG_USB_OBMF_I3C=y`, `CONFIG_I3C=y` | OBMF switch defaults to `y` if dependencies are met. |
| MCTP over the OBMF I2C channel | `CONFIG_MCTP=y`, `CONFIG_MCTP_TRANSPORT_I2C=y` | Enable separately for MCTP tests. |
| MCTP over the OBMF I3C channel | `CONFIG_MCTP=y`, `CONFIG_MCTP_TRANSPORT_I3C=y` | Enable separately for MCTP tests. |
| USB packet tracing with `usbmon` | `CONFIG_USB_MON=y`, `CONFIG_DEBUG_FS=y` | Enable separately for tracing. |

## Host Build Environment

Prepare a standard OpenBMC build environment on the host:

- Linux build host with the packages required by OpenBMC/Yocto
- An OpenBMC source tree and machine configuration for your target NPCM board
- A `linux-nuvoton` kernel recipe (and, if needed, your own layer/recipe append) that fetches the matching `NPCM-6.1-OBMF-dev` or `NPCM-6.12-OBMF-dev` branch rather than an unrelated kernel revision
- Access to the target BMC and the OBMF-capable USB-attached platform

## How to Integrate and Build

### 1. Select the matching Nuvoton kernel branch

In the OpenBMC `linux-nuvoton` recipe or your machine layer, point the kernel source at [NPCM-6.1-OBMF-dev](https://github.com/Nuvoton-Israel/linux/tree/NPCM-6.1-OBMF-dev) for a 6.1-based build or [NPCM-6.12-OBMF-dev](https://github.com/Nuvoton-Israel/linux/tree/NPCM-6.12-OBMF-dev) for a 6.12-based build. Pin the chosen revision (`SRCREV`) in your recipe for reproducible builds. Configure the kernel options from the [Kernel Config Matrix](#kernel-config-matrix) in your layer or kernel configuration workflow, and adapt the board DTS to the physical USB wiring below.

### 2. Add your layer to the OpenBMC build

Make sure your layer is present in `conf/bblayers.conf`.

### 3. Start the OpenBMC build environment

```bash
cd /path/to/openbmc
. setup <machine>
```

### 4. Build the kernel or full image

Build only the kernel when you want faster iteration:

```bash
bitbake virtual/kernel
```

Build the complete firmware image for deployment:

```bash
bitbake obmc-phosphor-image
```

### 5. Confirm the selected kernel source and configuration

```bash
bitbake -e virtual/kernel | grep -E '^(SRC_URI|SRCREV|KBRANCH|B)='
kernel_build_dir=$(bitbake -e virtual/kernel | sed -n 's/^B="\(.*\)"/\1/p')
grep -E '^CONFIG_USB_OBMF(=|_)' "$kernel_build_dir/.config"
```

Check that the resolved source revision is from the intended OBMF branch and that the final kernel `.config` enables the core and the handlers you need. If `B` does not point to a populated build directory yet, run the kernel build first.

### 6. Deploy and boot the BMC image

Use your normal board-specific OpenBMC flashing method.

### 7. Connect the OBMF-capable device

After the BMC is up, connect the OBMF-capable USB device or board to the BMC USB host port used for validation.

## USB Topology and Hierarchical DTS Design

An OBMF device does not have to sit directly on a BMC USB root port. It may be attached behind one or more USB hubs (for example, a hub board that fans out a single BMC USB host port to several SMC targets). This section explains how the driver locates the Device Tree node for a given USB device, and how to write the DTS accordingly for direct, one-hub, and two-hub topologies. These are board DTS examples, not patches to copy from this directory.

### 1. Where the DTS lives and how it is applied

The examples below belong in the DTS for your BMC board (for example, `arch/arm64/boot/dts/nuvoton/nuvoton-npcm845-evb.dts` on an NPCM845 EVB). Inspect the DTS supplied by your selected OBMF branch first. If its topology differs from your board, change the board DTS in your kernel source or carry a **board-specific** DTS patch in your OpenBMC layer. Describe exactly one physical topology per host port; do not combine the direct-device and hub examples under the same `reg`.

After changing the DTS in your OpenBMC layer, rebuild the kernel (force recipe tasks as needed for your layer workflow):

```bash
bitbake virtual/kernel
```

### 2. How the driver matches a USB device to a DT node

The driver does not require any special "USB device" compatible string. It walks the physical USB port chain and mirrors it in the DT:

1. Starting from the OBMF USB device, it walks `udev->parent` up to the root hub, recording each `portnum` it passes through.
2. It starts from the host controller's own OF node (e.g. `&ehci1`) and descends the DT one level per USB tier, at each level picking the child node whose `reg` equals the recorded port number.
3. The final node reached is the DT node for that USB device; its channel sub-nodes are then matched the same way, by `reg` equal to the OBMF `channel_id`.

This means:

- The DTS structure must mirror the physical USB wiring one-for-one: one DT node per USB tier (root port, hub port, hub port, ...).
- Node **names** (`smc@1`, `hub@1`, ...) are for readability only; only the `reg` value is used for matching.
- Every node that has USB/channel children needs `#address-cells = <1>; #size-cells = <0>;`, including the host-controller node itself (`&ehci1`).

### 3. Determining the port numbers

Before writing the DTS, find the real USB path of the target device:

```bash
lsusb -t
ls /sys/bus/usb/devices/
```

The kernel names devices using the `bus-port.port.port` convention, e.g. `1-1.2.1`. Each dash- or dot-separated number after the bus number is one tier, and maps to one `reg` value at one DT nesting level. These examples use the same port numbers as the dmesg captures and DTS samples later in this section:

| USB path | Meaning | DTS nesting |
|---|---|---|
| `1-1` | Directly on root port 1 (Case A) | `&ehci1 { smc@1 { reg = <1>; ... } }` |
| `1-1.1` | Root port 1 -> hub port 1 (Case B, SMC1) | `&ehci1 { hub@1 { reg = <1>; smc@1 { reg = <1>; ... } } }` |
| `1-1.2.1` | Root port 1 -> hub port 2 -> hub port 1 (Case C, SMC1) | `&ehci1 { hub@1 { reg = <1>; hub@2 { reg = <2>; smc@1 { reg = <1>; ... } } } }` |

### 4. Case A: device directly on the root port (no hub)

Topology: `BMC(ehci1) -- smc@1 (SMC1, 1-1)`

```dts
&ehci1 {
	status = "okay";
	#address-cells = <1>;
	#size-cells = <0>;

	/* SMC1: OBMF device directly on root port 1 (USB path 1-1) */
	smc@1 {
		reg = <1>;
		#address-cells = <1>;
		#size-cells = <0>;

		mem-dev@4 {
			reg = <4>;
			nuvoton,mtd-name = "npcm-espi-flash";
		};

		smc-gpio@5 {
			reg = <5>;
			/* gpio-line-names = "SMC1GPIO0", ... ; */
		};
	};
};
```

### 5. Case B: device behind one hub

Topology:

```text
BMC(ehci1) -- hub@1 -- port1: smc@1 (SMC1, 1-1.1)
                    \- port2: smc@2 (SMC2, 1-1.2)
```

```dts
&ehci1 {
	status = "okay";
	#address-cells = <1>;
	#size-cells = <0>;

	/* USB hub at root port 1 */
	hub@1 {
		reg = <1>;
		#address-cells = <1>;
		#size-cells = <0>;

		/* SMC1: OBMF device at hub port 1 (USB path 1-1.1) */
		smc@1 {
			reg = <1>;
			#address-cells = <1>;
			#size-cells = <0>;

			mem-dev@4 {
				reg = <4>;
				nuvoton,mtd-name = "npcm-espi-flash";
			};
		};

		/* SMC2: OBMF device at hub port 2 (USB path 1-1.2) */
		smc@2 {
			reg = <2>;
			#address-cells = <1>;
			#size-cells = <0>;

			mem-dev@4 {
				reg = <4>;
				nuvoton,mtd-name = "npcm-espi-flash1";
			};
		};
	};
};
```

Note that each OBMF device needs its own unique resources where the SoC side is shared, e.g. distinct `nuvoton,mtd-name` values and a separate `memory-region` per device on `&espi_flash`:

```dts
&espi_flash {
	memory-region = <&espi_flash_memory>, <&espi_flash_memory1>;
	status = "okay";
};
```

### 6. Case C: device behind two hubs

Topology:

```text
BMC(ehci1) -- hub@1 -- hub@2 -- port1: smc@1 (SMC1, 1-1.2.1)
                             \- port3: smc@3 (SMC2, 1-1.2.3)
```

The rule is simply "one nested hub node per USB tier". Extending the same pattern to a second hub:

```dts
&ehci1 {
	status = "okay";
	#address-cells = <1>;
	#size-cells = <0>;

	/* First-tier USB hub at root port 1 */
	hub@1 {
		reg = <1>;
		#address-cells = <1>;
		#size-cells = <0>;

		/* Second-tier USB hub at hub@1 port 2 */
		hub@2 {
			reg = <2>;
			#address-cells = <1>;
			#size-cells = <0>;

			/* SMC1: OBMF device at hub@2 port 1 (USB path 1-1.2.1) */
			smc@1 {
				reg = <1>;
				#address-cells = <1>;
				#size-cells = <0>;

				mem-dev@4 {
					reg = <4>;
					nuvoton,mtd-name = "npcm-espi-flash";
				};
			};

			/* SMC2: OBMF device at hub@2 port 3 (USB path 1-1.2.3) */
			smc@3 {
				reg = <3>;
				#address-cells = <1>;
				#size-cells = <0>;

				mem-dev@4 {
					reg = <4>;
					nuvoton,mtd-name = "npcm-espi-flash1";
				};
			};
		};
	};
};
```

A third hub simply adds one more nested `hubN@<port>` level following the same pattern, up to the USB spec limit of 7 tiers.

### 7. Rules and pitfalls

- Add one DT node per physical USB tier; do not skip levels even if a hub itself needs no configuration.
- Every node with children (host controller, hub, OBMF device) needs `#address-cells = <1>; #size-cells = <0>;`.
- `reg` on a hub/device node is the 1-based port number on its *immediate parent*, not a global USB address and not the OBMF `device_index`.
- Node names (`hub@1`, `smc@1`, ...) are cosmetic; only `reg` is matched against the live USB topology.
- OBMF channel sub-nodes (`mem-dev@4`, `smc-gpio@5`, `i2c-ch@10`, ...) must be children of the OBMF device node (`smc@N`), never of a `hub@N` node.
- `device_index` is derived from the OBMF device node's own `reg` (`device_index = reg - 1`) and is pinned with `ida_alloc_range()`. Two OBMF devices that reuse the same port number under different hubs (e.g. `hub@1/smc@1` and `hub@2/smc@1`) collide: the second probe fails with `-ENOSPC`. Use distinct final port numbers for stable indices. Dynamic index allocation only applies when no matching DT node exists or its `reg` is invalid.
- The port-chain buffer supports up to 8 entries; the USB specification limits topology depth to 7 tiers total, not 7 hubs plus a device.

### 8. Verifying the topology

```bash
lsusb -t
ls -l /sys/bus/usb/devices/1-1.2.1/of_node
dmesg | grep -i obmf
```

- `lsusb -t` / `/sys/bus/usb/devices/` confirm the real port path matches what the DTS encodes.
- The `of_node` symlink under the device's `/sys/bus/usb/devices/<path>/` confirms that USB core associated that USB device with a DT node.
- `dmesg` should show `OBMF-ICP obmf<device> ready: ...` without a `"DT node has no valid 'reg'; falling back to dynamic index"` warning or an index-allocation failure.

The USB path reported by the kernel (`1-1`, `1-1.1`, ...) is exactly the path used for DT matching, so it is the fastest way to confirm the topology before checking the DTS. Example from `dmesg | grep -i obmf` with the OBMF device directly on the root port (Case A, no hub):

```text
obmf 1-1:1.1: OBMF-ICP device detected
obmf 1-1:1.1: OBMF functional: wr=512 rd=512 wr_int=0 rd_int=0 ver=0x0080
obmf 1-1:1.1: OBMF_VER = 0x0100
obmf 1-1:1.1: Vendor=0xffff Device=0x0001 Role=2
obmf 1-1:1.1: Device Name: OpenBIC-OBMF
obmf 1-1:1.1: NUM_OF_CHANNELS = 18
obmf 1-1:1.1: ch1: type=0x01 name="MMIO" config_offset=0x00000078 config_size=4
...
obmf 1-1:1.1: OBMF-ICP obmf0 ready: 18 channels, ver 0x0080
```

Example with a USB hub inserted between the BMC and the SMC (Case B); note the extra `hub 1-1:1.0` line and the OBMF device now enumerating at `1-1.1` (hub port 1) instead of `1-1`:

```text
usb 1-1: new high-speed USB device number 3 using npcm7xx-ehci
usb 1-1: New USB device found, idVendor=05e3, idProduct=0610, bcdDevice= 6.63
usb 1-1: Product: USB2.1 Hub
usb 1-1: Manufacturer: GenesysLogic
hub 1-1:1.0: USB hub found
hub 1-1:1.0: 4 ports detected
usb 1-1.1: new high-speed USB device number 4 using npcm7xx-ehci
usb 1-1.1: New USB device found, idVendor=1d6b, idProduct=0104, bcdDevice= 2.06
usb 1-1.1: Product: NPCM400 USBD
usb 1-1.1: Manufacturer: ZEPHYR
obmf 1-1.1:1.1: OBMF-ICP device detected
obmf 1-1.1:1.1: OBMF functional: wr=512 rd=512 wr_int=0 rd_int=0 ver=0x0080
obmf 1-1.1:1.1: OBMF_VER = 0x0100
obmf 1-1.1:1.1: NUM_OF_CHANNELS = 18
obmf 1-1.1:1.1: ch1: type=0x01 name="MMIO" config_offset=0x00000078 config_size=4
...
obmf 1-1.1:1.1: OBMF-ICP obmf0 ready: 18 channels, ver 0x0080
```
Example with two hubs in the chain (Case C); the OBMF device now enumerates at `1-1.2.1` (root port 1 -> hub port 2 -> hub port 1), and the discovery sequence completes with the `ready` line:

```text
obmf 1-1.2.1:1.1: OBMF-ICP device detected
obmf 1-1.2.1:1.1: OBMF functional: wr=512 rd=512 wr_int=0 rd_int=0 ver=0x0080
obmf 1-1.2.1:1.1: OBMF_VER = 0x0100
obmf 1-1.2.1:1.1: Vendor=0xffff Device=0x0001 Role=2
obmf 1-1.2.1:1.1: Device Name: OpenBIC-OBMF
obmf 1-1.2.1:1.1: NUM_OF_CHANNELS = 18
obmf 1-1.2.1:1.1: ch1: type=0x01 name="MMIO" config_offset=0x00000078 config_size=4
...
obmf 1-1.2.1:1.1: OBMF-ICP obmf0 ready: 18 channels, ver 0x0080
```
If the SMC is moved to a different hub port (or a second hub tier is added), the trailing segment(s) of the `obmf <path>:1.1` prefix change accordingly (e.g. `1-1.2`, `1-1.2.1`), and the matching `smc@N` / `hub@N` `reg` values in the DTS must be updated to match.

## Channel Type to Linux Device Mapping

The kernel driver discovers Channel 0 first, then registers each discovered channel into the corresponding Linux subsystem.

| Channel Type | OBMF Type Code | What it means | Linux representation on the BMC |
|---|---:|---|---|
| Config / Discovery | `0x00` | Mandatory Channel 0 for discovery and channel config reads | No standalone device node; visible through kernel logs and OBMF sysfs hierarchy |
| MMIO | `0x01` | Generic MMIO channel | `/dev/obmf<device>-mmio-<channel>` miscdevice |
| GPIO | `0x02` | Optimized GPIO channel | `gpiochipN` device with label like `obmf<device>-gpio-ch<channel>` |
| Serial | `0x03` | Optimized UART/serial channel | `/dev/ttyOBMF*` |
| I2C Controller | `0x04` | Optimized I2C master/controller channel | `/dev/i2c-*` adapter named `OBMF<device> I2C ch<channel>` |
| I2C Target | `0x05` | Protocol-defined target mode | Rrequires `obmf,paired-controller` pointing at the I2C Controller adapter node |
| I3C Controller | `0x06` | Optimized I3C controller channel | I3C bus exposed to the MCTP stack as an `mctpi3c*` link |
| IPMI | `0x07` | Optimized IPMI/SSIF bridge | `/dev/ipmi-obmf<device>-<channel>` miscdevice |
| SPI Controller | `0x08` | Optimized SPI controller channel | Host-initiated, virtual MTD device named by `nuvoton,mtd-name` (for example `npcm-spi-flash`) |
| IO | `0x09` | Optimized IO channel | `/dev/obmf<device>-io-<channel>` miscdevice |
| Memory Device | `0x0A` | Byte-addressable device-initiated memory channel | MTD device named by `nuvoton,mtd-name` (default `npcm-espi-flash`), linked as `mtd` under the channel's sysfs directory |
| OEM | `0xF8-0xFF` | OEM-specific extension space | Framework exists, but device shape depends on OEM handler implementation |

Notes:

- GPIO, Serial, IPMI, and IO are the most visible channels because they create direct Linux device nodes or subsystem devices.
- SPI (`0x08`) is always host-initiated: the BMC forwards SPI NOR opcodes to the SMC over a virtual MTD device backed by `nuvoton,mtd-name`. Device-initiated byte-addressable access now lives on the separate Memory Device channel (`0x0A`).

## What You Should See on the BMC

Once the BMC boots with the patched kernel and the OBMF device is attached, you should be able to confirm OBMF bring-up in several places.

### Kernel logs

Check discovery and registration messages:

```bash
dmesg | grep -i obmf
```

Typical messages include:

- `OBMF-ICP device detected`
- `OBMF_VER = 0x....`
- `Vendor=0x.... Device=0x.... Role=...`
- `Device Name: ...`
- `NUM_OF_CHANNELS = ...`
- `chX: type=0x.. name="..." config_offset=... config_size=...`
- `chX: registered GPIO chip (...)`
- `chX: registered /dev/ipmi-obmf...`
- `chX: registered SPI Controller (MTD: <mtd-name>)`
- `chX: registered Memory Device (MTD: <mtd-name>, <size> bytes, erasesize <n>)`
- `OBMF-ICP obmf<device> ready: <count> channels, ver 0x....`

### USB driver binding

```bash
ls /sys/bus/usb/drivers/obmf
```

### OBMF sysfs hierarchy

The OBMF driver creates a per-channel sysfs tree under the USB interface device:

```bash
find /sys/devices -path '*/obmf/channel/*' | sort
```

You should see a structure similar to:

```text
.../usbX/Y-Z/obmf/channel/1
.../usbX/Y-Z/obmf/channel/2
.../usbX/Y-Z/obmf/channel/3
```

Depending on channel type, the channel directory may contain links such as:

- `device` for GPIO, I2C, Serial, and IPMI-backed Linux devices
- `mmio` for MMIO channel miscdevices
- `mtd` for the backing MTD device used by the Memory Device channel

### Device nodes and subsystem devices

Useful checks:

```bash
ls /dev | grep -E 'obmf|ttyOBMF|ipmi-obmf'
gpiodetect | grep -i obmf
gpioinfo
cat /proc/mtd
i2cdetect -l
```

## Channel Test Guide

This section focuses on practical sanity tests that match the current kernel interfaces. The exact test approach may vary based on your firmware's OBMF implementation and the channels it exposes. The goal is to confirm that the BMC can see the expected devices, that basic read/write operations work, and that device-initiated events are visible on the BMC side.

All tests below share the same physical shape; only the channel type and which side originates the traffic differ:

```text
                +----------------------------+
  Remote side   |             SMC            |   BMC side (this guide)
                |  (OpenBIC / OBMF device)   |
 Host / eSPI ---+  GPIO / UART / I2C / I3C / +--- USB ---> BMC (obmf driver)
 TEMP IC     ---+  SPI / KCS / Port80 pins   |             |
 Peer BMC    ---+                            |             +-- /dev, gpiochip, mtd, i2c-N, ...
                +----------------------------+
```

Unless a test says otherwise, run every command on the **BMC**; "remote / host side" actions must be triggered on the SMC or on whatever platform it is wired to.

| Test | Hardware topology | Who initiates | BMC-side interface |
|---|---|---|---|
| 1. Base bring-up | BMC — (USB) — SMC | BMC | `dmesg`, sysfs |
| 2. GPIO | GPIO pins — SMC — (USB) — BMC | either side | `gpiochipN` |
| 3. IPMI | Host — (eSPI / KCS) — SMC — (USB) — BMC | host | `/dev/ipmi-obmf*` |
| 4. SPI flash | BMC — (USB) — SMC — (SPI) — NOR flash | BMC | MTD (`npcm-spi-flash`) |
| 5. Memory Device | Host — (eSPI flash) — SMC — (USB) — BMC | host | MTD (`npcm-espi-flash`) |
| 6. UART / Serial | UART peer — SMC — (USB) — BMC | either side | `/dev/ttyOBMF*` |
| 7. Port 80 / POST | Host — (eSPI IO) — SMC — (USB) — BMC | host | `/dev/obmf*-io-*` |
| 8. I2C Controller | TEMP IC — (I2C) — SMC — (USB) — BMC | BMC | `/dev/i2c-N` |
| 9. I2C Target | BMC_A — (I2C) — SMC — (USB) — BMC_B | BMC_A | `/dev/i2c-N` + `mctpi2cN` |
| 10. I3C Controller | BMC — (USB) — SMC1 — (I3C) — SMC2 | BMC | `i3c-N` + `mctpi3cN` |

### 1. Base bring-up test

Hardware topology: BMC — (USB) — SMC

Prerequisites:

- `CONFIG_USB_OBMF=y` (see [Kernel Config Matrix](#kernel-config-matrix))
- A DTS topology patch matching the physical USB wiring (see [1. Where the DTS lives and how it is applied](#1-where-the-dts-lives-and-how-it-is-applied))

Run these checks first:

```bash
dmesg | grep -i obmf
ls /sys/bus/usb/drivers/obmf
ls /dev | grep -E 'obmf|ttyOBMF|ipmi-obmf'
```

Pass criteria:

- The OBMF USB interface is detected and bound to the `obmf` driver.
- The driver prints discovery information and the final `ready` line.
- At least the expected device nodes for your enabled channels appear.

### 2. GPIO test

Hardware topology: GPIO pins — SMC — (USB) — BMC

Prerequisites:

- `CONFIG_USB_OBMF_GPIO=y`, `CONFIG_GPIOLIB=y`

The GPIO channel is exposed as a normal Linux GPIO chip. Use `libgpiod` tools.

Find the chip:

```bash
gpiodetect | grep -i obmf
gpioinfo gpiochipN
```

If the firmware provides named GPIO lines, they should appear in `gpioinfo` because the driver imports GPIO names from the OBMF config space.

Read an input line:

```bash
gpioget gpiochipN <offset>
```

Write an output line:

```bash
gpioset gpiochipN <offset>=1
gpioset gpiochipN <offset>=0
```

Suggested checks on BMC:

- The chip label matches `obmf<device>-gpio-ch<channel>`.
- Line names, if defined by firmware, appear correctly.
- Reads reflect the expected input state.
- Writes succeed only on output-capable lines.
- Interrupt-capable lines can be monitored with `gpiomon` if firmware exposes IRQ notifications.

Example IRQ monitor:

```bash
gpiomon --rising-edge --falling-edge gpiochipN <offset>
```

### 3. IPMI channel test

Hardware topology: Host — (eSPI / KCS) — SMC — (USB) — BMC

Prerequisites:

- `CONFIG_USB_OBMF_IPMI=y`
- `CONFIG_USB_MON=y` and `CONFIG_DEBUG_FS=y` if you want to trace the traffic with `usbmon`

On the BMC side, the IPMI optimized channel is exposed as an OBMF miscdevice, typically `/dev/ipmi-obmf<device>-<channel>`. Treat it as a low-level bridge endpoint, not the host IPMI stack device.

Basic BMC-side procedure:

1. Confirm channel discovery and node creation.

```bash
dmesg | grep -i -E 'obmf|ipmi'
ls /dev/ipmi-obmf*
ls -al /sys/bus/usb/devices/*/obmf/channel/*
```

2. Start your responder or bridge process on the OBMF IPMI node. One practical example is:

```bash
/usr/libexec/kcsbridged -c ipmi-obmf<device>-<channel> &
```

3. Trigger IPMI traffic from the remote platform side (for example, an eSPI KCS command such as Get Device ID), then observe BMC-side behavior.

Suggested checks on BMC:

- The `/dev/ipmi-obmf*` node exists and remains stable during traffic.
- Incoming requests are observable from the BMC endpoint if enabled CONFIG_USB_MON.

Protocol reminder for userspace handlers:

- `read()` returns `[NetFn/LUN, IPMI_Cmd, Data...]`
- `write()` expects `[NetFn/LUN, IPMI_Cmd, CC, Data...]`

### 4. SPI flash test

Hardware topology: BMC — (USB) — SMC — (SPI) — NOR flash

Prerequisites:

- `CONFIG_USB_OBMF_SPI=y`, `CONFIG_MTD=y`

> **Warning:** the commands in this section write to raw MTD devices. Writing to the wrong `/dev/mtdX` will destroy the BMC's own U-Boot, environment or rootfs and can brick the board. Always resolve the number from `cat /proc/mtd` by the `nuvoton,mtd-name` you configured, and prefer the by-name path (`/dev/mtd/<name>`) over a hard-coded number.

The SPI Controller channel is always host-initiated: the BMC registers a virtual MTD device from the `nuvoton,flash-size` / `nuvoton,flash-erasesize` DTS properties, and forwards each MTD op as a single SPI NOR opcode (READ / PP / SE / BE) to the SMC, which drives the real SPI bus.

Notes:

- TX is not fragmented on the BMC side; max transfer unit is 448 bytes (`OBMF_SPI_MAX_XFER`).
- The SMC owns the full bus sequence (CS assert -> WREN before PP/Erase -> WIP polling -> CS deassert). The BMC does not issue WREN / RDSR and uses no local RAM or flash backing.

DTS example:

```dts
spi@17 {
    reg = <17>;
    #address-cells = <1>;
    #size-cells = <0>;

    nuvoton,flash-size = <0x4000000>;
    nuvoton,flash-erasesize = <0x1000>;
    nuvoton,flash-cs = <0x1>;
    nuvoton,mtd-name = "npcm-spi-flash";
};
```

The SMC-side SPI flash is currently fixed to `spi_fiu0_cs1`.

1. Verify that the MTD device is created.

```bash
dmesg | grep -i -E 'obmf|spi|mtd|npcm-spi-flash'
cat /proc/mtd
```

2. Run an erase / write / read-back test with `flashcp`.

Resolve the MTD number for your configured `nuvoton,mtd-name` first, and double-check it before writing:

```bash
cat /proc/mtd | grep npcm-spi-flash    # note the mtdN on the left
```

```bash
dd if=/dev/urandom of=/tmp/test.bin bs=1M count=5
flashcp -v /tmp/test.bin /dev/mtdN   # replace N with the matching MTD number
```

Suggested checks on BMC:

- The configured MTD node (for example `npcm-spi-flash`) appears in `/proc/mtd`.
- `flashcp -v` completes with a successful verify pass.

### 5. Memory Device test

Hardware topology: Host — (eSPI flash) — SMC — (USB) — BMC

Prerequisites:

- `CONFIG_USB_OBMF_MEM_DEV=y`, `CONFIG_MTD=y`
- `CONFIG_SPI_NPCM_FLASH=y` for the eSPI-backed MTD device (enable it in your board's kernel configuration)

> **Warning:** the commands in this section read/write raw MTD devices. Writing to the wrong `/dev/mtdX` will destroy the BMC's own U-Boot, environment or rootfs and can brick the board. Always resolve the number from `cat /proc/mtd` by the `nuvoton,mtd-name` you configured, and prefer the by-name path (`/dev/mtd/<name>`) over a hard-coded number.

The Memory Device channel (OBMF-ICP v1.0.0 RC1 §4.12, Channel Type `0x0A`) is always device-initiated (Producer/Responder): the SMC pushes Memory Read/Write/Erase commands, and the BMC maps them onto the MTD device named by `nuvoton,mtd-name` (default `npcm-espi-flash`). This channel replaces the SPI Controller channel's former `espi` mode.

Request format (device -> BMC):

| Field | Description |
|---|---|
| Command | `0`=Read, `1`=Write, `2`=Erase |
| Address (4B LE) | Byte offset into the device address space |
| Data Size (2B LE) | Bytes for Read/Write; KB for Erase (must align to the MTD erase block size) |
| Data | Write payload, Write only |

Channel-specific status codes: `0x40` access denied, `0x41` unsupported size, `0x42` erase not aligned, `0x43` write protected. Capability bits reported in `MEM_DEV_CFG` (`CAPABILITIES` at offset `0x00`): `WRITEABLE` (bit 0) and `NO_ERASE` (bit 1).

DTS example:

```dts
mem-dev@4 {
    reg = <4>;
    nuvoton,mtd-name = "npcm-espi-flash";
};
```

1. Verify that the Memory Device channel is registered and mapped to MTD.

```bash
dmesg | grep -i -E 'obmf|mtd|npcm-espi-flash'
ls -la /sys/bus/usb/devices/*/obmf/channel/*/mtd
cat /proc/mtd
```

2. When the remote host performs read/write/erase operations, observe the resulting data on BMC using hexdump.

```bash
hexdump -C /dev/mtd/npcm-espi-flash
```

Suggested checks on BMC:

- `npcm-espi-flash` (or equivalent mapped MTD node) appears in `/proc/mtd`.
- Hexdump output matches expected data after remote read operations.
- After remote write, hexdump shows the expected updated values in the target region.
- After remote erase, hexdump shows `0xff` fill pattern in the erased region.
- Remote requests that violate size/alignment/write-protect constraints get the matching `0x41`/`0x42`/`0x43` status instead of being silently applied.

### 6. UART / Serial test

Hardware topology: UART peer — SMC — (USB) — BMC

Prerequisites:

- `CONFIG_USB_OBMF_SERIAL=y`, `CONFIG_TTY=y`

On the BMC side, the Serial optimized channel appears as `/dev/ttyOBMF*` and can be used by normal Linux tty tools.

Basic BMC-side procedure:

1. Confirm tty node creation and driver registration.

```bash
dmesg | grep -i -E 'obmf|ttyOBMF|serial'
ls /dev/ttyOBMF*
```

2. Configure the tty and monitor incoming data.

```bash
stty -F /dev/ttyOBMF<device>_<channel> 115200 raw -echo
cat /dev/ttyOBMF<device>_<channel>
```

Suggested checks on BMC:

- `/dev/ttyOBMF*` appears after OBMF discovery and remains available.
- RX path: remote-originated data is visible on BMC.
- TX path: BMC-written data is accepted and forwarded.

### 7. Port 80 / POST code test

Hardware topology: Host — (eSPI IO port 0x80) — SMC — (USB) — BMC

Prerequisites:

- `CONFIG_USB_OBMF=y` (the IO channel is part of the OBMF core; it has no separate config)

On the BMC side, the port 80 / POST code validation is handled through an OBMF IO channel on the BMC side.

Basic BMC-side procedure:

1. Identify IO channel nodes and verify registration.

```bash
dmesg | grep -i -E 'obmf|io|post|port 80'
ls /dev/obmf*-io-*
ls -al /sys/bus/usb/devices/*/obmf/channel/*
```

2. Start a monitor utility on the IO device for POST code capture. A practical example:

```bash
snoopd -v -b 1 -d /dev/obmf<device>-io-<channel> &
snoopd -v -b 4 -d /dev/obmf<device>-io-<channel> &
```

3. Trigger boot progress on the remote platform and observe POST code changes on BMC.

Suggested checks on BMC:

- IO node is present and accessible.
- 1-byte and/or 4-byte POST code streams are captured as expected for your platform format.
- Captured codes change in sequence during remote boot progression.

### 8. I2C Controller test

Hardware topology: TEMP IC — (I2C) — SMC — (USB) — BMC

Prerequisites:

- `CONFIG_USB_OBMF_I2C=y` and `CONFIG_I2C=y`, see [Kernel Config Matrix](#kernel-config-matrix)
- BMC kernel driver and SMC firmware (OpenBIC) with I2C Controller Optimized Channel support applied
- DTS configured with OBMF I2C channel declarations

1. DTS configuration

On the BMC side, add the I2C channel declaration to the DTS. Verify the SMC node includes the `i2c-ch` entries with the correct channel IDs:

```dts
&ehci1 {
    smc@1 {
        reg = <1>;
		#address-cells = <1>;
		#size-cells = <0>;

        i2c-ch@10 {
            reg = <10>; // channel_id = 10 = OBMF I2C channel
            i2c27: i2c {
                #address-cells = <1>;
                #size-cells = <0>;
                tmp100@48 { compatible = "ti,tmp100"; reg = <0x48>; };
            };
        };
    };
};
```

2. Verify OBMF I2C adapter registration

After booting, confirm that the OBMF I2C adapters are registered in the kernel:

```bash
i2cdetect -l | grep -i obmf
```

Expected output:

```
i2c-27  i2c             OBMF0 I2C ch10                          I2C adapter
```

> The adapter number (`27` here) is assigned dynamically by the kernel and depends on how many I2C adapters your board already has. **Your number will differ** — substitute it in every command below, and do not hard-code it in scripts. The `i2c27:` DTS label above is only a phandle name; it does not force the adapter number.

3. Verify I2C device detection on OBMF bus

Wire a TEMP IC (e.g., TMP100 at address `0x48`) to the OBMF I2C bus. Run `i2cdetect` to confirm both devices are visible:

```bash
i2cdetect -y 27
```

Expected output: Devices detected at `0x48` (TEMP IC).

4. TEMP IC read test

Read 2 bytes from the TEMP IC via the OBMF I2C channel:

```bash
i2ctransfer -y -f 27 w1@0x48 0x0 r2
```

Expected: Returns 2 bytes of temperature data (e.g., `0x21 0x80`).

Verify the hwmon sysfs entry is accessible:

```bash
ls /sys/bus/i2c/devices/27-0048/hwmon/hwmonX/
cat /sys/bus/i2c/devices/27-0048/hwmon/hwmonX/temp1_input
```

`temp1_input` returns a valid temperature value (e.g., `33187` = 33.187 °C).

Suggested checks on BMC:

- OBMF I2C adapters appear in `i2cdetect -l` with the correct channel names.
- Target devices are visible at the expected addresses.
- `i2ctransfer` reads return valid data without errors.
- hwmon sysfs entries are accessible and return sensible values.

### 9. I2C Target test

Hardware topology: BMC_A — (Physical I2C) — SMC — (OBMF I2C via USB) — BMC_B

Prerequisites:

- `CONFIG_USB_OBMF_I2C=y` and `CONFIG_I2C_SLAVE=y`, see [Kernel Config Matrix](#kernel-config-matrix)
- `CONFIG_MCTP=y` and `CONFIG_MCTP_TRANSPORT_I2C=y` on both BMCs for the MCTP/PLDM steps
- `mctp` and `pldmtool` available in both images
- BMC kernel driver with I2C Target channel support (`obmf,paired-controller` property)
- Physical I2C wiring: SCL and SDA properly connected between BMC_A and SMC I2C bus

1. DTS configuration

Configure the Device Tree Source on BMC_B with both I2C Controller and I2C Target channel declarations.

```dts
&ehci1 {
    smc@1 {
        reg = <1>;
		#address-cells = <1>;
		#size-cells = <0>;

        // I2C controller
        i2c-ch@10 {
            reg = <10>;
            #address-cells = <1>;
            #size-cells = <0>;

            i2c27: i2c {
                #address-cells = <1>;
                #size-cells = <0>;

                mctp-controller;
                mctp@12 {
                    compatible = "mctp-i2c-controller";
                    reg = <(0x12 | I2C_OWN_SLAVE_ADDRESS)>;
                };
            };
        };
    };
};

// I2C target
i2c-tgt@14 {
    reg = <14>;
    obmf,paired-controller = <&i2c27>;
};
```

Key points:

- `i2c-tgt@<channel_id>` declares the I2C Target channel.
- `obmf,paired-controller` property links the target channel to its corresponding controller adapter.
- Target channel receives incoming I2C write frames from the SMC and delivers them to the paired virtual I2C adapter.

2. Verify OBMF I2C adapter registration

After booting BMC_B, confirm the OBMF I2C adapters are registered:

```bash
i2cdetect -l | grep -i obmf
```

Expected output:

```
i2c-27  i2c             OBMF0 I2C ch10                          I2C adapter
```

> The adapter number (`27`) and therefore the MCTP link name (`mctpi2c27`) are assigned dynamically by the kernel. **Your numbers will differ**; read them from `i2cdetect -l` and `mctp link show` on your own system and substitute them in every command below.

Verify the MCTP I2C interfaces are present:

```bash
mctp link show
```

Expected: `mctpi2c27` interfaces should appear.

3. MCTP network configuration

BMC_A (Physical I2C side):

```bash
# Set up MCTP link if mctpi2c0 is not already up
mctp link set mctpi2c0 net 1 up
mctp addr add 8 dev mctpi2c0

# Setup SMC endpoint ID
busctl call au.com.codeconstruct.MCTP1 \
    /au/com/codeconstruct/mctp1/interfaces/mctpi2c0 \
    au.com.codeconstruct.MCTP.BusOwner1 SetupEndpoint ay 1 <SMC_physaddr>

# Add routes to BMC_B endpoints
mctp route add 22 via mctpi2c0
mctp neigh add 22 dev mctpi2c0 lladdr <SMC_physaddr>
```

BMC_B (OBMF I2C Target side):

```bash
# Remove default EID on physical interface (if present)
mctp addr del 8 dev mctpi2c0

# Configure OBMF I2C target channel 0
mctp link set mctpi2c27 net 1 up
mctp addr add 22 dev mctpi2c27
mctp route add 8 via mctpi2c27
mctp neigh add 8 dev mctpi2c27 lladdr <BMC_A_physaddr>
```

4. Verify MCTP connectivity (BMC_A → BMC_B via SMC)

From BMC_A, verify MCTP connectivity to BMC_B (EID 22) through the SMC bridge:

```bash
pldmtool base gettid -m 22
```

Expected output:

```json
{
    "Response": 1
}
```

This confirms that:
1. BMC_A sends MCTP/SMBus write to SMC physical I2C slave address
2. SMC bridges the message via OBMF I2C Target channel to BMC_B
3. BMC_B responds through OBMF I2C Controller channel back to SMC
4. SMC forwards the response to BMC_A via physical I2C

5. PLDM type and command enumeration

Get PLDM types:

```bash
pldmtool base getpldmtypes -m 22
```

Expected output:

```json
{
    "CompletionCode": "SUCCESS",
    "PLDMTypes": [
        { "PLDM Type": "base", "PLDM Type Code": 0 },
        { "PLDM Type": "platform", "PLDM Type Code": 2 },
        { "PLDM Type": "bios", "PLDM Type Code": 3 },
        { "PLDM Type": "fru", "PLDM Type Code": 4 }
    ]
}
```

6. USB trace verification

Monitor USB traffic to confirm the OBMF I2C Target channel frames are correctly transmitted:

```bash
cat /sys/kernel/debug/usb/usbmon/1u &
```

### 10. I3C Controller test

Hardware topology: BMC (OBMF host) — (USB) — SMC1 (i3c5 controller) — (I3C) — (i3c5 target) SMC2

Prerequisites:

- `CONFIG_USB_OBMF_I3C=y` and `CONFIG_I3C=y`, see [Kernel Config Matrix](#kernel-config-matrix)
- `CONFIG_MCTP=y` and `CONFIG_MCTP_TRANSPORT_I3C=y` for the MCTP/PLDM steps
- `CONFIG_USB_OBMF_I2C=y` as well if the channel reports an associated I2C channel (`SET_ASSOC_I2C`)
- BMC kernel driver with I3C Controller channel support
- DTS configured with an OBMF I3C channel declaration
- SMC1 acting as the I3C controller and SMC2 attached as an I3C target

1. DTS configuration

On the OBMF host side, declare the I3C channel under the SMC node:

```dts
&ehci1 {
    smc@1 { /* Topology: BMC(ehci1) -- smc@1 (SMC1, 1-1) */
        reg = <1>;
        #address-cells = <1>;
        #size-cells = <0>;

        i3c-ch@18 {
            reg = <18>; // channel_id of this I3C channel
            i3c {
                #address-cells = <3>;
                #size-cells = <0>;
                mctp-controller;
            };
        };
    };
};
```

2. Verify I3C channel registration and device node

Confirm the channel came up and note the I3C bus number reported by the driver:

```bash
dmesg | grep -i -E 'obmf|i3c'
```

Typical messages include:

```text
obmf 1-1:1.1: ch18: I3C bus_init() called
obmf 1-1:1.1: ch18 I3C config: pp=12500kHz od=400kHz max_ops=16 payload=256 ibi=128 feat=0x01
obmf 1-1:1.1: ch18: SET_ASSOC_I2C(ch10)
obmf 1-1:1.1: ch18: registered I3C master (bus-2) with assoc I2C
```

Here `bus-2` means the channel was registered as I3C bus 2. List the I3C devices to find the target device node:

```bash
ls /sys/bus/i3c/devices/
```

Expected output:

```text
2-63212345607/ i3c-0/         i3c-1/         i3c-2/
```

`i3c-2` is the OBMF I3C master, and `2-63212345607` is the SMC2 target on that bus (bus number followed by the device PID).

> The bus number (`bus-2` / `i3c-2`) and the derived MCTP link name (`mctpi3c2`) are assigned dynamically, and the PID comes from the SMC2 firmware. **Your numbers will differ** — take them from your own `dmesg`, `ls /sys/bus/i3c/devices/` and `mctp link show` output before running the steps below.

3. Bring up the MCTP I3C interface

After booting, the OBMF I3C channel is exposed as an MCTP I3C link (for example `mctpi3c2`):

```bash
mctp link show
mctp link set mctpi3c2 up
```

4. MCTP network configuration

Assign a local EID and add the route and neighbour entry for the remote endpoint on SMC2.
The `0x06:32:12:34:56:07` is the PID (link-layer address) of the SMC2 I3C target found in step 2 (`2-63212345607`):

```bash
mctp addr add 8 dev mctpi3c2
mctp route add 10 via mctpi3c2
mctp neigh add 10 dev mctpi3c2 lladdr 0x06:32:12:34:56:07
```

5. Learn the remote endpoint

```bash
busctl call au.com.codeconstruct.MCTP1 \
    /au/com/codeconstruct/mctp1/interfaces/mctpi3c2 \
    au.com.codeconstruct.MCTP.BusOwner1 LearnEndpoint ay 6 0x06 0x32 0x12 0x34 0x56 0x07
```

Expected output:

```
yisb 10 1 "/au/com/codeconstruct/mctp1/networks/1/endpoints/10" true
```

6. Verify MCTP/PLDM connectivity over the I3C channel

```bash
pldmtool base GetTID -m 10
```

Expected output:

```json
{
    "Response": 134
}
```

Suggested checks on BMC:

- The driver log reports `registered I3C master (bus-N)` and the matching `i3c-N` entry exists under `/sys/bus/i3c/devices/`.
- The remote target appears as an `<bus>-<PID>` device node under `/sys/bus/i3c/devices/`.
- The OBMF I3C channel appears as an `mctpi3c*` link in `mctp link show`.
- `LearnEndpoint` returns the assigned EID and a valid endpoint object path.
- `pldmtool base GetTID` returns a valid TID from the remote endpoint on SMC2.
