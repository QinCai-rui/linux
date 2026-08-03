# Allwinner/Radxa A733 BSP for Linux 6.18

## Build Instructions
This repo has already simplified the build process.

Please download the official Linux 6.18.y code from tar or git to a parallel folder `linux-6.18.y`. And follow the general steps here:
```sh
git clone https://github.com/alexcaoys/allwinner-bsp.git -b linux-6.18.y --single-branch allwinner-bsp
cd linux-6.18.y
ln -s ../allwinner-bsp/ ./bsp
git apply ./bsp/patches/*.patch
cp ./bsp/configs/linux-6.18/*.dts* arch/arm64/boot/dts/allwinner/
cp ./bsp/configs/linux-6.18/defconfig .config

export BSP_TOP=$PWD/bsp/
```
This is for native ARM64 compile:
```sh
make menuconfig
# to build ARM64 Image
make
# Or to build deb packages
make bindeb-pkg
```

CROSS_COMPILE (on Ubuntu):
```sh
sudo apt install build-essential gcc-aarch64-linux-gnu

make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- menuconfig
# to build ARM64 Image
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu-

# Or to build deb packages, you might need some additional packages
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- bindeb-pkg
# Check if it asks for more packages, you might need the following to install arm64 packages
sudo dpkg --add-architecture arm64
# You might need to add http://ports.ubuntu.com/ubuntu-ports to /etc/apt/sources.list.d/ubuntu.sources here
sudo apt update
```

## Feature Matrix
Everything here is tested on Radxa Cubie A7Z, I only have this device so I didn't really worked on device tree for A7A and A7S, **PR welcome here.**

This table adapts this page from linux-sunxi: https://linux-sunxi.org/Linux_mainlining_effort

|Driver |       |device-tree    |Status |Note           |
|-      |-      |-              |-      |-              |
|**A733**|      |               |       |               |
|ADC    |GPADC  |gpadc          |**MAIN**|7.2[14]       |
|       |LRADC  |lradc          |OFF    |               |
|       |Thermal|ths            |BSP    |               |
|Audio  |sound  |sunxi-snd\*    |BSP    |Untested       |
|Camera |vin    |sunxi-vin\*    |UNK    |[1]            |
|Clocks |CCUs   |*ccu           |BSP    |MAIN WIP[7]    |
|CPUFreq|       |               |**PATCH**|             |
|CPUIdle|       |               |UNK    |No drivers     |
|Crypto |       |sunxi-ce       |OFF    |               |
|DRM    |DE     |display-engine |BSP    |               |
|       |DI     |deinterlace    |BSP    |               |
|       |DSI    |dsi\*(combophy)|BSP    |Untested       |
|       |EDP    |sunxi-edp      |BSP    |               |
|       |HDMI   |sunxi-hdmi     |BSP    |               |
|       |LVDS   |lvds\*         |OFF    |               |
|       |RGB    |rgb\*          |OFF    |               |
|       |TCON   |tcon-\*        |BSP    |               |
|DMA    |       |dma-v106       |BSP    |               |
|ETH    |GMAC   |gmac\*         |BSP    |Untested       |
|G2D    |       |g2d            |BSP    |               |
|GPU    |PowerVR|img-bxm-4-64   |**MAIN**|[2] 7.2[13]   |
|HW Spinlocks|  |hwspinlock     |OFF    |               |
|I2C    |       |sun55i-a523-i2c|**MAIN**|Same as A523  |
|IOMMU  |       |iommu-v20      |BSP    |               |
|IR     |IR RX  |irrx           |UNK    |               |
|       |IR TX  |irtx           |OFF    |               |
|LEDC   |       |sunxi-leds     |**MAIN**|Same as A523  |
|MsgBox |       |msgbox         |OFF    |               |
|NPU    |       |npu            |**PATCH**|Untested[3]  |
|NSI    |       |sunxi-nsi      |BSP    |What's this?   |
|PCIE   |       |pcie           |BSP    |Untested       |
|Pinctrl|       |\*-pinctrl     |BSP    |MAIN WIP[8]    |
|PMU    |A55&A76|\*-pmu         |**MAIN**|              |
|       |PCK600 |pck600         |**MAIN**|7.1[11]       |
|PWM    |       |sunxi-pwm\*    |BSP    |               |
|RTC    |       |rtc\*          |BSP    |MAIN WIP[9]    |
|SERDES |       |cadence-\*     |BSP    |USB DP PCIE PHY|
|SID    |EFUSE  |sunxi-sid      |BSP    |Untested       |
|SPI    |       |sunxi-spi\*    |BSP    |Untested       |
|Storage|SD/MMC |sdc0           |**PATCH**|[4]          |
|       |eMMC   |sdc2           |**PATCH**|Untested[4]  |
|       |UFS    |sunxi-ufs\*    |BSP    |Untested       |
|Timer  |SOC    |soc_timer0     |BSP    |               |
|UART   |       |uart\*         |**MAIN**|              |
|USB    |USB    |\*hci\*        |BSP    |               |
|       |USB OTG|sunxi-udc/otg  |BSP    |               |
|       |USB 3.0|dwc3           |BSP    |[5]            |
|VE     |       |sunxi-cedar-ve |BSP    |Untested       |
|Watchdog|      |wdt            |**MAIN**|Same as A523  |
|WIFI/BT|       |aic8800        |BSP    |Radxa PKG[6]   |
|**Board Specific**||           |       |               |
|USB-C  |PHY Switcher|phy_switcher|BSP  |               |
|       |Controller|et7304      |**MAIN**|A7Z 7.1[12]   |
|       |Controller|husb311     |**MAIN**|A7S 7.1[12]   |
|AXP PMU|       |axp8191        |BSP    |MAIN WIP[10]   |
|PWM FAN|       |pwm-fan        |**MAIN**|              |

**Status**: 
- BSP: BSP drivers ported for 6.18.y
- MAIN: Mainline 6.18 drivers or backported 7.x drivers (in patch)
- PATCH: Off mainline patch
- OFF: Disabled on Radxa config and/or Radxa device tree
- UNK: Unknown, disabled now, but enabled on Radxa config/dt

[1] Didn't port VIN Drivers since I don't have any camera to test. \
[2] Mainline Driver for BXM works, but patch for pck600 is needed (GENPD_FLAG_ALWAYS_ON). \
[3] Mainline vivante,gc can be detected, but need extra clocks & resets. \
[4] Mainline allwinner,sun55i-a523-(e)mmc works, need extra clocks & pinctrl fix. \
[5] Mainline dwc exists, please check linux-sunxi website. \
[6] Drivers incldued here is from Radxa repo. See below.

[7] https://lore.kernel.org/linux-sunxi/20260121-a733-rtc-v1-0-d359437f23a7@pigmoral.tech/ \
[7] https://lore.kernel.org/linux-sunxi/20260310-a733-clk-v1-0-36b4e9b24457@pigmoral.tech/ \
[8] https://lore.kernel.org/linux-sunxi/20250821004232.8134-1-andre.przywara@arm.com/ \
[9] https://lore.kernel.org/linux-sunxi/20260121-a733-rtc-v1-0-d359437f23a7@pigmoral.tech/ \
[10] https://lore.kernel.org/linux-sunxi/20250813235330.24263-1-andre.przywara@arm.com/

[11] https://lore.kernel.org/linux-sunxi/20260305-b4-pck600-a733-v2-0-ba6bbed7d253@gmail.com/ \
[12] https://lore.kernel.org/linux-usb/20260220-et7304-v3-0-ede2d9634957@gmail.com/ \
[12] https://lore.kernel.org/linux-usb/20260318-husb311-v4-0-69e029255430@flipper.net/ \
[13] https://lore.kernel.org/linux-sunxi/20260510-pck600-a733-gpu-v1-1-d6393646d714@gmail.com/ \
[14] https://lore.kernel.org/linux-sunxi/20260516-sunxi-a523-gpadc-v3-0-a3a04cff2620@mmpsystems.pl/

## Driver Specific

### MMC: allwinner,sun55i-a523-(e)mmc
Applying only clock-related changes allows the driver to work, but UHS-SDR104 speed is limited to ~45 MB/s. \
Investigation shows the clock and 1.8 V/3.3 V voltage switching logic differs on the A733. After adjusting for this, speeds reach ~80 MB/s. \
Further tweaking the `dma_mask` and `PAGE_SIZE` increases performance to ~92 MB/s (reason not fully understood). \
In the BSP, MMC supports HSQ and CQE, but most of these features are commented out in the device tree.

### GPU: IMG PowerVR BXM
The Imagination driver on Linux 6.18 works well on the A733. \
After placing the firmware [here](https://gitlab.freedesktop.org/imagination/linux-firmware/-/tree/powervr/powervr) in the correct location, the driver loads successfully. \
Building the Mesa driver from the main branch enables full functionality: `vulkaninfo` can detects the GPU, Wayland `sway` works with GLES, `vkmark` runs on-screen. \
Thanks to icenowy & iuncuim for assistance with debugging.

### WIFI\BT: FCU760K/AIC8800
Radxa maintains a dedicated [repo](https://github.com/radxa-pkg/aic8800) for AIC8800 drivers. Releases there should be compatible with Debian-based distributions. \
The AIC8800 is USB-based and not listed in the device tree. Drivers included here are stripped from Radxa’s repo ([this release](https://github.com/radxa-pkg/aic8800/releases/tag/5.0%2Bgit20260123.5f7be68d-5)) \
Firmware is required. `dmesg` will warn if it’s missing. Please put the firmware [here](https://github.com/radxa-pkg/aic8800/tree/main/src/USB/driver_fw/fw) in the correct place (put `aic8800D80` under `/lib/firmware/`). 

### PWM Fan
The stock thermal zone configuration only drives the PWM fan at full speed. \
By using the DDR thermal zone and switching its thermal policy to `step_wise` via a `udev` rule, the fan speed can be adjusted more dynamically based on temperature.