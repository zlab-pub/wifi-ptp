
# Wi-Fi PTP (Wi-PTP)

## Brief

This project implements Precision Time Protocol (PTP) for Wi-Fi. Likely to be used in time-sensitive WLAN, UAV, etc., Original PTP is designed for LAN but it is possible to port it to Wi-Fi. The simplest way is to timestamp PTP packets in the NIC driver but is subject to several software uncertainties. This project makes use of the hardware TSF counter to emulate the hardware PTP clock like Ethernet NICs. A standard PTP clock interface is implemented in `ath9k`. Its performance is at 1 μs level. More details can be found in the [ATC'21 paper](https://www.usenix.org/system/files/atc21-chen.pdf).  

![Wi-PTP Implementation Overview](figures/Wi-PTP-impl.png)

The above figure illustrates how Wi-PTP works.

## Usage

The `ath9k` directory contains the modified ath9k driver based on Linux Kernel 4.19.37. The `ptp` directory contains the adjusted [linuxptp](http://linuxptp.sourceforge.net/).

### Build & Replace ath9k Driver

1. Patch `drivers/net/wireless/ath` in the Kernel source tree with this repo's `ath9k.diff`. We recommend `patch` to do this.

   ```bash
   cd <kernel_source>/drivers/net/wireless/ath
   patch -sf -p1 < <repo_path>/wifi-ptp/ath9k.diff
   ```

2. Build ath9k driver.

    ```bash
    cd <kernel source>
    make M=drivers/net/wireless/ath
    ```

3. Replace the ath9k driver.

    ```bash
    WLAN_DEV=wlan0

    ifdown ${WLAN_DEV}

    rmmod ath9k
    rmmod ath9k_common
    rmmod ath9k_hw
    rmmod ath

    cd <kernel source>/drivers/net/wireless/ath
    insmod ./ath.ko
    insmod ./ath9k/ath9k_hw.ko
    insmod ./ath9k/ath9k_common.ko
    insmod ./ath9k/ath9k.ko

    sleep 1

    ifup ${WLAN_DEV}
    ```

    Note: please replace the `wlan0` above with the actual ath9k interface.

4. Check for the PTP device.

    ```bash
    ls /sys/class/net/${WLAN_DEV}/device/ptp
    ```

    The above command should output a `ptp<X>`.

### Build linuxptp

The ptp version in this repo is modified to disable PTP capability checking. Our modified version based on the commit [dd30b3a](https://github.com/richardcochran/linuxptp/tree/dd30b3a0d94d1c087066066e5df6bc84e3019b0b). We also provide a file `linuxptp.patch`, which is the diff information between the commit `dd30b3a` and our modified linuxptp.

```bash
cd <repo path>/ptp
make
```

After building the `linuxptp`, the ar9300+ wireless cards can be used to perform PTP as ethernet cards.

### Running with `ptp4l`

Master: `./ptp4l -i wlan0 -p /dev/ptp<X> -m`
Slave:  `./ptp4l -i wlan0 -p /dev/ptp<X> -m -s`

## Citation

```plain
@inproceedings {273763,
author = {Paizhuo Chen and Zhice Yang},
title = {Understanding Precision Time Protocol in Today{\textquoteright}s Wi-Fi Networks: A Measurement Study},
booktitle = {2021 {USENIX} Annual Technical Conference ({USENIX} {ATC} 21)},
year = {2021},
isbn = {978-1-939133-23-6},
pages = {597--610},
url = {https://www.usenix.org/conference/atc21/presentation/chen},
publisher = {{USENIX} Association},
month = jul,
}
```

## Contact

If you have any questions, please don't hesitate to contact [yangzhc@shanghaitech.edu.cn](mailto:yangzhc@shanghaitech.edu.cn) or [chenpzh@shanghaitech.edu.cn](mailto:chenpzh@shanghaitech.edu.cn).
