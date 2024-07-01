# This is a record of a successful attempt, the source code version of linux is not quite the same but it does not have a great impact.
# This script is just as a reference.
# I use the a computer with Ubuntu 20.04 LTS, and a Jetson Nano with the lastest pack (r32_release_v7.4).

# if need, upgrade or downgrade kernel
# mkdir -p ~/Dev/kernel-upgrade && cd ~/Dev/kernel-upgrade
# wget -c https://kernel.ubuntu.com/~kernel-ppa/mainline/v4.19.35/linux-headers-4.19.35-041935_4.19.35-041935.201904170334_all.deb
# wget -c https://kernel.ubuntu.com/~kernel-ppa/mainline/v4.19.35/linux-headers-4.19.35-041935-generic_4.19.35-041935.201904170334_amd64.deb
# wget -c https://kernel.ubuntu.com/~kernel-ppa/mainline/v4.19.35/linux-image-unsigned-4.19.35-041935-generic_4.19.35-041935.201904170334_amd64.deb
# wget -c https://kernel.ubuntu.com/~kernel-ppa/mainline/v4.19.35/linux-modules-4.19.35-041935-generic_4.19.35-041935.201904170334_amd64.deb
# sudo dpkg -i linux-headers-*.deb linux-image-*.deb linux-modules-*.deb


# add source code
mkdir -p ~/Dev
cd ~/Dev
git clone https://github.com/zlab-pub/wifi-ptp.git
git clone https://mirrors.tuna.tsinghua.edu.cn/git/linux-stable.git
git clone --depth 1 --branch v4.19.37 --single-branch  https://mirrors.tuna.tsinghua.edu.cn/git/linux-stable.git

# https://developer.nvidia.com/embedded/jetson-linux-archive
# nvidia nano Driver Package (BSP) Sources:  r32_release_v7.4 
# linux kernel source: v4.9.253
# https://developer.nvidia.com/downloads/embedded/l4t/r32_release_v7.4/sources/t210/public_sources.tbz2
# tar -xjf public_sources.tbz2
# cd Linux_for_Tegra/source/public
# tar –xjf kernel_src.tbz2
# !: cp -r kernel/kernel-4.9/drivers/net/wireless/ath ~/Dev/linux/drivers/net/wireless/ath
# mkdir -p ~/Dev/kernel/output
# cd kernel/kernel-4.9 && make ARCH=arm64 O=~/Dev/kernel/output tegra_defconfig

# install dependencies
sudo apt-get -y install build-essential libncurses-dev bison flex libssl-dev libelf-dev


# patch ath
cd ~/Dev/linux-stable/drivers/net/wireless/ath
cd ~/Dev/sources/kernel/kernel-4.9/drivers/net/wireless/ath
# touch ./ath9k/reload.sh
patch -sf -p1 < ~/Dev/wifi-ptp/ath9k.diff


# Jetson Nano compile
cd ~/Dev/sources/kernel/kernel-4.9
TEGRA_KERNEL_OUT=~/Dev/sources/kernel/output
mkdir -p $TEGRA_KERNEL_OUT
make ARCH=arm64 O=$TEGRA_KERNEL_OUT tegra_defconfig
make ARCH=arm64 O=$TEGRA_KERNEL_OUT -j$(nproc)
make ARCH=arm64 -j$(nproc) M=drivers/net/wireless/ath O=$TEGRA_KERNEL_OUT

#! ubuntu compile
cd ~/Dev/linux-stable/
cp /boot/config-$(uname -r) .config
make menuconfig
sudo make -j$(nproc) M=drivers/net/wireless/ath

# then install it with the reload.sh
sudo bash ./reload.sh


# Check for the PTP device.
ls /sys/class/net/wlan0/device/ptp
ls /sys/class/net/wlp1s0/device/ptp

# build linuxptp
cd ~/Dev/wifi-ptp/ptp
make

# start

sudo ./ptp4l -i wlan0 -p /dev/ptp1 -m
sudo ./ptp4l -i wlp1s0 -p /dev/ptp1 -m -s