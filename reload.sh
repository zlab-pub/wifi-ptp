WLAN_DEV=wlan0
WLAN_DEV=wlp1s0
ifconfig ${WLAN_DEV} down

rmmod ath9k
rmmod ath9k_common
rmmod ath9k_hw
rmmod ath

# Ubuntu
cd ~/Dev/linux-stable/drivers/net/wireless/ath
# Nano
cd ~/Dev/sources/kernel/output/drivers/net/wireless/ath

insmod ./ath.ko
insmod ./ath9k/ath9k_hw.ko
insmod ./ath9k/ath9k_common.ko
insmod ./ath9k/ath9k.ko

sleep 1

ifconfig ${WLAN_DEV} up