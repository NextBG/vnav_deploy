echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015",ATTRS{serial}=="DP04YF64", MODE:="0777", GROUP:="dialout", SYMLINK+="rover"' >/etc/udev/rules.d/rover.rules
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", ATTRS{bcdDevice}=="8134", MODE:="0777", GROUP:="dialout", SYMLINK+="wit_imu"' >/etc/udev/rules.d/wit_imu.rules
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", ATTRS{bcdDevice}=="0264", MODE:="0777", GROUP:="dialout", SYMLINK+="wit_gps"' >/etc/udev/rules.d/wit_gps.rules

service udev reload
sleep 0.5
service udev restart


