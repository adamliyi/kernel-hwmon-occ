# kernel-hwmon-occ
hwmon driver for the openpower OCC. BMC uses it to read CPU sensor data via OCC (On-Chip-Controller).
It is part of openbmc.

After building, a kernel module named "occ.ko" is generated.

The driver exposed bellow HWMON interface:

*_input: means sensor_value.
*_label: means sensor_id

root@palmetto:~# ls /sys/class/hwmon/hwmon0/
device        freq4_input   power2_input  temp1_input   temp4_label   temp8_input
freq1_input   freq4_label   power2_label  temp1_label   temp5_input   temp8_label
freq1_label   name          power3_input  temp2_input   temp5_label   temp9_input
freq2_input   of_node       power3_label  temp2_label   temp6_input   temp9_label
freq2_label   power         power4_input  temp3_input   temp6_label   uevent
freq3_input   power1_input  power4_label  temp3_label   temp7_input
freq3_label   power1_label  subsystem     temp4_input   temp7_label

