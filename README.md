# unity-gps-rtk
1. Run the py script on the PI with the following args:

sudo python3 gps.py \
-u rtkusername \
-p rtkpassword \
-f gpsdata.txt \
--fixrate 100 \
--relayhost server_ip \
--relayport server_port \
rtk_ip \
rtk_port \
VRS_RTCM34_MSM4
