from pymavlink import mavutil
connect=mavutil.mavlink_connection('/dev/ttyTHS1',baud=115200)
#connect.target_system=255
#connect=mavutil.mavlink_connection('udpout:localhost:14445',source_system=1)
connect.wait_heartbeat()
print('connect successfully')
for i in range(100):
	connect.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO,"target number123".encode())
