from pymavlink import mavutil
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import String

num="none"
def cb_str(msg):
	global num
	num=msg.data
	print("get"+num)
def cb(msg):
	global num
	if msg.data==777.777:
		connect=mavutil.mavlink_connection('/dev/ttyTHS1',baud=115200)
		#connect.target_system=255
		#connect=mavutil.mavlink_connection('udpout:localhost:14445',source_system=1)
		connect.wait_heartbeat()
		print('connect successfully')
		for i in range(5):
			connect.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO,num.encode())

rospy.init_node("send")
rospy.Subscriber('send_topic', Float64, cb, queue_size=10)
rospy.Subscriber('final_result', String, cb_str, queue_size=10)


	
rospy.spin()