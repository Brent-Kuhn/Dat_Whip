import rospy as rp
from std_msgs.msg import String

def callback(data):
	rp.loginfo(rp.get_caller_id() + 'I heard %s', data.data)

def listener():
	rp.init_node('polylistener', anonymous=True)
        rp.Subscriber('polycrier', String, callback)
	rp.spin()

if __name__ = '__main__':
	try:
		listener()
	except rp.ROSInterruptException:
		pass
