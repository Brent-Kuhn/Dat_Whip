import rospy as rp
from std_msgs.msg import String

pub = rp.Publisher('polycrier', String, queue_size=10)
rp.init_node('polylistener', anonymous=True)
r = rp.Rate(10) # 10hz
def publisher():
  while not rp.is_shutdown():
    pub.publish('hello, world!')
    r.sleep()

if __name__ = '__main__':
  try:
    publisher()
  except rp.ROSInterruptException:
    pass
