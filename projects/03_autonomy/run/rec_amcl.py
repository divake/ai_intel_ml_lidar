import time, sys, rclpy, csv
from geometry_msgs.msg import PoseWithCovarianceStamped
rclpy.init(); n=rclpy.create_node('rec_amcl')
f=open('/tmp/amcl_track.csv','w'); w=csv.writer(f); w.writerow(['t','x','y'])
cnt=[0]
def cb(m):
    cnt[0]+=1; w.writerow([round(time.time(),3), round(m.pose.pose.position.x,4), round(m.pose.pose.position.y,4)]); f.flush()
n.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',cb,10)
dur=float(sys.argv[1]) if len(sys.argv)>1 else 120
t0=time.time()
while time.time()-t0<dur: rclpy.spin_once(n,timeout_sec=0.1)
f.close(); print("amcl poses recorded:",cnt[0])
