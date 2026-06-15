"""Manual loop closure on KISS-SLAM poses: snap end->start, linearly distribute
the position gap along the trajectory, rebuild the global map from the bag."""
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2

POSES = "/home/nus-ai/divek_nus/ml_lidar/projects/03_autonomy/results/kiss_slam/slam_output/2026-06-13_23-56-02/corridor2_poses.npy"
BAG = "/home/nus-ai/divek_nus/ml_lidar/projects/02_fast_lio2/results/corridor2.0"
OUT = "/tmp/kiss_map_closed.npz"
OUT_POSES = "/home/nus-ai/divek_nus/ml_lidar/projects/03_autonomy/results/kiss_slam/poses_loopclosed.npy"
VOX = 0.05

poses = np.load(POSES)                          # (N,4,4)
N = len(poses)
gap = poses[0][:3,3] - poses[-1][:3,3]          # vector that brings END onto START
print("end-start gap %.3f m -> distributing linearly" % np.linalg.norm(gap), flush=True)
corr = poses.copy()
frac = (np.arange(N)/(N-1))[:,None]             # 0 at start, 1 at end
corr[:,:3,3] += frac * gap                      # pose0 unchanged; poseN-1 += gap (lands on start)
print("residual end-start after closure: %.4f m" % np.linalg.norm(corr[0][:3,3]-corr[-1][:3,3]), flush=True)
np.save(OUT_POSES, corr)

dt = np.dtype({'names':['x','y','z','intensity'],'formats':['<f4']*4,'offsets':[0,4,8,12],'itemsize':26})
def vdedup(w, inten, vox):
    g = np.floor(w/vox).astype(np.int64); g -= g.min(0); span = g.max(0)+1
    packed=(g[:,0]*span[1]+g[:,1])*span[2]+g[:,2]
    _, idx = np.unique(packed, return_index=True); return w[idx], inten[idx]

r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='mcap'), rosbag2_py.ConverterOptions('',''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/rslidar_points']))
acc=[]; aci=[]; i=0
while r.has_next():
    _, d, _ = r.read_next()
    if i>=N: break
    m = deserialize_message(d, PointCloud2)
    a = np.frombuffer(m.data, dtype=dt)
    xyz = np.stack([a['x'],a['y'],a['z']],1).astype(np.float64)
    inten = a['intensity'].astype(np.float32)
    good = np.isfinite(xyz).all(1); rr=np.linalg.norm(xyz,axis=1); good&=(rr>0.5)&(rr<80)
    xyz=xyz[good]; inten=inten[good]
    if len(xyz):
        T=corr[i]; w=(T[:3,:3]@xyz.T).T + T[:3,3]
        w,inten=vdedup(w.astype(np.float32),inten,VOX); acc.append(w); aci.append(inten)
    i+=1
    if i%1600==0: print("scan",i,flush=True)
world=np.concatenate(acc); inten=np.concatenate(aci)
world,inten=vdedup(world,inten,VOX)
np.savez_compressed(OUT, world=world, inten=inten, traj=corr[:,:3,3])
print("DONE %d pts -> %s"%(len(world),OUT), flush=True)
