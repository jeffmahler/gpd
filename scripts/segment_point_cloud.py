"""
Node to segment point clouds based on the workspace
Author: Jeff Mahler
"""
from geometry_msgs.msg import Point
from gpd.msg import CloudIndexed
import IPython
import numpy as np
from scipy.linalg import lstsq
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header, Int64
import rospy

from autolab_core import Box, PointCloud, RigidTransform
from perception import CameraIntrinsics
from visualization import Visualizer3D as vis3d

CAMERA_INTR_FILENAME = '/nfs/diskstation/calib/primesense_overhead/primesense_overhead.intr'
CAMERA_POSE_FILENAME = '/nfs/diskstation/calib/primesense_overhead/primesense_overhead_to_world.tf'
camera_intr = CameraIntrinsics.load(CAMERA_INTR_FILENAME)
T_camera_world = RigidTransform.load(CAMERA_POSE_FILENAME).as_frames('camera', 'world')
workspace = Box(min_pt=np.array([0.22, -0.22, 0.005]),
                max_pt=np.array([0.56, 0.22, 0.2]),
                frame='world')
pub = None

def cloudCallback(msg):
    global T_camera_world
    
    # read the cloud
    cloud_array = []
    for p in point_cloud2.read_points(msg):
        cloud_array.append([p[0], p[1], p[2]])
    cloud_array = np.array(cloud_array).T
            
    # form a point cloud
    point_cloud_camera = PointCloud(cloud_array, frame='camera')

    # segment the point cloud
    point_cloud_world = T_camera_world * point_cloud_camera
    seg_point_cloud_world, valid_indices = point_cloud_world.box_mask(workspace)

    """
    point_cloud_world.remove_infinite_points()
    vis3d.figure()
    vis3d.points(point_cloud_world, random=True, subsample=10, scale=0.001, color=(0,0,1))
    vis3d.points(seg_point_cloud_world, random=True, subsample=10, scale=0.0015, color=(0,1,0))
    vis3d.pose(T_camera_world)
    vis3d.show()
    IPython.embed()
    """
    
    # convert to indexed cloud frame
    frame_id = msg.header.frame_id
    msg = CloudIndexed()
    header = Header()
    header.frame_id = frame_id
    header.stamp = rospy.Time.now()
    msg.cloud_sources.cloud = point_cloud2.create_cloud_xyz32(header, cloud_array.T.tolist())
    msg.cloud_sources.view_points.append(Point(0,0,0))
    for i in xrange(cloud_array.shape[1]):
        msg.cloud_sources.camera_source.append(Int64(0))
    for i in valid_indices:
        msg.indices.append(Int64(i))    

    # publish
    global pub
    pub.publish(msg)
    print 'Published cloud with', len(msg.indices), 'indices'
    
if __name__ == '__main__':
    # create a ROS node.
    rospy.init_node('segment_point_cloud')

    # subscribe to the ROS topic that contains the grasps.
    cloud_sub = rospy.Subscriber('/camera/depth_registered/points', PointCloud2, cloudCallback)

    # Publish point cloud and nonplanar indices.
    global pub
    pub = rospy.Publisher('/segmented_cloud', CloudIndexed, queue_size=1)

    # Spin
    rospy.spin()
