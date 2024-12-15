import rclpy
import rclpy.duration
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import Point, TransformStamped, PointStamped
import cv2
import numpy as np
import math

DEBUG = 0

class pc_node(Node):
    def __init__(self):
        super().__init__('pc_node')
        self.bridge = CvBridge()
        
        # xyz thresholds
        self.x_thresh = 6.0
        self.y_thresh = 2.0
        self.z_thresh = 1.0
        self.lane_z_thresh = 0.2 # only white points below this z will be taken as lanes

        # RGB thresholds
        self.lane_thresh = [230, 230, 230]
        self.cone_thresh = [150, 100, 100]

        # Publishers (correct topics if needed)
        self.lane_pub = self.create_publisher(PointCloud2, '/lane_pc', 1)
        self.cone_pub = self.create_publisher(PointCloud2, '/cones_pc', 1)

        # Subscribers (correct topics if needed)
        self.create_subscription(PointCloud2, '/zed/zed_node/point_cloud/cloud_registered', self.pc_callback, 10)
        self.flag = 1

        try:
            # Transform from camera frame to base_link
            # Adjust these values according to the transform between zed and base_link on the bot
            # Ideally should be getting this from tf2 buffer
            # I am only rotating the points here, manually adding z coordinate to get it up to ground level
            # If transform is done normally, dont need to worry about anything
            r, p, y = self.euler_from_quaternion(0.0, 0.122, 0.0, 0.993)
            # Creating the rotation matrix
            self.Rx = np.array([[1, 0, 0],
                          [0, math.cos(r), -math.sin(r)],
                          [0, math.sin(r), math.cos(r)]])
            
            self.Ry = np.array([[math.cos(p), 0, math.sin(p)],
                          [0, 1, 0],
                          [-math.sin(p), 0, math.cos(p)]])
            
            self.Rz = np.array([[math.cos(y), -math.sin(y), 0],
                          [math.sin(y), math.cos(y), 0],
                          [0, 0, 1]])
            
        except Exception as e:
            self.get_logger().error(f'Error getting transform: {str(e)}')
    

    
    def pc_callback(self, data):
        """
        Processing the pointcloud data ->
        Converting to xyz and then processing to separate ground plane and obstacles
        """
        try:
            points = []
            transformed_points = []
            lane_points = []
            cone_points = []
            # Getting the xyz and rgb data of the points
            points = point_cloud2.read_points_list(data, field_names=["x", "y", "z", "rgb"], skip_nans=False)
            print(f"zed pointcloud points = {len(points)}")
            
            # Filtering by xyz to reduce points being processed
            points = np.array([(point.x, point.y, point.z, point.rgb) for point in points 
                                  if ((point.x < self.x_thresh) and (abs(point.y < self.y_thresh)) and (point.z < self.z_thresh))])
            
            # Rotating the points to base link
            for point in points:
                temp_point = np.array([[point[0]], 
                                      [point[1]], 
                                      [point[2]]])
                trans_point = np.matmul(np.matmul(np.matmul(self.Rx, self.Ry), self.Rz), temp_point)
                # Adding to z because we haven't translated the points while transforming
                transformed_points.append((trans_point[0][0], trans_point[1][0], trans_point[2][0] + 1.2, point[3]))
            transformed_points = np.float32(transformed_points)

            print(f"transformed points after masking = {len(transformed_points)}")

            # ChatGPT for extracting RGB, had NaN issues doing it the normal way - 
            transformed_points = np.array(transformed_points)
            rgb_floats = np.ascontiguousarray(transformed_points[:, 3])
            rgb_integers = rgb_floats.view(np.uint32)

            # Extract RGB values
            r = (rgb_integers >> 16) & 0xFF
            g = (rgb_integers >> 8) & 0xFF
            b = rgb_integers & 0xFF
            mask_lane = (r > self.lane_thresh[0]) & (g > self.lane_thresh[1]) & (b > self.lane_thresh[2])
            mask_cones = (r > self.cone_thresh[0]) & (g < self.cone_thresh[1]) & (b < self.cone_thresh[2])

            # Extract lane points and cone points
            lane_points = transformed_points[mask_lane]
            lane_points = [point for point in lane_points if (point[2] < self.lane_z_thresh)]
            cone_points = transformed_points[mask_cones]

            # Creating a pointcloud message from the filtered points
            header = Header()
            header.frame_id = "base_link"
            header.stamp = self.get_clock().now().to_msg()
            fields = [
                PointField(name="x", offset=0, datatype=7, count=1),
                PointField(name="y", offset=4, datatype=7, count=1),
                PointField(name="z", offset=8, datatype=7, count=1),
                PointField(name="rgb", offset=12, datatype=7, count=1),
            ]

            # Creating a pointcloud message from the lane points and cone points
            lane_pc_msg = point_cloud2.create_cloud(header=header, fields=fields, points=lane_points)
            cones_pc_msg = point_cloud2.create_cloud(header=header, fields=fields, points=cone_points)
            self.lane_pub.publish(lane_pc_msg)
            self.cone_pub.publish(cones_pc_msg)

            
        except Exception as e:
            self.get_logger().error(f'Error processing pointcloud: {str(e)}')

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

def main(args=None):
    rclpy.init(args=args)
    node = pc_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
