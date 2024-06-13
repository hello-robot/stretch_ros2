import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_geometry_msgs 

class PoseTransformer(Node):
    def __init__(self):
        super().__init__('object_transform_node')
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self)

        self.pose_pub = self.create_publisher(PoseStamped, 'object_pose', 10)
        self.object_frame = 'Pringles' # The frame of the object we want to transform
        self.base_frame = 'base_link' # The frame we want to transform the object to

        self.timer = self.create_timer(0.1, self.handle_pose_transformation)

    def handle_pose_transformation(self):
        # Try to transform the pose of Pringles with respect to base_link and publish it
        try:
            # Wait for the transformation to be available
            if not self.tf_buffer.can_transform(self.base_frame, self.object_frame, rclpy.time.Time()):
                self.get_logger().info('Transformation not available yet.')
                return

            now = rclpy.time.Time()
            object_pose = self.tf_buffer.lookup_transform('base_link', 'Pringles', now)

            object_pose_transform = PoseStamped()
            object_pose_transform.header.frame_id = self.base_frame
            object_pose_transform.header.stamp = now.to_msg()

            object_pose_transform.pose.position.x = object_pose.transform.translation.x 
            object_pose_transform.pose.position.y = object_pose.transform.translation.y 
            object_pose_transform.pose.position.z = object_pose.transform.translation.z 
            object_pose_transform.pose.orientation = object_pose.transform.rotation

            self.pose_pub.publish(object_pose_transform)
            
        except Exception as e:
            self.get_logger().error('Failed to transform pose: %s' % str(e))

def main(args=None):
    rclpy.init(args=args)
    node = PoseTransformer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
