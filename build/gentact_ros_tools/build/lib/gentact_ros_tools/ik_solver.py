import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from trac_ik_python.trac_ik import IK
import tf2_ros
import tf_transformations
import numpy as np

class FrankaIKSolver(Node):
    def __init__(self):
        super().__init__('franka_ik_solver')

        self.declare_parameter('base_link', 'fr3_link0')
        self.declare_parameter('tip_link', 'fr3_link8')
        self.declare_parameter('urdf_param', 'robot_description')
        self.declare_parameter('target_frame', 'map')

        base_link = self.get_parameter('base_link').get_parameter_value().string_value
        tip_link = self.get_parameter('tip_link').get_parameter_value().string_value
        urdf_param = self.get_parameter('urdf_param').get_parameter_value().string_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.target_sub = self.create_subscription(
            TransformStamped,
            '/target_pose',
            self.pose_callback,
            10
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.ik_solver = IK(base_link, tip_link, urdf_string=self.get_parameter(urdf_param).get_parameter_value().string_value)
        self.joint_names = self.ik_solver.get_joint_names()

    def pose_callback(self, msg: TransformStamped):
        try:
            # Transform pose to "map" frame
            transform_to_map = self.tf_buffer.lookup_transform(
                self.target_frame,  # to frame
                msg.header.frame_id,  # from frame
                rclpy.time.Time()
            )

            # Combine transform_to_map and msg.transform
            pose_mat = self._transform_to_matrix(msg.transform)
            map_mat = self._transform_to_matrix(transform_to_map.transform)
            full_mat = np.matmul(map_mat, pose_mat)

            # Extract translation and rotation
            trans = full_mat[:3, 3]
            rot = tf_transformations.quaternion_from_matrix(full_mat)

            # Solve IK
            seed = [0.0] * self.ik_solver.number_of_joints
            ik_solution = self.ik_solver.get_ik(
                seed,
                trans[0], trans[1], trans[2],
                rot[0], rot[1], rot[2], rot[3]
            )

            if ik_solution:
                js = JointState()
                js.header.stamp = self.get_clock().now().to_msg()
                js.name = self.joint_names
                js.position = ik_solution
                self.joint_pub.publish(js)
            else:
                self.get_logger().warn('No IK solution found for pose in map frame.')

        except tf2_ros.LookupException as e:
            self.get_logger().warn(f'TF lookup failed: {e}')

    def _transform_to_matrix(self, t: TransformStamped.transform.__class__):
        q = [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]
        mat = tf_transformations.quaternion_matrix(q)
        mat[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
        return mat

def main(args=None):
    rclpy.init(args=args)
    node = FrankaIKSolver()
    rclpy.spin(node)
    rclpy.shutdown()