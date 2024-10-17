import rclpy
from rclpy.node import Node
import rclpy.time
import tf2_ros
import math
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.N = 10
        self.rate = self.create_timer(0.02, self.control_loop)  # 50Hz
        self.get_state = self.create_timer(0.02, self.get_current_state)  # 50Hz

        self.local_plan_sub = self.create_subscription(
            Float32MultiArray,
            '/local_plan',
            self.local_planner_cb,
            10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.curr_state_pub = self.create_publisher(Float32MultiArray, '/curr_state', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.linear_speed = self.angular_speed = 0.0
        self.local_plan = np.zeros([self.N, 2])

    def quart_to_rpy(self, x, y, z, w):
        r = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
        p = math.asin(2*(w*y-z*x))
        y = math.atan2(2*(w*z+x*y), 1-2*(z*z+y*y))
        return r, p, y
    
    def get_current_state(self):
        try:
            transform = self.tf_buffer.lookup_transform('world', 'base_link',
                                                        rclpy.time.Time())
            _,_,yaw = self.quart_to_rpy(transform.transform.rotation.x,
                                        transform.transform.rotation.y,
                                        transform.transform.rotation.z,
                                        transform.transform.rotation.w)
            curr_state = Float32MultiArray()
            curr_state.data = [transform.transform.translation.x,
                               transform.transform.translation.y,
                               yaw]
            self.curr_state_pub.publish(curr_state)
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException,
               tf2_ros.ExtrapolationException):
            pass
    
    def pub_vel(self):
        control_cmd = Twist()
        control_cmd.linear.x = self.linear_speed
        control_cmd.angular.z = self.angular_speed
        self.get_logger().info('Linear Speed: %.1f, Angular Speed: %.1f' % (self.linear_speed, self.angular_speed))
        self.vel_pub.publish(control_cmd)

    def control_loop(self):
        self.linear_speed = self.local_plan[0, 0]
        self.angular_speed = self.local_plan[0, 1]
        self.pub_vel()

    def local_planner_cb(self, msg):
        for i in range(self.N):
            self.local_plan[i, 0] = msg.data[0 + 2 * i]
            self.local_plan[i, 1] = msg.data[1 + 2 * i]

def main(args = None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
            