import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from my_robot_interfaces.msg import BesturingsData
from geometry_msgs.msg import Twist

class OdomFilterNode(Node):
    def __init__(self):
        super().__init__('rf2o_filter')
        
        # Unfiltered frames van rf2o
        self.source_odom_frame = 'odom_unfiltered'
        self.source_base_frame = 'base_link_unfiltered'
        
        # Gefilterde frames waar SLAM mee werkt
        self.target_odom_frame = 'odom'
        self.target_base_frame = 'base_link'
        
        # Drempels voor minimale beweging (pas aan naar wens) (tf)
        self.trans_threshold = 0.0005   # 0.5 mm
        self.rot_threshold = 0.0005    # ~0.0005 rad ≈ 0.029° 

        # Drempelwaarden voor linear en angular snelheid (odom)
        self.linear_threshold = 0.05
        self.angular_threshold = 0.09

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.last_transform = None

        # Timer om periodiek de transform op te halen
        self.timer = self.create_timer(0.05, self.check_transform) # 20 Hz
        
        # Subscribe op het bestaande odom topic
        self.odom_sub = self.create_subscription(Odometry, '/odom_rf2o', self.odom_callback, 10)
        
        # Publisher voor de gefilterde odom
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Subscribe to /cmd_vel and besturings_data
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.besturings_data_sub = self.create_subscription(BesturingsData, 'besturings_data', self.besturings_data_callback, 10)

        # Motion state flags
        self.cmd_vel_moving = False
        self.besturings_data_moving = False

    def cmd_vel_callback(self, msg: Twist):
    # Check if the robot is moving based on /cmd_vel
        if abs(msg.linear.x) > 0.0 or abs(msg.angular.z) > 0.0:
            self.cmd_vel_moving = True
        else:
            self.cmd_vel_moving = False

    def besturings_data_callback(self, msg: BesturingsData):
    # Check if the robot is moving based on besturings_data
    # throttle & steering
        if abs(msg.throttle) > 0.0 or abs(msg.steering) > 0.0:
            self.besturings_data_moving = True
        else:
            self.besturings_data_moving = False

    def odom_callback(self, msg: Odometry):
        # Kopieer het bericht, zodat we het kunnen aanpassen
        filtered_msg = Odometry()
        filtered_msg.header = msg.header
        filtered_msg.child_frame_id = msg.child_frame_id
        filtered_msg.pose = msg.pose
        
        # Haal de snelheden op
        vx = msg.twist.twist.linear.x
        vz = msg.twist.twist.angular.z
        
        # Check de thresholds en zet op 0 indien onder de grens
        if abs(vx) < self.linear_threshold:
            vx = 0.0
        if abs(vz) < self.angular_threshold:
            vz = 0.0
        
        # Stel de aangepaste snelheden in
        filtered_msg.twist.twist.linear.x = vx
        filtered_msg.twist.twist.linear.y = msg.twist.twist.linear.y
        filtered_msg.twist.twist.linear.z = msg.twist.twist.linear.z
        
        filtered_msg.twist.twist.angular.x = msg.twist.twist.angular.x
        filtered_msg.twist.twist.angular.y = msg.twist.twist.angular.y
        filtered_msg.twist.twist.angular.z = vz
        
        # Overnemen van covariance matrix (optioneel, afhankelijk van je behoeften)
        filtered_msg.twist.covariance = msg.twist.covariance
        filtered_msg.pose.covariance = msg.pose.covariance
        
        # print(f'rf2o x: {msg.twist.twist.linear.x}, z: {msg.twist.twist.angular.z}, filtered odom x: {filtered_msg.twist.twist.linear.x}, z: {filtered_msg.twist.twist.angular.z}')

        # Publiceer het aangepaste bericht
        self.odom_pub.publish(filtered_msg)

    def check_transform(self):
        # Skip publishing if both cmd_vel and besturings_data indicate no motion
        if not self.cmd_vel_moving and not self.besturings_data_moving:
            # print('No motion detected, skipping transform update')
            return

        try:
            trans = self.tf_buffer.lookup_transform(
                self.source_odom_frame, 
                self.source_base_frame, 
                rclpy.time.Time()
            )
        except Exception:
            # Geen transform beschikbaar, nog even wachten
            return
        
        if self.last_transform is None:
            self.last_transform = trans
            # Direct publiceren als eerste transform
            self.publish_filtered_tf(trans)
            return
        
        # Bereken verschil in translatie
        dx = trans.transform.translation.x - self.last_transform.transform.translation.x
        dy = trans.transform.translation.y - self.last_transform.transform.translation.y
        dz = trans.transform.translation.z - self.last_transform.transform.translation.z
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)

        # Bereken verschil in oriëntatie (als ruwe maat)
        dq_w = trans.transform.rotation.w - self.last_transform.transform.rotation.w
        dq_x = trans.transform.rotation.x - self.last_transform.transform.rotation.x
        dq_y = trans.transform.rotation.y - self.last_transform.transform.rotation.y
        dq_z = trans.transform.rotation.z - self.last_transform.transform.rotation.z
        rot_diff = math.sqrt(dq_w*dq_w + dq_x*dq_x + dq_y*dq_y + dq_z*dq_z)
        
        # Controleer of verschil groter is dan thresholds
        if dist > self.trans_threshold or rot_diff > self.rot_threshold:
            self.last_transform = trans
            self.publish_filtered_tf(trans)
        else:
            # Geen update als er geen significante verandering is
            pass

    def publish_filtered_tf(self, trans: TransformStamped):
        # We gebruiken dezelfde positie en oriëntatie, maar met target_odom_frame en target_base_frame
        filtered_trans = TransformStamped()
        filtered_trans.header.stamp = self.get_clock().now().to_msg()
        filtered_trans.header.frame_id = self.target_odom_frame
        filtered_trans.child_frame_id = self.target_base_frame

        filtered_trans.transform.translation = trans.transform.translation
        filtered_trans.transform.rotation = trans.transform.rotation

        self.tf_broadcaster.sendTransform(filtered_trans)


def main(args=None):
    rclpy.init(args=args)
    node = OdomFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
