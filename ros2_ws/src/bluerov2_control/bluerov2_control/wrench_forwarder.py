"""
Wrench forwarder — receives body wrench from ROS2 and applies it
to the BlueROV2 base_link in Gazebo via native gz-transport.

This bridges the gap between the ROS2 controller (wrench message)
and Gazebo's internal force application, bypassing the Thruster
plugin and its DART-incompatible PID joint torque approach.
"""
import os
os.environ['PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION'] = 'python'

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench

from gz.transport13 import Node as GzNode
from gz.msgs10.entity_wrench_pb2 import EntityWrench
from gz.msgs10.entity_pb2 import Entity


class WrenchForwarder(Node):
    """
    Subscribes to /model/bluerov2/body_wrench (geometry_msgs/Wrench)
    and applies the wrench to the base_link using gz-transport publisher.
    """

    def __init__(self):
        super().__init__('wrench_forwarder')

        # Gazebo transport publisher
        self.gz_node = GzNode()
        self.gz_pub = self.gz_node.advertise(
            '/world/underwater_world/wrench',
            EntityWrench)

        # ROS subscriber
        self.sub = self.create_subscription(
            Wrench, '/model/bluerov2/body_wrench',
            self._wrench_cb, 10)

        self._count = 0
        self.get_logger().info('Wrench forwarder started (native gz-transport)')

    def _wrench_cb(self, msg: Wrench):
        """Forward wrench to Gazebo via native gz-transport."""
        # Build EntityWrench protobuf message
        ew = EntityWrench()
        ew.entity.name = 'bluerov2::base_link'
        ew.entity.type = Entity.LINK

        ew.wrench.force.x = msg.force.x
        ew.wrench.force.y = msg.force.y
        ew.wrench.force.z = msg.force.z
        ew.wrench.torque.x = 0.0
        ew.wrench.torque.y = 0.0
        ew.wrench.torque.z = msg.torque.z

        self.gz_pub.publish(ew)

        self._count += 1
        if self._count % 50 == 0:
            self.get_logger().info(
                f'Wrench: F=[{msg.force.x:.1f},{msg.force.y:.1f},{msg.force.z:.1f}] '
                f'T=[0,0,{msg.torque.z:.1f}]')


def main(args=None):
    rclpy.init(args=args)
    node = WrenchForwarder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
