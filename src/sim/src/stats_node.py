import rclpy
import numpy as np
import time
import matplotlib.pyplot as plt
from rclpy.node import Node
from nav_msgs.msg import Odometry


class StatsNode(Node):
  def __init__(self):
    super().__init__('stats_node')
    self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.handle_odom, 10)
    self.perfect_odom_subscriber = self.create_subscription(Odometry, 'perfect_odom', self.handle_perfect_odom, 10)
    self.est_odom_subscriber = self.create_subscription(Odometry, 'est_odom', self.handle_est_odom, 10)

    self.timer = self.create_timer(0.5, self.update_plot)

    self.odom_subscriber
    self.perfect_odom_subscriber
    self.est_odom_subscriber

    self.odom = []
    self.perfect_odom = []
    self.est_odom = []

  def handle_odom(self, msg):
    self.odom.append(msg)

  def handle_perfect_odom(self, msg):
    self.perfect_odom.append(msg)

  def handle_est_odom(self, msg):
    self.est_odom.append(msg)

  def create_timer(self):

    pass


def main(args=None):
  rclpy.init(args=args)

  stats_node = StatsNode()
  rospy.spin(stats_node)

  stats_node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
