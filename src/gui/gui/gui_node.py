import sys
from math import atan2, cos, sin, pi
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from omnibot_msgs.msg import Landmarks, Observations
from geometry_msgs.msg import Pose

from tkinter import Tk, Canvas, Frame, BOTH

def quaternion_to_yaw(q):
  return atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

def rotate_2d_vector(vector, theta):
  x, y = vector
  return ((x * cos(theta)) - (y * sin(theta)), (x * sin(theta) + y * cos(theta)))

def translate_2d_vector(vector, translation_vector):
  x, y = vector
  tx, ty = translation_vector
  return (x + tx, y + ty)

def robot_to_world(pose, vr):
  theta = quaternion_to_yaw(pose.orientation)
  return translate_2d_vector(rotate_2d_vector(vr, theta), (pose.position.x, pose.position.y))

def get_obs_position(pose, observation):
  xr = observation.range * cos(observation.bearing)
  yr = observation.range * sin(observation.bearing)
  return robot_to_world(pose, (xr, yr))

class GUIFrame(Frame):
  GRID_WIDTH = 20
  GRID_HEIGHT = 20

  def __init__(self, window_dim=(600, 600)):
    super().__init__()
    self.master.title("omnibot gui")
    self.pack(fill=BOTH, expand=1)

    self.canvas = Canvas(self)
    self.unit_px = 200
    w, h = window_dim
    self.center = (w / 2.0 , h / 2.0)



  def point_to_pixel(self, x , y):
    cx, cy = self.center
    px = cx + (x * (self.unit_px))
    py = cy - (y * (self.unit_px))
    return (px, py)


  def draw_grid(self):
    cx, cy = self.center
    grid_height_px = GUIFrame.GRID_HEIGHT * self.unit_px
    grid_width_px = GUIFrame.GRID_WIDTH * self.unit_px
    x0 = cx - grid_width_px/2
    y0 = cy - grid_height_px/2
    x1 = x0 + grid_width_px
    y1 = y0 + grid_height_px

    self.canvas.create_rectangle(x0, y0, x1, y1, fill='white')

    # vertical lines
    for x in range(int(x0), int(grid_width_px), int(self.unit_px/2.0)):
      self.canvas.create_line(x, y0, x, y1, fill='#ddd')

    # horizontal lines
    for y in range(int(y0), int(grid_height_px), int(self.unit_px/2.0)):
      self.canvas.create_line(x0, y, x1, y, fill='#ddd')

    self.canvas.create_oval(cx-1, cy-1, cx+1, cy+1, fill='#000', outline='#000')


  def draw_coordinates(self):
    cx, cy = self.center
    (ix, iy) = self.point_to_pixel(1.0, 0.0)
    (jx, jy) = self.point_to_pixel(0.0, 1.0)

    # create the x axis
    self.canvas.create_line(cx, cy, ix, iy, fill='#0f0')

    # create the y axis
    self.canvas.create_line(cx, cy, jx, jy, fill='#f00')

  def draw_robot(self, pose, color='#000', radius=1.0):
    (rcx, rcy) = self.point_to_pixel(pose.position.x, pose.position.y)
    r = radius * self.unit_px
    self.canvas.create_oval(rcx-r, rcy-r, rcx+r, rcy+r, outline=color, width=5)

    theta = quaternion_to_yaw(pose.orientation)

    # draw line
    line_pts = [translate_2d_vector(rotate_2d_vector(v, theta), (pose.position.x, pose.position.y)) for v in [(radius, 0), (-radius, 0)]]
    line_pts_px = [self.point_to_pixel(x, y) for x,y in line_pts]
    [(lx0, ly0), (lx1, ly1)] = line_pts_px

    self.canvas.create_line(lx0, ly0, lx1, ly1, fill=color, width=5)


    reg_triangle_pts = [(radius/2.0, radius/4.0), (radius/2.0, -radius/4.0), (radius, 0)]
    triangle_pts = [translate_2d_vector(rotate_2d_vector(v, theta), (pose.position.x, pose.position.y)) for v in reg_triangle_pts]  
    triangle_pts_px = [x for t in [self.point_to_pixel(x, y) for x, y in triangle_pts] for x in t]

    self.canvas.create_polygon(*triangle_pts_px, fill=color)

  def draw_square(self, center, color):
    px_size = 10
    px, py = center
    sx, sy = self.point_to_pixel(px, py)
    x0, y0 = sx - px_size, sy - px_size
    x1, y1 = sx + px_size, sy + px_size

    self.canvas.create_rectangle(x0, y0, x1, y1, fill=color)


  def draw_landmarks(self, landmarks):
    for landmark in landmarks.landmarks:
      v = (landmark.position.x, landmark.position.y)
      self.draw_square(center=v, color='#f00')

  def draw_observations(self, pose, observations):
    for observation in observations.observations:
      self.draw_square(center=get_obs_position(pose, observation), color='#f0f')



class GUINode(Node):
  def __init__(self, gui_window, gui_frame):
    super().__init__("gui_node")
    self.gui_window = gui_window
    self.gui_frame = gui_frame
    self.odom_subscription = self.create_subscription(Odometry, 'odom', self.handle_odom, 10)
    self.perfect_odom_subscription = self.create_subscription(Odometry, 'perfect_odom', self.handle_perfect_odom, 10)
    self.est_odom_subscription = self.create_subscription(Odometry, 'est_odom', self.handle_est_odom, 10)
    self.landmarks_subscription = self.create_subscription(Landmarks, 'landmarks', self.handle_landmarks, 10)
    self.observations_subscription = self.create_subscription(Observations, 'observations', self.handle_observations, 10)
    self.gui_timer = self.create_timer(0.1, self.timer_callback)

    self.odom = Odometry()
    self.perfect_odom = Odometry()
    self.est_odom = Odometry()
    self.landmarks = Landmarks()
    self.observations = Observations()
    
    self.draw()
    self.gui_window.update()
    self.odom_subscription # prevent unused variable warning
    self.perfect_odom_subscription # prevent unused variable warning
    self.est_odom_subscription # prevent unused variable warning
    self.landmarks_subscription # prevent unused variable warning
    self.observations_subscription # prevent unused variable warning

  def timer_callback(self):
    try:
      self.gui_window.update()
    except:
      print('exited window')
      sys.exit(0)
      

  def destroy_node(self):
    self.gui_window.quit()
    super().destroy_node()

  def draw(self):
    self.gui_frame.draw_grid()
    self.gui_frame.draw_coordinates()

    self.gui_frame.draw_robot(self.perfect_odom.pose.pose, color='#00f',  radius=0.1225)
    self.gui_frame.draw_robot(self.odom.pose.pose, color='#000',  radius=0.1225)
    self.gui_frame.draw_robot(self.est_odom.pose.pose, color='#0f0',  radius=0.1225)
    self.gui_frame.draw_landmarks(self.landmarks)
    self.gui_frame.draw_observations(self.odom.pose.pose, self.observations)


    self.gui_frame.canvas.pack(fill=BOTH, expand=1)


  def handle_odom(self, msg):
    self.odom = msg
    self.draw()
    self.gui_window.update()
    return

  def handle_perfect_odom(self, msg):
    self.perfect_odom = msg
    self.draw()
    self.gui_window.update()
    return

  def handle_est_odom(self, msg):
    self.est_odom = msg
    self.draw()
    self.gui_window.update()
    return

  def handle_landmarks(self, msg):
    print('got landmarks')
    self.landmarks = msg
    self.draw()
    self.gui_window.update()
    return

  def handle_observations(self, msg):
    print('got observations')
    self.observations = msg
    self.draw()
    self.gui_window.update()
    return
    

def main(args=None):
  rclpy.init(args=args)
  gui_window = Tk()
  w, h = 1400, 1600
  gui_window.geometry('{}x{}+400+400'.format(w, h))
  gui_frame = GUIFrame(window_dim=(w, h))
  gui_node = GUINode(gui_window, gui_frame)
  rclpy.spin(gui_node)
  gui_node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
