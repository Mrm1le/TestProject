import math


class Point2D(object):
  def __init__(self, x, y, theta):
    self.x = x
    self.y = y
    self.theta = theta


def tf2d(local_frame: Point2D, pose: Point2D):
  Point_local = Point2D(0, 0, 0)
  Point_local.x, Point_local.y = rotate2d(pose.x - local_frame.x,
                                          pose.y - local_frame.y,
                                          -local_frame.theta, 0.0, 0.0)
  Point_local.theta = NormalizeAngle(pose.theta - local_frame.theta)
  return Point_local


def rotate2d(lx, ly, theta, ox, oy):
  cos_a = math.cos(theta)
  sin_a = math.sin(theta)
  gx = ox + lx * cos_a - ly * sin_a
  gy = oy + lx * sin_a + ly * cos_a
  return gx, gy


def NormalizeAngle(angle):
  a = math.fmod(angle + math.pi, 2.0 * math.pi)
  if a < 0.0:
    a += (2.0 * math.pi)

  return a - math.pi


# if __name__ == '__main__':
#     local_frame = Point2D(1, 1, 1)
#     pose = Point2D(2, 2, 2)
#     pose_local = tf2d(local_frame, pose)
#     print(pose_local.x, pose_local.y, pose_local.theta)
