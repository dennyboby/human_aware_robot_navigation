import tf
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import ColorRGBA


def get_point(position, scale=1.0):
    pt = Point()
    if position is None:
        pt.x = 0.0
        pt.y = 0.0
        pt.z = 0.0
    elif('x' in dir(position)):
        pt.x = position.x
        pt.y = position.y
        pt.z = position.z
    else:
        pt.x = position[0]
        pt.y = position[1]
        pt.z = position[2]

    pt.x /= scale
    pt.y /= scale
    pt.z /= scale

    return pt


def get_quat(orientation):
    quat = Quaternion()
    if orientation is None:
        quat.x = 0.0
        quat.y = 0.0
        quat.z = 0.0
        quat.w = 1.0
    elif('x' in dir(orientation)):
        quat.w = orientation.w
        quat.x = orientation.x
        quat.y = orientation.y
        quat.z = orientation.z
    elif len(orientation) == 4:
        quat.x = orientation[0]
        quat.y = orientation[1]
        quat.z = orientation[2]
        quat.w = orientation[3]
    else:
        q2 = tf.transformations.quaternion_from_euler(orientation[0], orientation[1], orientation[2])
        quat.x = q2[0]
        quat.y = q2[1]
        quat.z = q2[2]
        quat.w = q2[3]
    return quat


def get_color(color):
    rgba = ColorRGBA()
    if color is None:
        color = [1.0] * 4

    if hasattr(color, 'x'):
        rgba.r = getattr(color, 'r', 1.0)
        rgba.g = getattr(color, 'g', 1.0)
        rgba.b = getattr(color, 'b', 1.0)
        rgba.a = getattr(color, 'a', 1.0)
    elif len(color) == 4:
        rgba.r = color[0]
        rgba.g = color[1]
        rgba.b = color[2]
        rgba.a = color[3]
    else:
        rgba.r = color[0]
        rgba.g = color[1]
        rgba.b = color[2]
        rgba.a = 1.0
    return rgba


class MarkerGenerator:
    def __init__(self):
        self.reset()

    def reset(self):
        self.counter = 0
        self.frame_id = ''
        self.ns = 'marker'
        self.type = 0
        self.action = Marker.ADD
        self.scale = [1.0] * 3
        self.color = None
        self.points = []
        self.colors = []
        self.text = ''
        self.lifetime = 0.0

    def marker(self, position=None, orientation=None, points=None, colors=None, scale=1.0, color=None):
        mark = Marker()
        mark.header.stamp = rospy.Time.now()
        mark.header.frame_id = self.frame_id
        mark.ns = self.ns
        mark.type = self.type
        mark.id = self.counter
        mark.action = self.action
        mark.scale.x = self.scale[0]
        mark.scale.y = self.scale[1]
        mark.scale.z = self.scale[2]
        if color:
            mark.color = get_color(color)
        else:
            mark.color = get_color(self.color)
        mark.lifetime = rospy.Duration(self.lifetime)

        if points is not None:
            mark.points = []
            for point in points:
                mark.points.append(get_point(point, scale))
        if colors is not None:
            mark.colors = colors

        if position is not None or orientation is not None:
            mark.pose.position = get_point(position, scale)
            mark.pose.orientation = get_quat(orientation)
        else:
            mark.pose.orientation.w = 1.0

        self.counter += 1
        return mark
