#!/usr/bin/env python3
import math
import unittest
import rospy
import actionlib
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from tortoisebot_waypoints.msg import WaypointActionAction, WaypointActionGoal

def yaw_from_quat(q):
    num = 2.0 * (q.w * q.z + q.x * q.y)
    den = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(num, den)

def norm(a):
    return math.atan2(math.sin(a), math.cos(a))

class WaypointTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node("tortoisebot_waypoints_tests", anonymous=True)
        cls._odom = None
        rospy.Subscriber("/odom", Odometry, lambda m: setattr(cls, "_odom", m))
        cls.client = actionlib.SimpleActionClient(
            "tortoisebot_as", WaypointActionAction
        )
        assert cls.client.wait_for_server(rospy.Duration(60)), "Action server not available"

        # wait for at least one odom
        t0 = rospy.Time.now()
        r = rospy.Rate(20)
        while cls._odom is None and rospy.Time.now() - t0 < rospy.Duration(60):
            r.sleep()
        assert cls._odom is not None, "No /odom received"

    def _send_goal(self, x, y, z=0.0):
        g = WaypointActionGoal()
        g.position = Point(x=x, y=y, z=z)
        self.client.send_goal(g)
        ok = self.client.wait_for_result(rospy.Duration(180))
        self.assertTrue(ok, "Action did not finish")

    def test_final_position(self):
        gx = rospy.get_param("~goal_x", rospy.get_param("/goal_x", 1.0))
        gy = rospy.get_param("~goal_y", rospy.get_param("/goal_y", 0.0))
        ex = rospy.get_param("~expected_x", rospy.get_param("/expected_x", 1.0))
        ey = rospy.get_param("~expected_y", rospy.get_param("/expected_y", 0.0))
        tol = rospy.get_param("~pos_tol", rospy.get_param("/pos_tol", 0.25))

        self._send_goal(gx, gy)
        p = self._odom.pose.pose.position
        dist = math.hypot(ex - p.x, ey - p.y)
        rospy.loginfo(f"[pos] expected=({ex:.2f},{ey:.2f}) got=({p.x:.2f},{p.y:.2f}) dist={dist:.3f}")
        self.assertLessEqual(dist, tol, "Position outside tolerance")

    def test_final_yaw(self):
        gx = rospy.get_param("~goal_x", rospy.get_param("/goal_x", 1.0))
        gy = rospy.get_param("~goal_y", rospy.get_param("/goal_y", 0.0))
        tol_deg = rospy.get_param("~yaw_tol_deg", rospy.get_param("/yaw_tol_deg", 20.0))
        tol = math.radians(tol_deg)

        self._send_goal(gx, gy)
        pose = self._odom.pose.pose
        yaw = yaw_from_quat(pose.orientation)
        desired = math.atan2(gy - pose.position.y, gx - pose.position.x)
        err = abs(norm(desired - yaw))
        rospy.loginfo(f"[yaw] desired={desired:.2f} got={yaw:.2f} err={err:.2f} rad")
        self.assertLessEqual(err, tol, "Yaw outside tolerance")

if __name__ == "__main__":
    import rostest
    rostest.rosrun("tortoisebot_waypoints", "test_waypoints", WaypointTests)