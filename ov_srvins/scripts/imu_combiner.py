#!/usr/bin/env python3
"""
Combines separate gyro and accel topics from a RealSense D435I ROS1 bag into
a single sensor_msgs/Imu topic expected by sqrtVINS / OpenVINS.

Subscribes:
  /device_0/sensor_2/Gyro_0/imu/data   -- angular_velocity filled, accel = 0
  /device_0/sensor_2/Accel_0/imu/data  -- linear_acceleration filled, gyro = 0

Publishes:
  /imu0  -- both fields filled (accel linearly interpolated to gyro timestamps)

Strategy: gyro messages drive output rate (~200 Hz).  For each gyro message we
linearly interpolate the two accel readings that bracket its timestamp.  If no
bracketing pair is available yet (startup lag) we fall back to the nearest
available accel measurement.
"""

import rospy
from sensor_msgs.msg import Imu
from collections import deque
import threading


class ImuCombiner:

    ACCEL_BUF_SIZE = 200  # ~1 s at 200 Hz

    def __init__(self):
        self._lock = threading.Lock()
        self._accel_buf = deque(maxlen=self.ACCEL_BUF_SIZE)

        gyro_topic = rospy.get_param("~gyro_topic",
                                     "/device_0/sensor_2/Gyro_0/imu/data")
        accel_topic = rospy.get_param("~accel_topic",
                                      "/device_0/sensor_2/Accel_0/imu/data")
        imu_topic = rospy.get_param("~imu_topic", "/imu0")

        self._pub = rospy.Publisher(imu_topic, Imu, queue_size=1000)

        # Large queues so rosbag playback never drops messages.
        rospy.Subscriber(accel_topic, Imu, self._accel_cb, queue_size=2000)
        rospy.Subscriber(gyro_topic,  Imu, self._gyro_cb,  queue_size=2000)

        rospy.loginfo("[imu_combiner] gyro  -> %s", gyro_topic)
        rospy.loginfo("[imu_combiner] accel -> %s", accel_topic)
        rospy.loginfo("[imu_combiner] out   -> %s", imu_topic)

    # ------------------------------------------------------------------
    def _accel_cb(self, msg):
        with self._lock:
            self._accel_buf.append(msg)

    # ------------------------------------------------------------------
    def _gyro_cb(self, msg):
        with self._lock:
            buf = self._accel_buf
            n = len(buf)
            if n == 0:
                return  # no accel data yet

            t = msg.header.stamp.to_sec()
            t_newest_accel = buf[-1].header.stamp.to_sec()

            # Discard stale buffer caused by a recording gap (e.g. RealSense SDK
            # startup burst followed by silence).  If the newest accel is more
            # than 0.5 s older than the gyro timestamp, the buffer is stale —
            # wait until fresh accel messages have arrived.
            if t - t_newest_accel > 0.5:
                return

            # Find bracketing pair
            a0 = a1 = None
            for i in range(n - 1):
                t0 = buf[i].header.stamp.to_sec()
                t1 = buf[i + 1].header.stamp.to_sec()
                if t0 <= t <= t1:
                    a0, a1 = buf[i], buf[i + 1]
                    break

            if a0 is not None:
                dt = a1.header.stamp.to_sec() - a0.header.stamp.to_sec()
                alpha = (t - a0.header.stamp.to_sec()) / dt if dt > 0.0 else 0.0
            else:
                # Fall back: nearest neighbour (accel arrived slightly late)
                nearest = min(buf, key=lambda a: abs(a.header.stamp.to_sec() - t))
                a0 = a1 = nearest
                alpha = 0.0

        ax = a0.linear_acceleration.x + alpha * (a1.linear_acceleration.x - a0.linear_acceleration.x)
        ay = a0.linear_acceleration.y + alpha * (a1.linear_acceleration.y - a0.linear_acceleration.y)
        az = a0.linear_acceleration.z + alpha * (a1.linear_acceleration.z - a0.linear_acceleration.z)

        out = Imu()
        out.header = msg.header
        out.header.frame_id = "imu"

        out.angular_velocity = msg.angular_velocity
        out.angular_velocity_covariance = msg.angular_velocity_covariance

        out.linear_acceleration.x = ax
        out.linear_acceleration.y = ay
        out.linear_acceleration.z = az
        out.linear_acceleration_covariance = a0.linear_acceleration_covariance

        # No orientation estimate available
        out.orientation_covariance[0] = -1.0

        self._pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("imu_combiner")
    ImuCombiner()
    rospy.spin()
