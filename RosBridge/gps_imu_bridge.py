#!/usr/bin/env python3
"""ROS 2 bridge for the Domain-Expansion Pi stream.

Listens for the same JSON-over-UDP messages gps.py already sends, and
republishes them as standard ROS 2 topics so RViz can draw them and
robot_localization can fuse them. Nothing on the Pi has to change.

Publishes:
    /imu/data   sensor_msgs/Imu         orientation from heading, world-frame accel
    /gps/fix    sensor_msgs/NavSatFix   raw lat/lon/alt
    /odom       nav_msgs/Odometry       local ENU metres from the first fix
    TF          map -> base_link        so RViz has a frame to render into

Frame conventions (REP-103): ROS uses ENU with x=East, y=North, z=Up, and yaw
measured counter-clockwise from East. The project's heading is a compass
bearing (0=North, increasing clockwise), so the two are NOT the same number -
see compass_to_enu_yaw() below.

Run (inside WSL, with ROS sourced):
    python3 gps_imu_bridge.py
    python3 gps_imu_bridge.py --ros-args -p port:=5002
"""

import json
import math
import queue
import socket
import threading

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from tf2_ros import TransformBroadcaster

METERS_PER_DEGREE_LAT = 111320.0


def compass_to_enu_yaw(heading_deg):
    """Compass bearing (0=North, clockwise) -> ROS ENU yaw (0=East, CCW), in
    radians. These differ by both an origin shift and a direction flip, so
    getting this wrong rotates and mirrors the whole track."""
    return math.radians((90.0 - heading_deg) % 360.0)


def yaw_to_quaternion(yaw_rad):
    """Quaternion for a rotation about Z only."""
    return (0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0))


class GpsImuBridge(Node):
    def __init__(self):
        super().__init__("gps_imu_bridge")

        self.declare_parameter("port", 5002)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", True)

        self.port = self.get_parameter("port").value
        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.publish_tf = self.get_parameter("publish_tf").value

        self.imu_pub = self.create_publisher(Imu, "imu/data", 10)
        self.fix_pub = self.create_publisher(NavSatFix, "gps/fix", 10)
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self._origin_lat = None
        self._origin_lon = None
        self._origin_alt = 0.0
        self._heading_deg = 0.0
        self._position = (0.0, 0.0, 0.0)
        self._imu_count = 0
        self._fix_count = 0

        # The UDP thread only enqueues; all publishing happens on the ROS
        # executor thread via the timers below.
        self._inbox = queue.Queue()
        self._running = True
        threading.Thread(target=self._udp_loop, daemon=True).start()
        self.create_timer(0.005, self._drain_inbox)

        # TF goes out on a steady timer rather than only when a GPS fix lands.
        # Publishing it from _handle_fix alone meant an IMU-only stream (or
        # any gap before the first fix) left RViz with no frame at all -
        # "Fixed Frame [map] does not exist". 1Hz TF would also be uselessly
        # sparse for rendering even once fixes were arriving.
        self.create_timer(0.05, self._publish_tf)

        # Cheap "is anything arriving at all" signal - the first thing you
        # want to know when RViz is empty.
        self.create_timer(5.0, self._log_counts)

        self.get_logger().info(f"Listening for Pi messages on UDP :{self.port}")

    def _udp_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("", self.port))
        while self._running:
            try:
                data, _ = sock.recvfrom(65535)
                self._inbox.put(data)
            except OSError as e:
                self.get_logger().warn(f"UDP receive error: {e}")

    def _drain_inbox(self):
        while True:
            try:
                data = self._inbox.get_nowait()
            except queue.Empty:
                return
            try:
                msg = json.loads(data.decode("utf-8").strip())
            except (ValueError, UnicodeDecodeError):
                continue

            if msg.get("type") == "imu":
                self._handle_imu(msg)
            elif "lat" in msg:
                self._handle_fix(msg)

    def _log_counts(self):
        if self._imu_count == 0 and self._fix_count == 0:
            self.get_logger().warn(
                f"No messages received yet on UDP :{self.port}. If the Pi is definitely "
                "sending, check WSL networking (needs networkingMode=mirrored in .wslconfig)."
            )
        else:
            self.get_logger().info(
                f"received: {self._imu_count} imu, {self._fix_count} fixes"
            )

    def _publish_tf(self):
        """Publishes map->base_link from whatever state we have. Runs even
        with no GPS fixes at all, so RViz always has a frame to render into
        (position just stays at the origin until fixes start arriving)."""
        if not self.publish_tf:
            return
        if self._imu_count == 0 and self._fix_count == 0:
            return  # nothing received yet - don't invent a pose

        x, y, z = self._position
        qx, qy, qz, qw = yaw_to_quaternion(compass_to_enu_yaw(self._heading_deg))

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

    def _handle_imu(self, msg):
        self._imu_count += 1
        self._heading_deg = float(msg.get("heading", 0.0))
        yaw = compass_to_enu_yaw(self._heading_deg)
        qx, qy, qz, qw = yaw_to_quaternion(yaw)

        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = self.base_frame
        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz
        imu.orientation.w = qw
        # Only yaw is meaningful here (the Pi sends a heading, not a full
        # attitude), so roll/pitch get large covariances to tell any consumer
        # not to trust them.
        imu.orientation_covariance = [
            10.0, 0.0, 0.0,
            0.0, 10.0, 0.0,
            0.0, 0.0, 0.05,
        ]
        # accelEast/accelNorth are already world-frame ENU from the Pi, which
        # maps directly onto ROS x=East, y=North.
        imu.linear_acceleration.x = float(msg.get("accelEast", 0.0))
        imu.linear_acceleration.y = float(msg.get("accelNorth", 0.0))
        imu.linear_acceleration.z = 0.0
        imu.linear_acceleration_covariance = [
            0.5, 0.0, 0.0,
            0.0, 0.5, 0.0,
            0.0, 0.0, 0.5,
        ]
        # No angular velocity in the stream at all - flagged unavailable via
        # the REP-145 convention of -1 in the first covariance element.
        imu.angular_velocity_covariance[0] = -1.0
        self.imu_pub.publish(imu)

    def _handle_fix(self, msg):
        self._fix_count += 1
        lat = float(msg["lat"])
        lon = float(msg["lon"])
        alt = float(msg.get("alt", 0.0))
        stamp = self.get_clock().now().to_msg()

        fix = NavSatFix()
        fix.header.stamp = stamp
        fix.header.frame_id = self.base_frame
        fix.latitude = lat
        fix.longitude = lon
        fix.altitude = alt
        quality = int(msg.get("fixQuality", 0))
        fix.status.status = (
            NavSatStatus.STATUS_FIX if quality > 0 else NavSatStatus.STATUS_NO_FIX
        )
        fix.status.service = NavSatStatus.SERVICE_GPS
        # RTK fixed/float are far tighter than an uncorrected fix; give the
        # EKF something roughly honest to weight against.
        sigma = {4: 0.05, 5: 0.5}.get(quality, 3.0)
        fix.position_covariance = [
            sigma ** 2, 0.0, 0.0,
            0.0, sigma ** 2, 0.0,
            0.0, 0.0, (sigma * 3) ** 2,
        ]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.fix_pub.publish(fix)

        if self._origin_lat is None:
            self._origin_lat = lat
            self._origin_lon = lon
            self._origin_alt = alt
            self.get_logger().info(f"Origin set to {lat:.7f}, {lon:.7f}")

        # Flat-earth projection, same as the rest of the project uses.
        meters_per_degree_lon = METERS_PER_DEGREE_LAT * math.cos(math.radians(self._origin_lat))
        x = (lon - self._origin_lon) * meters_per_degree_lon   # East
        y = (lat - self._origin_lat) * METERS_PER_DEGREE_LAT   # North
        z = alt - self._origin_alt
        self._position = (x, y, z)

        yaw = compass_to_enu_yaw(self._heading_deg)
        qx, qy, qz, qw = yaw_to_quaternion(yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.map_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance[0] = sigma ** 2
        odom.pose.covariance[7] = sigma ** 2
        odom.pose.covariance[14] = (sigma * 3) ** 2
        odom.pose.covariance[35] = 0.05
        self.odom_pub.publish(odom)
        # TF is handled by the _publish_tf timer, not here - see its comment.


def main():
    rclpy.init()
    node = GpsImuBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._running = False
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
