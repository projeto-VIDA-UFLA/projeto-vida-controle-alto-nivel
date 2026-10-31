"""Simulador do VIDA: substitui ESP32 + GPS + EKF com os mesmos topicos.

    ros2 run vida_navigation sim_vehicle --ros-args -p path_file:=pista.csv

Assina as referencias (/vida/control/*/reference) e o enable logico e publica
o que o Stanley e o mode_manager leem no veiculo real:
    /robot_state/odom, /robot_state/status   (EKF)
    /vida/sensors/gps/{fix,status}           (receptor, antena no eixo traseiro)
    /vida/safety/status, /vida/sensors/steering (ESP32)
    /sim/ground_truth                        (pose real, so no simulador)
e atende /robot_state_ekf_node/reset como o EKF: origem apos 3 fixes parado,
heading depois de andar 2 m com RTK FIXED.

Cenarios de falha em tempo de execucao:
    ros2 param set /sim_vehicle gnss_scenario float     # fixed|float|no_rtk|lost
    ros2 param set /sim_vehicle inject_inhibit 512      # ESP32 bloqueando
"""
import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_srvs.srv import Trigger
from vida_interfaces.msg import (DriveReference, EkfStatus, GnssStatus, LogicalEnable,
                                 SafetyStatus, Steering, SteeringReference)

from vida_navigation.geo import Origin, to_geodetic, to_local, wrap_angle
from vida_navigation.path import PathFile
from vida_navigation.vehicle_model import VehicleModel, VehicleParams, VehicleState

RTK_BY_SCENARIO = {"fixed": GnssStatus.RTK_FIXED, "float": GnssStatus.RTK_FLOAT,
                   "no_rtk": GnssStatus.RTK_NONE, "lost": GnssStatus.RTK_NONE}


class EkfEmulator:
    """Fases do robot_state_ekf e o erro tipico da estimativa.

    Entre dois fixes (gps_rate_hz) a posicao e integrada pelos encoders com
    erro de escala `encoder_scale_error`; a cada fix ela volta ao GPS (o dente
    de serra de ate 0,35 m visto no teste de 12/10 com GPS a 1 Hz e +7%).
    """

    def __init__(self, world, gps_rate_hz=2.0, origin_samples=3, heading_min_distance_m=2.0,
                 heading_timeout_s=20.0, encoder_scale_error=0.0, gps_noise_m=0.0, seed=0):
        self.world = world
        self.gps_period = 1.0 / gps_rate_hz
        self.origin_samples = origin_samples
        self.heading_min_distance = heading_min_distance_m
        self.heading_timeout = heading_timeout_s
        self.scale_error = encoder_scale_error
        self.gps_noise = gps_noise_m
        self.rng = np.random.default_rng(seed)
        self.origin_id = 0
        self.reset(0.0)

    def reset(self, now):
        self.phase = EkfStatus.PHASE_WAITING_ORIGIN
        self.heading_source = EkfStatus.HEADING_NONE
        self.origin = None
        self.origin_fixes = 0
        self.heading_start = None
        self.heading_start_time = None
        self.last_fix_time = -1e9
        self.fix_true = None          # (x, y) real no ultimo fix
        self.fix_measured = None      # (x, y) medido (com ruido) no ultimo fix
        self.yaw_offset = 0.0

    def _fix(self, state, now):
        self.last_fix_time = now
        noise = self.rng.normal(0.0, self.gps_noise, 2) if self.gps_noise > 0 else (0.0, 0.0)
        self.fix_true = (state.x, state.y)
        self.fix_measured = (state.x + noise[0], state.y + noise[1])

    def update(self, state, now, rtk_status, gps_alive, stationary):
        """Avanca as fases; retorna True quando chegou um fix novo."""
        new_fix = gps_alive and now - self.last_fix_time >= self.gps_period
        if not new_fix:
            return False
        self._fix(state, now)
        if self.phase == EkfStatus.PHASE_WAITING_ORIGIN:
            if rtk_status == GnssStatus.RTK_NONE or not stationary:
                self.origin_fixes = 0
                return True
            self.origin_fixes += 1
            if self.origin_fixes >= self.origin_samples:
                lat, lon = to_geodetic(self.world, *self.fix_measured)
                self.origin = Origin(float(lat), float(lon))
                self.origin_id += 1
                self.phase = EkfStatus.PHASE_WAITING_HEADING
                self.heading_start, self.heading_start_time = self.fix_measured, now
        elif self.phase == EkfStatus.PHASE_WAITING_HEADING:
            if rtk_status != GnssStatus.RTK_FIXED:
                self.heading_start, self.heading_start_time = self.fix_measured, now
                return True
            dx = self.fix_measured[0] - self.heading_start[0]
            dy = self.fix_measured[1] - self.heading_start[1]
            if math.hypot(dx, dy) >= self.heading_min_distance:
                self.phase = EkfStatus.PHASE_RUNNING
                self.heading_source = EkfStatus.HEADING_GPS
            elif now - self.heading_start_time > self.heading_timeout:
                self.phase = EkfStatus.PHASE_RUNNING
                self.heading_source = EkfStatus.HEADING_PARAMETER
                self.yaw_offset = -state.yaw      # initial_heading_deg = 0: rumo errado
        return True

    def estimate(self, state):
        """Pose estimada (x, y, yaw) no frame local do EKF (ENU na origem)."""
        fx, fy = self.fix_measured
        tx, ty = self.fix_true
        k = 1.0 + self.scale_error
        wx = fx + k * (state.x - tx)
        wy = fy + k * (state.y - ty)
        yaw = state.yaw + self.yaw_offset
        lat, lon = to_geodetic(self.world, wx, wy)
        x, y = to_local(self.origin, lat, lon)
        return float(x), float(y), float(wrap_angle(yaw))

    def to_ekf_frame(self, x, y):
        lat, lon = to_geodetic(self.world, x, y)
        ex, ey = to_local(self.origin, lat, lon)
        return float(ex), float(ey)


class SimVehicle(Node):
    def __init__(self):
        super().__init__("sim_vehicle")
        dp = self.declare_parameter
        path_file = dp("path_file", "").value
        rate = dp("rate_hz", 100.0).value
        self.odom_rate = dp("odom_rate_hz", 50.0).value
        self.gnss_scenario = dp("gnss_scenario", "fixed").value
        dp("inject_inhibit", 0)
        self.use_logical_enable = dp("use_logical_enable", False).value
        self.logical_enable_timeout = 0.3

        params = VehicleParams(
            # 1,42 m: entre-eixos efetivo medido no chao em 30/10 (o veiculo vira menos
            # que a geometria de 1,125 m preve); o Stanley usa 1,125 m para a posicao
            # do eixo dianteiro e 1,42 m no feedforward.
            wheelbase_m=dp("wheelbase_m", 1.42).value,
            steering_rate_max_deg_s=dp("steering_rate_max_deg_s", 60.0).value,
            drive_time_constant_s=dp("drive_time_constant_s", 1.2).value)

        # Pose inicial: no inicio do caminho, com deslocamentos para testar a entrada.
        if path_file:
            path_file = PathFile.load(path_file)
            self.world = path_file.start_origin
            local = path_file.to_local(self.world)
            yaw0 = float(local.heading[0])
        else:
            self.world = Origin(dp("world_latitude_deg", -21.2293928).value,
                                dp("world_longitude_deg", -44.9732956).value)
            yaw0 = 0.0
        back = dp("start_back_m", 0.0).value
        left = dp("start_left_m", 0.0).value
        yaw = yaw0 + math.radians(dp("start_yaw_deg", 0.0).value)
        x0 = -back * math.cos(yaw0) - left * math.sin(yaw0)
        y0 = -back * math.sin(yaw0) + left * math.cos(yaw0)
        self.model = VehicleModel(params, VehicleState(x=x0, y=y0, yaw=yaw))
        self.model.logical_enable = not self.use_logical_enable
        self.last_logical_enable = -1e9

        self.ekf = EkfEmulator(
            self.world, gps_rate_hz=dp("gps_rate_hz", 2.0).value,   # ZED-F9R a 2 Hz desde 31/10
            encoder_scale_error=dp("encoder_scale_error", 0.0).value,
            gps_noise_m=dp("gps_noise_m", 0.0).value)
        self.correction_age = 1.0
        self.last_publish_odom = -1e9

        self.create_subscription(SteeringReference, "/vida/control/steering/reference",
                                 self.on_steering_reference, 10)
        self.create_subscription(DriveReference, "/vida/control/drive/reference",
                                 self.on_drive_reference, 10)
        self.create_subscription(LogicalEnable, "/vida/safety/logical_enable", self.on_logical_enable, 10)
        self.odom_pub = self.create_publisher(Odometry, "/robot_state/odom", 10)
        self.truth_pub = self.create_publisher(Odometry, "/sim/ground_truth", 10)
        self.status_pub = self.create_publisher(EkfStatus, "/robot_state/status", 10)
        self.gnss_pub = self.create_publisher(GnssStatus, "/vida/sensors/gps/status", 10)
        self.fix_pub = self.create_publisher(NavSatFix, "/vida/sensors/gps/fix", 10)
        self.safety_pub = self.create_publisher(SafetyStatus, "/vida/safety/status", 10)
        self.steering_pub = self.create_publisher(Steering, "/vida/sensors/steering", 10)
        self.create_service(Trigger, "/robot_state_ekf_node/reset", self.on_reset)

        self.t0 = self.get_clock().now()
        self.last_step = 0.0
        self.create_timer(1.0 / rate, self.on_timer)
        self.create_timer(0.1, self.publish_slow)
        self.get_logger().info(
            f"Simulador pronto em ({x0:.1f}, {y0:.1f}) m, rumo {math.degrees(yaw):.0f} graus"
            + (f" (caminho '{path_file.meta.get('name', '')}')" if path_file else ""))

    def now(self):
        return (self.get_clock().now() - self.t0).nanoseconds * 1e-9

    def on_steering_reference(self, msg):
        self.model.commands.steering_deg = msg.angle_deg
        self.model.commands.steering_time = self.now()

    def on_drive_reference(self, msg):
        c = self.model.commands
        c.speed_m_s, c.accel_m_s2, c.brake = msg.velocity_m_s, msg.acceleration_m_s2, msg.brake_command
        c.drive_time = self.now()

    def on_logical_enable(self, msg):
        if self.use_logical_enable:
            self.model.logical_enable = msg.enabled
            self.last_logical_enable = self.now()

    def on_reset(self, request, response):
        self.ekf.reset(self.now())
        response.success = True
        response.message = "EKF simulado reiniciado: aguardando origem"
        self.get_logger().info(response.message)
        return response

    def on_timer(self):
        now = self.now()
        dt = min(now - self.last_step, 0.05)
        self.last_step = now
        self.gnss_scenario = self.get_parameter("gnss_scenario").value
        self.model.injected_inhibit = int(self.get_parameter("inject_inhibit").value)
        if self.use_logical_enable and now - self.last_logical_enable > self.logical_enable_timeout:
            self.model.logical_enable = False
        self.model.step(now, dt)

        rtk = RTK_BY_SCENARIO.get(self.gnss_scenario, GnssStatus.RTK_FIXED)
        alive = self.gnss_scenario != "lost"
        if self.ekf.update(self.model.state, now, rtk, alive, self.model.state.speed < 0.02):
            self.publish_gnss(rtk, alive)
        if self.ekf.phase == EkfStatus.PHASE_RUNNING and now - self.last_publish_odom >= 1.0 / self.odom_rate:
            self.last_publish_odom = now
            self.publish_odom()

    def _odom(self, x, y, yaw, frame="odom"):
        s = self.model.state
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.twist.twist.linear.x = s.speed
        msg.twist.twist.angular.z = self.model.yaw_rate
        return msg

    def publish_odom(self):
        x, y, yaw = self.ekf.estimate(self.model.state)
        odom = self._odom(x, y, yaw)
        odom.twist.twist.linear.x *= 1.0 + self.ekf.scale_error
        self.odom_pub.publish(odom)
        s = self.model.state
        tx, ty = self.ekf.to_ekf_frame(s.x, s.y)
        self.truth_pub.publish(self._odom(tx, ty, s.yaw))

    def publish_gnss(self, rtk, alive):
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = "gps"
        fix.status.status = NavSatStatus.STATUS_GBAS_FIX if rtk != GnssStatus.RTK_NONE else NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        lat, lon = to_geodetic(self.world, *self.ekf.fix_measured)
        fix.latitude, fix.longitude, fix.altitude = float(lat), float(lon), 900.0
        sigma = {GnssStatus.RTK_FIXED: 0.014, GnssStatus.RTK_FLOAT: 0.25}.get(rtk, 1.5)
        fix.position_covariance = [sigma ** 2, 0.0, 0.0, 0.0, sigma ** 2, 0.0, 0.0, 0.0, 4 * sigma ** 2]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.fix_pub.publish(fix)

        msg = GnssStatus()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.fix_type = 3
        msg.rtk_status = rtk
        msg.satellites = 22
        msg.gnss_fix_ok = alive
        msg.differential_solution = rtk != GnssStatus.RTK_NONE
        msg.corrections_received = self.gnss_scenario in ("fixed", "float")
        self.correction_age = 1.0 if msg.corrections_received else self.correction_age + self.ekf.gps_period
        msg.correction_age_s = float(self.correction_age)
        msg.horizontal_accuracy_m = {GnssStatus.RTK_FIXED: 0.014, GnssStatus.RTK_FLOAT: 0.25}.get(rtk, 1.5)
        msg.vertical_accuracy_m = 2.0 * msg.horizontal_accuracy_m
        self.gnss_pub.publish(msg)

    def publish_slow(self):
        now = self.now()
        stamp = self.get_clock().now().to_msg()
        ekf, s = self.ekf, self.model.state

        status = EkfStatus()
        status.timestamp = stamp
        status.phase = ekf.phase
        status.heading_source = ekf.heading_source
        status.origin_valid = ekf.origin is not None
        if ekf.origin is not None:
            status.origin_latitude_deg = ekf.origin.latitude_deg
            status.origin_longitude_deg = ekf.origin.longitude_deg
        status.origin_id = ekf.origin_id
        status.stationary = s.speed < 0.02
        status.bias_calibration_samples = 100 if ekf.phase != EkfStatus.PHASE_WAITING_ORIGIN else 0
        status.gps_rtk_status = RTK_BY_SCENARIO.get(self.gnss_scenario, GnssStatus.RTK_FIXED)
        self.status_pub.publish(status)

        flags = self.model.inhibit_flags(now)
        safety = SafetyStatus()
        safety.timestamp = stamp
        safety.state = SafetyStatus.STATE_READY if flags == 0 else SafetyStatus.STATE_DISABLED
        safety.physical_enable = True
        safety.logical_enable = self.model.logical_enable
        safety.logical_enable_received = True
        safety.logical_enable_fresh = True
        safety.drive_reference_fresh = not flags & VehicleModel.INHIBIT_DRIVE_REFERENCE_NOT_FRESH
        safety.steering_reference_fresh = not flags & VehicleModel.INHIBIT_STEERING_REFERENCE_NOT_FRESH
        safety.motion_permitted = flags == 0
        safety.inhibit_flags = flags
        self.safety_pub.publish(safety)

        steering = Steering()
        steering.timestamp = stamp
        steering.raw_angle_deg = steering.filtered_angle_deg = float(s.steering_deg)
        self.steering_pub.publish(steering)


def main():
    rclpy.init()
    node = SimVehicle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
