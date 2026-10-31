"""Gerenciador de modos: manual (gamepad) <-> automatico (Stanley).

    ros2 run vida_navigation mode_manager --ros-args -p path_file:=pista.csv

E o UNICO publicador de /vida/control/{steering,drive}/reference. O gamepad
publica as referencias manuais em /gamepad/* e os pedidos de modo em
/navigation/auto_request (triangulo 5 s = true, quadrado 5 s = false).

    MANUAL --pedido--> PREPARANDO --rodas em 0--> CALIBRANDO --origem--> MANOBRA
    MANOBRA --heading GPS + alinhado + perto do inicio--> AUTOMATICO
    AUTOMATICO --fim do caminho / quadrado / aborto--> PARANDO --parado--> MANUAL

Para entrar: veiculo parado, ESP32 liberando movimento, GPS com correcoes RTK
(FLOAT aceito) e a antena a menos de start_max_distance_m do inicio do
caminho. Abortos (freio total e volta ao manual): L2, gamepad perdido, ESP32
bloqueando, GPS sem comunicacao, correcoes RTK perdidas, EKF parado ou
reiniciado, desvio lateral acima de max_cross_track_m e tempos limite.
"""
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger
from vida_interfaces.msg import (DriveReference, EkfStatus, GnssStatus, NavigationMode, SafetyStatus,
                                 StanleyStatus, Steering, SteeringReference)

from vida_navigation.geo import to_local, wrap_angle
from vida_navigation.path import PathFile
from vida_navigation.ros_common import ekf_ready, origin_from_status, yaw_from_quaternion

M = NavigationMode
MODE_NAMES = {M.MODE_MANUAL: "MANUAL", M.MODE_PREPARING: "PREPARANDO", M.MODE_CALIBRATING: "CALIBRANDO",
              M.MODE_MANEUVER: "MANOBRA", M.MODE_AUTOMATIC: "AUTOMATICO", M.MODE_STOPPING: "PARANDO"}


class Latest:
    """Ultima mensagem recebida e quando chegou."""

    def __init__(self, node):
        self.node, self.msg, self.time = node, None, None

    def __call__(self, msg):
        self.msg, self.time = msg, self.node.now()

    def age(self):
        return math.inf if self.time is None else self.node.now() - self.time

    def fresh(self, timeout):
        return self.age() < timeout


class ModeManager(Node):
    def __init__(self):
        super().__init__("mode_manager")
        dp = self.declare_parameter
        self.path_file = PathFile.load(dp("path_file", "").value)
        self.path_name = self.path_file.meta.get("name", "")
        self.wheelbase = dp("wheelbase_m", 1.125).value
        self.start_max_distance = dp("start_max_distance_m", 3.0).value
        # A 2,5 m/s, entrar com 1 m e 30 graus levava o erro a 1,46 m (aborto em
        # 1,5 m); com 0,5 m e 15 graus o pior caso fica em 0,64 m (simulador).
        self.max_heading_error = math.radians(dp("max_heading_error_deg", 15.0).value)
        self.max_enable_cross_track = dp("max_enable_cross_track_m", 0.5).value
        self.max_cross_track = dp("max_cross_track_m", 1.5).value
        # 2 m/s: abaixo disso a controladora de tracao pulsa (30/10). Com a rampa
        # de 0,4 m/s^2 a manobra anda ~5-8 m ate o EKF ter o rumo do GPS.
        self.maneuver_speed = dp("maneuver_speed_m_s", 2.0).value
        self.maneuver_max_distance = dp("maneuver_max_distance_m", 15.0).value
        self.maneuver_timeout = dp("maneuver_timeout_s", 20.0).value
        self.preparing_timeout = dp("preparing_timeout_s", 5.0).value
        self.calibrating_timeout = dp("calibrating_timeout_s", 15.0).value
        self.stopping_timeout = dp("stopping_timeout_s", 15.0).value
        self.steering_aligned_deg = dp("steering_aligned_deg", 1.5).value
        self.max_correction_age = dp("max_correction_age_s", 5.0).value
        self.gnss_timeout = dp("gnss_timeout_s", 2.5).value
        self.esp_block_tolerance = dp("esp_block_tolerance_s", 0.3).value
        self.blocked_since = None
        self.reference_timeout = 0.3      # gamepad, Stanley
        self.status_timeout = 0.5         # ESP32, EKF
        self.odom_timeout = 0.3

        self.gamepad_steering = Latest(self)
        self.gamepad_drive = Latest(self)
        self.safety = Latest(self)
        self.ekf_status = Latest(self)
        self.odom = Latest(self)
        self.gnss = Latest(self)
        self.fix = Latest(self)
        self.steering = Latest(self)
        self.stanley_status = Latest(self)
        self.stanley_steering = Latest(self)
        self.stanley_drive = Latest(self)
        sub = self.create_subscription
        sub(SteeringReference, "/gamepad/steering/reference", self.on_gamepad_steering, 10)
        sub(DriveReference, "/gamepad/drive/reference", self.on_gamepad_drive, 10)
        sub(Bool, "/navigation/auto_request", self.on_auto_request, 10)
        sub(SafetyStatus, "/vida/safety/status", self.safety, 10)
        sub(EkfStatus, "/robot_state/status", self.ekf_status, 10)
        sub(Odometry, "/robot_state/odom", self.odom, 10)
        sub(GnssStatus, "/vida/sensors/gps/status", self.gnss, 10)
        # O driver do ZED-F9R publica o fix como best-effort (SensorDataQoS): uma
        # assinatura reliable nao recebe nada dele (30/10: "sem posicao do GPS").
        sub(NavSatFix, "/vida/sensors/gps/fix", self.fix, qos_profile_sensor_data)
        sub(Steering, "/vida/sensors/steering", self.steering, 10)
        sub(StanleyStatus, "/navigation/stanley/status", self.stanley_status, 10)
        sub(SteeringReference, "/navigation/stanley/steering", self.stanley_steering, 10)
        sub(DriveReference, "/navigation/stanley/drive", self.stanley_drive, 10)
        self.steering_pub = self.create_publisher(SteeringReference, "/vida/control/steering/reference", 10)
        self.drive_pub = self.create_publisher(DriveReference, "/vida/control/drive/reference", 10)
        self.mode_pub = self.create_publisher(NavigationMode, "/navigation/mode", 10)
        self.ekf_reset = self.create_client(Trigger, "/robot_state_ekf_node/reset")
        self.stanley_enable = self.create_client(SetBool, "/stanley/enable")

        self.t0 = self.get_clock().now()
        self.mode = M.MODE_MANUAL
        self.mode_since = 0.0
        self.aborted = False
        self.reason = "inicio"
        self.hold_steering = 0.0
        self.wait_neutral_gamepad = False
        self.pending = None               # chamada de servico em andamento
        self.old_origin_id = None
        self.maneuver_start_fix = None
        self.auto_origin_id = None
        self.refused_count = 0
        self.create_timer(0.02, self.on_timer)
        self.create_timer(0.1, self.publish_mode)
        self.get_logger().info(f"Modo MANUAL; caminho '{self.path_name}' "
                               f"({'fechado' if self.path_file.closed else 'aberto'})")

    # ------------------------------------------------------------ utilitarios
    def now(self):
        return (self.get_clock().now() - self.t0).nanoseconds * 1e-9

    def set_mode(self, mode, reason, aborted=False):
        if mode == self.mode:
            return
        text = f"{MODE_NAMES[self.mode]} -> {MODE_NAMES[mode]}: {reason}"
        if aborted:
            self.get_logger().error(text)
        else:
            self.get_logger().warn(text)
        self.mode, self.mode_since, self.reason = mode, self.now(), reason
        if mode == M.MODE_STOPPING:
            self.aborted = aborted
            self.call_stanley(False)
        elif mode == M.MODE_PREPARING:
            self.aborted = False
        self.publish_mode()

    def abort(self, reason):
        self.set_mode(M.MODE_STOPPING, reason, aborted=True)

    def in_mode_for(self):
        return self.now() - self.mode_since

    def publish_refs(self, steering_deg, speed, brake=0.0, accel=0.0):
        stamp = self.get_clock().now().to_msg()
        steering = SteeringReference()
        steering.timestamp = stamp
        steering.angle_deg = float(steering_deg)
        drive = DriveReference()
        drive.timestamp = stamp
        drive.velocity_m_s, drive.acceleration_m_s2, drive.brake_command = float(speed), float(accel), float(brake)
        self.steering_pub.publish(steering)
        self.drive_pub.publish(drive)

    def call_stanley(self, enable, on_done=None):
        if not self.stanley_enable.service_is_ready():
            if enable:
                self.abort("no do Stanley nao responde")
            return
        future = self.stanley_enable.call_async(SetBool.Request(data=enable))
        if on_done is not None:
            future.add_done_callback(on_done)

    def path_local(self):
        status = self.ekf_status.msg
        origin = origin_from_status(status) if status is not None else None
        return None if origin is None else self.path_file.to_local(origin)

    # ------------------------------------------------------------ entradas
    def on_gamepad_steering(self, msg):
        self.gamepad_steering(msg)
        if self.mode == M.MODE_MANUAL:
            self.steering_pub.publish(msg)

    def on_gamepad_drive(self, msg):
        self.gamepad_drive(msg)
        if self.mode == M.MODE_MANUAL:
            if self.wait_neutral_gamepad:
                if msg.velocity_m_s > 0.05:
                    out = DriveReference()
                    out.timestamp, out.velocity_m_s, out.brake_command = msg.timestamp, 0.0, msg.brake_command
                    self.drive_pub.publish(out)
                    return
                self.wait_neutral_gamepad = False
                self.get_logger().info("R2 solto: gamepad no comando")
            self.drive_pub.publish(msg)
        elif msg.brake_command > 0.0 and self.mode != M.MODE_STOPPING:
            self.abort("L2 (freio) no gamepad")

    def on_auto_request(self, msg):
        if msg.data and self.mode == M.MODE_MANUAL:
            problem = self.entry_problem()
            if problem:
                self.reason = f"automatico recusado: {problem}"
                self.refused_count += 1
                self.get_logger().error(self.reason)
                self.publish_mode()
                return
            self.set_mode(M.MODE_PREPARING, "pedido pelo gamepad")
        elif not msg.data and self.mode not in (M.MODE_MANUAL, M.MODE_STOPPING):
            self.set_mode(M.MODE_STOPPING, "saida pedida pelo gamepad (quadrado)")

    # ------------------------------------------------------------ verificacoes
    def gnss_problem(self):
        if not self.gnss.fresh(self.gnss_timeout) or not self.gnss.msg.gnss_fix_ok:
            return "GPS sem comunicacao ou sem fix"
        g = self.gnss.msg
        if not g.corrections_received or g.correction_age_s > self.max_correction_age:
            return f"correcoes RTK perdidas (idade {g.correction_age_s:.1f} s)"
        if g.rtk_status == GnssStatus.RTK_NONE:
            return "GPS sem RTK"
        return None

    def common_problem(self):
        """Condicoes que valem em todo o automatico (da preparacao ao Stanley)."""
        if not self.gamepad_drive.fresh(self.reference_timeout):
            return "gamepad perdido"
        if not self.safety.fresh(self.status_timeout):
            return "ESP32 sem status"
        # Um bloqueio curto do ESP32 (30/10: 60 ms de enable "atrasado") nao
        # aborta; so se persistir por esp_block_tolerance_s.
        if self.safety.msg.motion_permitted or self.in_mode_for() <= 0.5:
            self.blocked_since = None
        else:
            if self.blocked_since is None:
                self.blocked_since = self.now()
            if self.now() - self.blocked_since > self.esp_block_tolerance:
                return f"ESP32 bloqueando o movimento (flags {self.safety.msg.inhibit_flags})"
        if not self.ekf_status.fresh(self.status_timeout):
            return "EKF sem status"
        return self.gnss_problem()

    def entry_problem(self):
        if not self.gamepad_drive.fresh(self.reference_timeout):
            return "gamepad sem referencias"
        if not self.safety.fresh(self.status_timeout) or not self.safety.msg.motion_permitted:
            return "ESP32 nao libera o movimento"
        if not self.ekf_status.fresh(self.status_timeout):
            return "EKF sem status"
        if not self.ekf_status.msg.stationary:
            return "veiculo em movimento"
        if not self.stanley_enable.service_is_ready() or not self.ekf_reset.service_is_ready():
            return "Stanley ou EKF sem servico"
        problem = self.gnss_problem()
        if problem:
            return problem
        if not self.fix.fresh(self.gnss_timeout):
            return "sem posicao do GPS"
        start = self.path_file.start_origin
        x, y = to_local(start, self.fix.msg.latitude, self.fix.msg.longitude)
        distance = math.hypot(float(x), float(y))
        if distance > self.start_max_distance:
            return f"longe do inicio do caminho ({distance:.1f} m > {self.start_max_distance:.1f} m)"
        return None

    # ------------------------------------------------------------ maquina de estados
    def on_timer(self):
        mode = self.mode
        if mode == M.MODE_MANUAL:
            return
        if mode != M.MODE_STOPPING:
            problem = self.common_problem()
            if problem:
                self.abort(problem)
                return
        handler = {M.MODE_PREPARING: self.run_preparing, M.MODE_CALIBRATING: self.run_calibrating,
                   M.MODE_MANEUVER: self.run_maneuver, M.MODE_AUTOMATIC: self.run_automatic,
                   M.MODE_STOPPING: self.run_stopping}[mode]
        handler()

    def run_preparing(self):
        self.publish_refs(0.0, 0.0)
        aligned = self.steering.fresh(self.status_timeout) and \
            abs(self.steering.msg.filtered_angle_deg) < self.steering_aligned_deg
        if aligned and self.in_mode_for() > 0.5 and self.pending is None:
            self.old_origin_id = self.ekf_status.msg.origin_id
            self.pending = self.ekf_reset.call_async(Trigger.Request())
        if self.pending is not None and self.pending.done():
            ok = self.pending.result() is not None and self.pending.result().success
            self.pending = None
            if ok:
                self.set_mode(M.MODE_CALIBRATING, "rodas alinhadas, EKF reiniciado")
            else:
                self.abort("EKF recusou o reset")
        elif self.in_mode_for() > self.preparing_timeout:
            self.abort("rodas nao alinharam em 0 grau")

    def run_calibrating(self):
        self.publish_refs(0.0, 0.0)
        s = self.ekf_status.msg
        if s.origin_valid and s.origin_id != self.old_origin_id and s.phase == EkfStatus.PHASE_WAITING_HEADING:
            self.maneuver_start_fix = self.fix.msg
            self.set_mode(M.MODE_MANEUVER, f"origem #{s.origin_id} definida, bias calibrado")
        elif s.origin_id != self.old_origin_id and s.phase == EkfStatus.PHASE_RUNNING:
            self.abort("EKF iniciou sem a manobra (heading por parametro)")
        elif self.in_mode_for() > self.calibrating_timeout:
            self.abort("EKF nao definiu a origem (GPS parado com RTK?)")

    def run_maneuver(self):
        self.publish_refs(0.0, self.maneuver_speed)
        s = self.ekf_status.msg
        start, fix = self.maneuver_start_fix, self.fix.msg
        distance = 0.0
        if start is not None and fix is not None:
            origin = origin_from_status(s)
            x, y = to_local(origin, fix.latitude, fix.longitude)
            x0, y0 = to_local(origin, start.latitude, start.longitude)
            distance = math.hypot(float(x - x0), float(y - y0))
        if s.origin_id != self.old_origin_id and ekf_ready(s) and self.odom.fresh(self.odom_timeout):
            self.verify_and_engage()
        elif s.phase == EkfStatus.PHASE_RUNNING and s.heading_source != EkfStatus.HEADING_GPS:
            self.abort("EKF iniciou com heading por parametro (sem RTK FIXED na manobra?)")
        elif distance > self.maneuver_max_distance:
            self.abort(f"manobra passou de {self.maneuver_max_distance:.0f} m sem heading")
        elif self.in_mode_for() > self.maneuver_timeout:
            self.abort("manobra sem heading no tempo limite (RTK FIXED?)")

    def verify_and_engage(self):
        if self.pending is not None:
            return
        path = self.path_local()
        o = self.odom.msg.pose.pose
        yaw = yaw_from_quaternion(o.orientation)
        fx = o.position.x + self.wheelbase * math.cos(yaw)
        fy = o.position.y + self.wheelbase * math.sin(yaw)
        proj = path.project(fx, fy, hint=0, back_m=2.0, ahead_m=25.0)
        heading_error = float(wrap_angle(proj.heading - yaw))
        if abs(proj.cross_track) > self.max_enable_cross_track:
            self.abort(f"desalinhado: {proj.cross_track:+.2f} m do caminho")
            return
        if abs(heading_error) > self.max_heading_error:
            self.abort(f"desalinhado: rumo {math.degrees(heading_error):+.0f} graus")
            return
        self.hold_steering = 0.0
        self.auto_origin_id = self.ekf_status.msg.origin_id
        self.pending = self.stanley_enable.call_async(SetBool.Request(data=True))

        def done(future, proj=proj, heading_error=heading_error):
            self.pending = None
            result = future.result()
            if self.mode != M.MODE_MANEUVER:
                return
            if result is None or not result.success:
                self.abort(f"Stanley recusou: {result.message if result else 'sem resposta'}")
            else:
                self.set_mode(M.MODE_AUTOMATIC, f"alinhado ({proj.cross_track:+.2f} m, "
                              f"{math.degrees(heading_error):+.0f} graus): {result.message}")
        self.pending.add_done_callback(done)

    def run_automatic(self):
        status = self.stanley_status.msg
        if not self.stanley_steering.fresh(self.reference_timeout) or \
                not self.stanley_drive.fresh(self.reference_timeout) or status is None:
            if self.in_mode_for() > 0.5:
                self.abort("Stanley sem comandos (odom do EKF parou?)")
            self.publish_refs(self.hold_steering, self.maneuver_speed)
            return
        s = self.ekf_status.msg
        if s.phase != EkfStatus.PHASE_RUNNING or s.origin_id != self.auto_origin_id:
            self.abort("EKF reiniciou durante o automatico")
            return
        if status.state == StanleyStatus.STATE_NO_ODOM:
            self.abort("odom do EKF atrasada")
            return
        if abs(status.cross_track_m) > self.max_cross_track:
            self.abort(f"desvio lateral de {status.cross_track_m:+.2f} m")
            return
        self.hold_steering = self.stanley_steering.msg.angle_deg
        drive = self.stanley_drive.msg
        self.publish_refs(self.hold_steering, drive.velocity_m_s, drive.brake_command, drive.acceleration_m_s2)
        if status.state == StanleyStatus.STATE_FINISHED:
            self.set_mode(M.MODE_STOPPING, "fim do caminho")

    def run_stopping(self):
        self.publish_refs(self.hold_steering, 0.0, 1.0 if self.aborted else 0.0)
        stopped = self.ekf_status.fresh(self.status_timeout) and self.ekf_status.msg.stationary
        if (stopped and self.in_mode_for() > 1.0) or self.in_mode_for() > self.stopping_timeout:
            self.wait_neutral_gamepad = True
            self.set_mode(M.MODE_MANUAL, "parado; solte o R2 para voltar ao gamepad", aborted=self.aborted)

    def publish_mode(self):
        msg = NavigationMode()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.mode = self.mode
        msg.aborted = self.aborted
        msg.refused_count = self.refused_count
        msg.reason = self.reason
        msg.path_name = self.path_name
        status = self.stanley_status.msg
        if status is not None and self.mode in (M.MODE_AUTOMATIC, M.MODE_STOPPING):
            msg.progress_m, msg.remaining_m = status.progress_m, status.remaining_m
        self.mode_pub.publish(msg)


def main():
    rclpy.init()
    node = ModeManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
