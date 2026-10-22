"""Cenarios de teste do modo automatico no simulador (faz o papel do operador).

    ros2 launch vida_navigation simulation.launch.py &
    ros2 run vida_navigation sim_scenarios volta

Publica o gamepad em /gamepad/* (parado, sem freio), pede o automatico como
o triangulo e injeta falhas no sim_vehicle. Confere a sequencia de modos e,
na volta completa, o erro lateral real (/sim/ground_truth) contra o caminho.

Cenarios: volta, float, l2, quadrado, rtk_perdido, gps_perdido, esp_bloqueio,
longe (lancar o simulador com start_back_m:=6) e desalinhado (start_yaw_deg:=40).
"""
import math
import sys

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from std_msgs.msg import Bool
from vida_interfaces.msg import DriveReference, EkfStatus, NavigationMode, SteeringReference

from vida_navigation.mode_manager import MODE_NAMES
from vida_navigation.path import PathFile
from vida_navigation.ros_common import origin_from_status

M = NavigationMode
FULL = [M.MODE_PREPARING, M.MODE_CALIBRATING, M.MODE_MANEUVER, M.MODE_AUTOMATIC, M.MODE_STOPPING, M.MODE_MANUAL]

# nome: (acao apos N s no automatico, sequencia esperada, deve abortar, motivo esperado)
SCENARIOS = {
    "volta": (None, FULL, False, "fim do caminho"),
    "float": (("gnss", "float"), FULL, False, "fim do caminho"),
    "l2": (("l2", None), FULL, True, "L2"),
    "quadrado": (("exit", None), FULL, False, "saida pedida"),
    "rtk_perdido": (("gnss", "no_rtk"), FULL, True, "correcoes RTK"),
    "gps_perdido": (("gnss", "lost"), FULL, True, "GPS sem comunicacao"),
    "esp_bloqueio": (("inhibit", 512), FULL, True, "ESP32 bloqueando"),
    "longe": (None, [], False, "longe do inicio"),
    "desalinhado": (None, [M.MODE_PREPARING, M.MODE_CALIBRATING, M.MODE_MANEUVER, M.MODE_STOPPING,
                           M.MODE_MANUAL], True, "desalinhado"),
}


class Operator(Node):
    def __init__(self, scenario, path_file, action_after_s):
        super().__init__("sim_operator")
        self.scenario = scenario
        self.action, self.expected, self.expect_abort, self.expect_reason = SCENARIOS[scenario]
        self.action_after = action_after_s
        self.path_file = PathFile.load(path_file)
        self.steering_pub = self.create_publisher(SteeringReference, "/gamepad/steering/reference", 10)
        self.drive_pub = self.create_publisher(DriveReference, "/gamepad/drive/reference", 10)
        self.request_pub = self.create_publisher(Bool, "/navigation/auto_request", 10)
        self.create_subscription(NavigationMode, "/navigation/mode", self.on_mode, 10)
        self.create_subscription(EkfStatus, "/robot_state/status", self.on_status, 10)
        self.create_subscription(Odometry, "/sim/ground_truth", self.on_truth, 10)
        self.set_params = self.create_client(SetParameters, "/sim_vehicle/set_parameters")
        self.create_timer(0.04, self.on_timer)
        self.t0 = self.now()
        self.modes, self.reasons = [], []
        self.mode_msg = None
        self.status = None
        self.truth = []
        self.brake = False
        self.requested = False
        self.auto_since = None
        self.action_done = False
        self.done = False
        self.local_path = None

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_mode(self, msg):
        first = self.mode_msg is None
        if first or msg.mode != self.mode_msg.mode:
            if not first:
                self.modes.append(msg.mode)
                self.reasons.append(msg.reason)
                print(f"  {self.now() - self.t0:6.1f} s  {MODE_NAMES[msg.mode]:10s} {msg.reason}")
            if msg.mode == M.MODE_AUTOMATIC:
                self.auto_since = self.now()
            if msg.mode == M.MODE_MANUAL and not first:
                self.done = True
        if self.mode_msg is not None and msg.refused_count != self.mode_msg.refused_count:
            self.reasons.append(msg.reason)
            print(f"  {self.now() - self.t0:6.1f} s  {msg.reason}")
            self.done = True
        self.mode_msg = msg

    def on_status(self, msg):
        self.status = msg

    def on_truth(self, msg):
        if self.mode_msg is not None and self.mode_msg.mode == M.MODE_AUTOMATIC:
            p = msg.pose.pose.position
            self.truth.append((p.x, p.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
            if self.local_path is None and self.status is not None:
                self.local_path = self.path_file.to_local(origin_from_status(self.status))

    def set_sim(self, name, value):
        if isinstance(value, str):
            pv = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=value)
        else:
            pv = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=int(value))
        self.set_params.call_async(SetParameters.Request(parameters=[Parameter(name=name, value=pv)]))

    def on_timer(self):
        stamp = self.get_clock().now().to_msg()
        steering = SteeringReference()
        steering.timestamp = stamp
        drive = DriveReference()
        drive.timestamp = stamp
        drive.brake_command = 1.0 if self.brake else 0.0
        self.steering_pub.publish(steering)
        self.drive_pub.publish(drive)

        elapsed = self.now() - self.t0
        if not self.requested and elapsed > 3.0 and self.mode_msg is not None:
            self.requested = True
            print(f"  {elapsed:6.1f} s  operador: triangulo 5 s")
            self.request_pub.publish(Bool(data=True))
        if self.action and not self.action_done and self.auto_since is not None and \
                self.now() - self.auto_since > self.action_after:
            self.action_done = True
            kind, value = self.action
            print(f"  {elapsed:6.1f} s  falha/acao: {kind} {value if value is not None else ''}")
            if kind == "gnss":
                self.set_sim("gnss_scenario", value)
            elif kind == "inhibit":
                self.set_sim("inject_inhibit", value)
            elif kind == "l2":
                self.brake = True
            elif kind == "exit":
                self.request_pub.publish(Bool(data=False))

    def lateral_errors(self):
        if not self.truth or self.local_path is None:
            return np.array([])
        errors, hint = [], None
        for x, y, qz, qw in self.truth:
            yaw = 2.0 * math.atan2(qz, qw)
            fx, fy = x + 1.125 * math.cos(yaw), y + 1.125 * math.sin(yaw)
            proj = self.local_path.project(fx, fy, hint=hint)
            hint = proj.index
            errors.append(proj.cross_track)
        return np.array(errors)

    def verdict(self):
        problems = []
        if self.modes != self.expected:
            problems.append("sequencia " + " > ".join(MODE_NAMES[m] for m in self.modes))
        aborted = self.mode_msg is not None and self.mode_msg.aborted
        if self.expected and aborted != self.expect_abort:
            problems.append(f"aborted={aborted}")
        if not any(self.expect_reason in r for r in self.reasons):
            problems.append(f"motivo '{self.expect_reason}' nao apareceu")
        e = self.lateral_errors()
        if len(e):
            print(f"  erro lateral real (eixo dianteiro) no automatico: media {np.abs(e).mean():.3f} m, "
                  f"p95 {np.percentile(np.abs(e), 95):.3f} m, maxima {np.abs(e).max():.3f} m")
        return problems


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    rclpy.init()
    scenario = argv[0] if argv else "volta"
    from ament_index_python.packages import get_package_share_directory
    path_file = argv[1] if len(argv) > 1 else \
        f"{get_package_share_directory('vida_navigation')}/paths/exemplo_estadio.csv"
    node = Operator(scenario, path_file, action_after_s=15.0)
    print(f"cenario '{scenario}':")
    deadline = node.now() + 240.0
    try:
        while rclpy.ok() and not node.done and node.now() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.05)
        problems = node.verdict()
        print(f"  resultado: {'OK' if not problems else 'FALHOU: ' + '; '.join(problems)}")
        return 0 if not problems else 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
