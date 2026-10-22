"""Malha fechada sem ROS: modelo do veiculo + EKF simulado + Stanley.

Usado nos testes e para ajustar o ganho antes de ir a pista:

    python3 -m vida_navigation.closed_loop pista.csv --gain 1.0 --scale-error 0.07
"""
import argparse
import math
from dataclasses import dataclass, field

import numpy as np
from vida_interfaces.msg import GnssStatus, StanleyStatus

from vida_navigation.path import LocalPath
from vida_navigation.sim_vehicle import EkfEmulator
from vida_navigation.stanley import StanleyController, StanleyParams
from vida_navigation.vehicle_model import VehicleModel, VehicleParams, VehicleState
from vida_navigation.geo import Origin


@dataclass
class RunResult:
    finished: bool
    time_s: float
    cross_track: np.ndarray            # erro lateral real do eixo dianteiro (m)
    steering: np.ndarray               # comando (graus)
    saturated_fraction: float
    trajectory: np.ndarray = field(repr=False)   # (x, y) real do eixo traseiro

    def summary(self, settle_m=0.0):
        e = np.abs(self.cross_track)
        return (f"{'terminou' if self.finished else 'NAO terminou'} em {self.time_s:.0f} s; "
                f"lateral media {e.mean():.3f} m, p95 {np.percentile(e, 95):.3f} m, "
                f"maxima {e.max():.3f} m; saturado {100 * self.saturated_fraction:.0f}% do tempo")


def run(path, start_back_m=0.0, start_left_m=0.0, start_yaw_deg=0.0, stanley=None,
        vehicle=None, encoder_scale_error=0.0, gps_noise_m=0.0, maneuver_m=3.0,
        dt=0.01, timeout_s=None, seed=0, enable_left_m=None, enable_yaw_deg=0.0):
    """Posiciona no inicio do caminho, anda maneuver_m reto (como a manobra do
    mode_manager), habilita o Stanley e roda ate o fim.

    enable_left_m / enable_yaw_deg: em vez do erro que sobrar da manobra, coloca
    o eixo dianteiro com esse erro lateral e de rumo no momento da habilitacao
    (o que o mode_manager confere: < 1 m e < 30 graus)."""
    stanley = stanley or StanleyParams()
    yaw0 = float(path.heading[0])
    yaw = yaw0 + math.radians(start_yaw_deg)
    x0 = path.x[0] - start_back_m * math.cos(yaw0) - start_left_m * math.sin(yaw0)
    y0 = path.y[0] - start_back_m * math.sin(yaw0) + start_left_m * math.cos(yaw0)
    model = VehicleModel(vehicle or VehicleParams(wheelbase_m=1.38), VehicleState(x=x0, y=y0, yaw=yaw))
    # O frame local do caminho e o "mundo"; o EKF simulado tem a origem no 1o fix.
    ekf = EkfEmulator(Origin(-21.2293928, -44.9732956), encoder_scale_error=encoder_scale_error,
                      gps_noise_m=gps_noise_m, seed=seed)
    truth = StanleyController(path, stanley)        # so para medir o erro real

    t, c = 0.0, model.commands
    while ekf.origin is None:                       # parado: origem
        t += dt
        c.steering_time = c.drive_time = t
        model.step(t, dt)
        ekf.update(model.state, t, GnssStatus.RTK_FIXED, True, True)
    origin_shift = ekf.to_ekf_frame(0.0, 0.0)
    shifted = LocalPath(path.x + origin_shift[0], path.y + origin_shift[1], path.closed)
    controller = StanleyController(shifted, stanley)

    start_distance = model.state.distance           # manobra reta
    while model.state.distance - start_distance < maneuver_m or ekf.heading_source == 0:
        t += dt
        c.steering_deg, c.speed_m_s = 0.0, 0.5
        c.steering_time = c.drive_time = t
        model.step(t, dt)
        ekf.update(model.state, t, GnssStatus.RTK_FIXED, True, False)
        if t > 60:
            raise RuntimeError("manobra nao inicializou o heading")

    s = model.state
    if enable_left_m is not None:
        L = stanley.wheelbase_m
        ref = path.project(s.x + L * math.cos(s.yaw), s.y + L * math.sin(s.yaw))
        yaw = ref.heading + math.radians(enable_yaw_deg)
        fx = ref.x - enable_left_m * math.sin(ref.heading)
        fy = ref.y + enable_left_m * math.cos(ref.heading)
        nx, ny = fx - L * math.cos(yaw), fy - L * math.sin(yaw)
        dx, dy = nx - s.x, ny - s.y
        s.x, s.y, s.yaw = nx, ny, yaw
        ekf.fix_true = (ekf.fix_true[0] + dx, ekf.fix_true[1] + dy)
        ekf.fix_measured = (ekf.fix_measured[0] + dx, ekf.fix_measured[1] + dy)
    x, y, yaw = ekf.estimate(model.state)
    controller.start(x, y, yaw, model.state.steering_deg)
    truth.start(s.x, s.y, s.yaw)
    timeout_s = timeout_s or t + 3.0 * path.length / max(stanley.speed_m_s, 0.1) + 30.0
    errors, steering, saturated, trajectory = [], [], 0, []
    next_control = t
    while t < timeout_s:
        t += dt
        ekf.update(model.state, t, GnssStatus.RTK_FIXED, True, s.speed < 0.02)
        if t >= next_control:                       # odom a 50 Hz
            next_control += 0.02
            x, y, yaw = ekf.estimate(s)
            out = controller.update(x, y, yaw, s.speed * (1 + encoder_scale_error), 0.02)
            c.steering_deg, c.speed_m_s = out.steering_command_deg, out.speed_command_m_s
            c.steering_time = c.drive_time = t
            real = truth.update(s.x, s.y, s.yaw, s.speed, 0.02)
            errors.append(real.cross_track_m)
            steering.append(out.steering_command_deg)
            saturated += out.steering_saturated
            trajectory.append((s.x, s.y))
            if out.state == StanleyStatus.STATE_FINISHED:
                break
        model.step(t, dt)
    return RunResult(controller.finished, t, np.array(errors), np.array(steering),
                     saturated / max(len(errors), 1), np.array(trajectory))


def main(argv=None):
    from vida_navigation.path import PathFile, plot
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("path_file")
    parser.add_argument("--gain", type=float, default=2.0)
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--scale-error", type=float, default=0.0)
    parser.add_argument("--gps-noise", type=float, default=0.0)
    parser.add_argument("--left", type=float, default=0.0, help="deslocamento lateral inicial (m)")
    parser.add_argument("--yaw", type=float, default=0.0, help="desalinhamento inicial (graus)")
    parser.add_argument("--plot", default="")
    args = parser.parse_args(argv)
    path_file = PathFile.load(args.path_file)
    path = path_file.to_local(path_file.start_origin)
    result = run(path, start_left_m=args.left, start_yaw_deg=args.yaw,
                 stanley=StanleyParams(gain_k=args.gain, speed_m_s=args.speed),
                 encoder_scale_error=args.scale_error, gps_noise_m=args.gps_noise)
    print(result.summary())
    if args.plot:
        plot(path, args.plot, f"{path_file.meta.get('name', '')}: trajetoria simulada (preto)",
             raw_xy=(result.trajectory[:, 0], result.trajectory[:, 1]))
        print(f"grafico: {args.plot}")
    return 0 if result.finished else 2


if __name__ == "__main__":
    raise SystemExit(main())
