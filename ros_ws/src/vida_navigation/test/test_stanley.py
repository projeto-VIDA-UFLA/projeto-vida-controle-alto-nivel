import math

import numpy as np

from vida_navigation.closed_loop import run
from vida_navigation.path import LocalPath
from vida_navigation.stanley import StanleyController, StanleyParams


def stadium(straight=20.0, radius=5.0, start_m=8.0):
    pts = []
    for t in np.arange(0, straight, 0.25):
        pts.append((t, -radius))
    for a in np.arange(-math.pi / 2, math.pi / 2, 0.25 / radius):
        pts.append((straight + radius * math.cos(a), radius * math.sin(a)))
    for t in np.arange(straight, 0, -0.25):
        pts.append((t, radius))
    for a in np.arange(math.pi / 2, 3 * math.pi / 2, 0.25 / radius):
        pts.append((radius * math.cos(a), radius * math.sin(a)))
    pts = np.roll(np.array(pts), -int(start_m / 0.25), axis=0)
    return LocalPath(pts[:, 0], pts[:, 1], True)


def test_steers_right_when_left_of_path_and_aligned():
    path = LocalPath(np.linspace(0, 30, 121), np.zeros(121), closed=False)
    controller = StanleyController(path, StanleyParams(steering_rate_deg_s=1e6))
    controller.start(2.0, 0.2, 0.0)
    out = controller.update(2.0, 0.2, 0.0, 1.0, 0.02)
    assert out.cross_track_m > 0 and out.steering_command_deg < 0
    assert abs(out.steering_command_deg - math.degrees(-math.atan(2.0 * 0.2 / 2.0))) < 1e-6


def test_output_is_saturated_and_rate_limited():
    path = LocalPath(np.linspace(0, 30, 121), np.zeros(121), closed=False)
    controller = StanleyController(path)
    controller.start(2.0, -3.0, 0.0)
    out = controller.update(2.0, -3.0, 0.0, 1.0, 0.02)
    assert out.steering_saturated and abs(out.steering_command_deg - 40.0 * 0.02) < 1e-9
    for _ in range(100):
        out = controller.update(2.0, -3.0, 0.0, 1.0, 0.02)
    assert out.steering_command_deg == 20.0


def test_one_lap_tracks_and_stops_at_start_point():
    path = stadium()
    result = run(path, encoder_scale_error=0.07, gps_noise_m=0.02)
    assert result.finished
    distance = np.r_[0, np.cumsum(np.hypot(*np.diff(result.trajectory, axis=0).T))]
    e = np.abs(result.cross_track[distance > 10.0])
    assert e.mean() < 0.15 and e.max() < 0.35
    # para no ponto de partida do caminho (eixo dianteiro), com a tolerancia de parada
    x, y = result.trajectory[-1]
    yaw = path.project(x, y).heading
    fx, fy = x + 1.125 * math.cos(yaw), y + 1.125 * math.sin(yaw)
    assert math.hypot(fx - path.x[0], fy - path.y[0]) < 0.6


def test_worst_enabled_error_stays_below_abort_threshold():
    # O mode_manager so habilita com < 1 m e < 30 graus; o aborto lateral e 1,5 m.
    path = stadium()
    for left, yaw in ((1.0, 30.0), (-1.0, -30.0)):
        result = run(path, enable_left_m=left, enable_yaw_deg=yaw, encoder_scale_error=0.07)
        distance = np.r_[0, np.cumsum(np.hypot(*np.diff(result.trajectory, axis=0).T))]
        assert np.abs(result.cross_track).max() < 1.4
        assert np.abs(result.cross_track[distance > 5.0]).max() < 0.35
