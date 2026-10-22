import math

import numpy as np

from vida_navigation.geo import Origin, to_geodetic, to_local, wrap_angle
from vida_navigation.path import LocalPath, PathFile, build_path, validate

UFLA = Origin(-21.2293928, -44.9732956)


def circle(radius, closed=True, n=400, start_angle=-math.pi / 2):
    angles = start_angle + np.linspace(0, 2 * math.pi, n, endpoint=not closed)
    return radius * np.cos(angles), radius * np.sin(angles)


def test_geo_roundtrip_is_exact_to_millimeters():
    x = np.array([0.0, 10.0, -35.5, 120.25])
    y = np.array([0.0, -5.0, 42.0, 300.0])
    lat, lon = to_geodetic(UFLA, x, y)
    bx, by = to_local(UFLA, lat, lon)
    assert np.allclose(bx, x, atol=1e-6) and np.allclose(by, y, atol=1e-6)


def test_geo_matches_ekf_scale():
    # 1e-5 graus de latitude ~ 1,1072 m em -21,2 graus (raio meridiano WGS84)
    _, y = to_local(UFLA, UFLA.latitude_deg + 1e-5, UFLA.longitude_deg)
    assert abs(float(y) - 1.1072) < 1e-3


def test_wrap_angle():
    assert abs(abs(float(wrap_angle(3 * math.pi))) - math.pi) < 1e-9
    assert abs(float(wrap_angle(2 * math.pi + 0.5)) - 0.5) < 1e-9
    assert abs(float(wrap_angle(-math.pi / 2)) + math.pi / 2) < 1e-9


def test_build_path_closed_circle_has_uniform_spacing_and_radius():
    x, y = circle(10.0, n=24, closed=True)
    lat, lon = to_geodetic(UFLA, x, y)
    out_lat, out_lon = build_path(lat, lon, closed=True, spacing_m=0.25)
    px, py = to_local(UFLA, out_lat, out_lon)
    path = LocalPath(px, py, closed=True)
    assert abs(path.length - 2 * math.pi * 10.0) < 0.3
    assert np.allclose(path.segment_length, 0.25, atol=0.02)
    assert np.allclose(1 / np.abs(path.curvature), 10.0, rtol=0.03)
    assert abs(px[0] - x[0]) < 1e-6 and abs(py[0] - y[0]) < 1e-6  # comeca no 1o ponto


def test_csv_roundtrip(tmp_path):
    lat, lon = to_geodetic(UFLA, *circle(8.0, n=50))
    original = PathFile(np.asarray(lat), np.asarray(lon), True, {"name": "teste"})
    filename = tmp_path / "c.csv"
    original.save(filename)
    loaded = PathFile.load(filename)
    assert loaded.closed and loaded.meta["name"] == "teste"
    assert np.allclose(loaded.latitude_deg, lat, atol=1e-9)


def test_cross_track_sign_left_positive():
    # Caminho reto para leste: veiculo ao norte esta a esquerda.
    path = LocalPath(np.linspace(0, 20, 81), np.zeros(81), closed=False)
    left = path.project(5.0, 1.5)
    right = path.project(5.0, -0.7)
    assert abs(left.cross_track - 1.5) < 1e-9 and abs(right.cross_track + 0.7) < 1e-9
    assert abs(left.heading) < 1e-9 and abs(left.s - 5.0) < 1e-9


def test_project_with_hint_does_not_jump_to_nearby_branch():
    # Grampo: ida em y=0 e volta em y=2. Perto do fim da ida, com hint na ida,
    # a projecao deve ficar na ida mesmo que a volta esteja proxima.
    x = np.r_[np.linspace(0, 10, 41), np.linspace(10, 0, 41)[1:]]
    y = np.r_[np.zeros(41), np.full(40, 2.0)]
    path = LocalPath(x, y, closed=False)
    proj = path.project(5.0, 1.2, hint=20, back_m=2.0, ahead_m=3.0)
    assert proj.y == 0.0
    assert abs(proj.cross_track - 1.2) < 1e-9


def test_validate_flags_tight_curve_and_short_start():
    x, y = circle(3.0, n=200)
    path = LocalPath(x, y, closed=True)
    report = validate(path, min_turn_radius_m=3.8, start_straight_required_m=5.0)
    assert not report.ok
    assert report.infeasible_m > 15.0 and report.start_straight_m < 1.0


def test_validate_accepts_stadium_with_straight_start():
    # Estadio: retas de 20 m e semicirculos de raio 8 m, comecando no meio da reta.
    pts = []
    for t in np.linspace(0, 20, 81)[:-1]:
        pts.append((t, -8.0))
    for a in np.linspace(-math.pi / 2, math.pi / 2, 101)[:-1]:
        pts.append((20 + 8 * math.cos(a), 8 * math.sin(a)))
    for t in np.linspace(20, 0, 81)[:-1]:
        pts.append((t, 8.0))
    for a in np.linspace(math.pi / 2, 3 * math.pi / 2, 101)[:-1]:
        pts.append((8 * math.cos(a), 8 * math.sin(a)))
    pts = np.array(pts)
    pts = np.roll(pts, -40, axis=0)
    path = LocalPath(pts[:, 0], pts[:, 1], closed=True)
    report = validate(path)
    assert report.ok, report.problems
    assert report.direction == "anti-horario"


def test_from_recording_refuses_gap(tmp_path):
    from vida_navigation.path_tools import main
    # reta de 20 m gravada a cada 0,25 m com um buraco de 4 m no meio
    x = np.r_[np.arange(0, 8, 0.25), np.arange(12, 20, 0.25)]
    lat, lon = to_geodetic(UFLA, x, np.zeros_like(x))
    raw = tmp_path / "bruto.csv"
    PathFile(np.asarray(lat), np.asarray(lon), False).save(raw)
    assert main(["from-recording", str(raw), str(tmp_path / "a.csv"), "--start-straight", "1"]) == 2
    assert main(["from-recording", str(raw), str(tmp_path / "b.csv"), "--start-straight", "1",
                 "--max-gap", "5"]) == 0
