import math

from vida_interfaces.msg import EkfStatus, GnssStatus

from vida_navigation.geo import Origin
from vida_navigation.sim_vehicle import EkfEmulator
from vida_navigation.vehicle_model import VehicleModel, VehicleParams

DT = 0.01


def drive(model, seconds, t0=0.0, steering=0.0, speed=0.0, brake=0.0, fresh=True):
    t = t0
    for _ in range(int(round(seconds / DT))):
        t += DT
        if fresh:
            c = model.commands
            c.steering_deg, c.speed_m_s, c.brake = steering, speed, brake
            c.steering_time = c.drive_time = t
        model.step(t, DT)
    return t


def test_speed_ramp_reaches_reference_without_overshoot():
    model = VehicleModel()
    drive(model, 2.0, speed=1.0)
    assert 0.3 < model.state.speed < 0.8          # rampa de 0,4 m/s^2 + atraso
    drive(model, 8.0, t0=2.0, speed=1.0)
    assert abs(model.state.speed - 1.0) < 0.02


def test_constant_steering_gives_kinematic_radius():
    model = VehicleModel(VehicleParams(wheelbase_m=1.38))
    drive(model, 10.0, steering=20.0, speed=1.0)
    radius = model.state.speed / model.yaw_rate
    assert 19.0 < model.state.steering_deg < 21.0                    # zona morta + sobressinal
    assert abs(radius - 1.38 / math.tan(math.radians(model.state.steering_deg))) < 1e-6
    assert 3.6 < radius < 4.1                                           # ~3,8 m


def test_steering_reference_is_saturated_and_deadband_holds():
    model = VehicleModel()
    drive(model, 3.0, steering=35.0)
    assert abs(model.state.steering_deg - 20.0) < 1.0
    angle = model.state.steering_deg
    drive(model, 1.0, t0=3.0, steering=angle + 1.0)   # dentro da histerese: nao mexe
    assert abs(model.state.steering_deg - angle) < 0.05


def test_stale_references_block_and_brake():
    model = VehicleModel()
    t = drive(model, 6.0, speed=1.0)
    assert model.inhibit_flags(t) == 0
    t = drive(model, 2.0, t0=t, fresh=False)
    assert model.inhibit_flags(t) & VehicleModel.INHIBIT_DRIVE_REFERENCE_NOT_FRESH
    assert model.state.speed == 0.0


def test_ekf_emulator_phases_and_heading_requirement():
    model = VehicleModel()
    ekf = EkfEmulator(Origin(-21.2293928, -44.9732956))
    t = 0.0
    for _ in range(400):                              # parado, 4 s, FIXED: origem
        t += DT
        ekf.update(model.state, t, GnssStatus.RTK_FIXED, True, True)
    assert ekf.phase == EkfStatus.PHASE_WAITING_HEADING and ekf.origin_id == 1
    for _ in range(600):                              # anda com FLOAT: sem heading
        t = drive(model, DT, t0=t, speed=0.5)
        ekf.update(model.state, t, GnssStatus.RTK_FLOAT, True, False)
    assert ekf.phase == EkfStatus.PHASE_WAITING_HEADING
    for _ in range(800):                              # FIXED de novo: 2 m e pronto
        t = drive(model, DT, t0=t, speed=0.5)
        ekf.update(model.state, t, GnssStatus.RTK_FIXED, True, False)
    assert ekf.phase == EkfStatus.PHASE_RUNNING and ekf.heading_source == EkfStatus.HEADING_GPS
    x, y, yaw = ekf.estimate(model.state)
    ex, ey = ekf.to_ekf_frame(model.state.x, model.state.y)
    assert math.hypot(x - ex, y - ey) < 1e-6 and abs(yaw) < 1e-9


def test_ekf_emulator_sawtooth_from_encoder_scale():
    model = VehicleModel()
    ekf = EkfEmulator(Origin(-21.2293928, -44.9732956), encoder_scale_error=0.07, gps_rate_hz=1.0)
    t, worst = 0.0, 0.0
    for _ in range(400):
        t += DT
        ekf.update(model.state, t, GnssStatus.RTK_FIXED, True, True)
    for _ in range(3000):
        t = drive(model, DT, t0=t, speed=1.0)
        ekf.update(model.state, t, GnssStatus.RTK_FIXED, True, False)
        if ekf.origin is not None:
            x, y, _ = ekf.estimate(model.state)
            ex, ey = ekf.to_ekf_frame(model.state.x, model.state.y)
            worst = max(worst, math.hypot(x - ex, y - ey))
    assert 0.05 < worst < 0.08                         # 7% de 1 m entre fixes a 1 Hz
