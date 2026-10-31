"""Modelo do VIDA para o simulador: bicicleta no eixo traseiro + malhas do ESP32.

Reproduz o que o firmware faz com as referencias, nao o motor em si:
- esterçamento: referencia saturada em +-20 graus e limitada a 40 graus/s,
  motor com taxa limitada e zona morta com histerese (0,4 / 1,2 grau), como o
  SteeringController;
- velocidade: rampa da referencia (acceleration_m_s2, padrao 0,4 m/s^2, maximo
  0,8; parte da velocidade atual se o veiculo ja anda acima dela) seguida de
  uma resposta de 1a ordem (PI do DriveController fechado); so para frente
  (a re nao existe); rampa em zero = freio de parada total;
- numeros do chao (30/10/2026): entre-eixos efetivo 1,42 m, freio total
  ~1,7 m/s^2 com a roda livre, roda livre ~0,5 m/s^2. Abaixo de ~2 m/s a
  controladora real pulsa (zona morta); o modelo nao reproduz isso, entao a
  navegacao nao deve pedir menos que isso andando;
- seguranca: sem referencias novas ha 200 ms, sem enable logico ou com uma
  inibicao injetada, o ESP32 bloqueia (tracao zero, freio total).
"""
import math
from dataclasses import dataclass, field

import numpy as np

from vida_navigation.geo import wrap_angle


@dataclass
class VehicleParams:
    wheelbase_m: float = 1.42                 # efetivo (geometrico 1,125)
    steering_reference_max_deg: float = 20.0
    steering_reference_rate_deg_s: float = 40.0
    steering_max_deg: float = 25.0
    steering_rate_max_deg_s: float = 30.0
    steering_gain_1_s: float = 6.0            # taxa = ganho * erro, ate a taxa maxima
    steering_deadband_enter_deg: float = 0.4
    steering_deadband_exit_deg: float = 1.2
    drive_default_accel_m_s2: float = 0.4
    drive_max_accel_m_s2: float = 0.8
    drive_time_constant_s: float = 1.2        # malha de velocidade fechada
    drive_traction_accel_max_m_s2: float = 1.0
    brake_decel_max_m_s2: float = 1.2         # freio total (+ roda livre = ~1,7)
    rolling_decel_m_s2: float = 0.5           # sem tracao nem freio
    reference_timeout_s: float = 0.2


@dataclass
class VehicleState:
    x: float = 0.0          # eixo traseiro, m (frame do mundo, ENU)
    y: float = 0.0
    yaw: float = 0.0        # rad
    steering_deg: float = 0.0
    speed: float = 0.0      # m/s, >= 0
    distance: float = 0.0   # m percorridos


@dataclass
class Commands:
    steering_deg: float = 0.0
    steering_time: float = -1e9
    speed_m_s: float = 0.0
    accel_m_s2: float = 0.0
    brake: float = 0.0
    drive_time: float = -1e9


@dataclass
class VehicleModel:
    params: VehicleParams = field(default_factory=VehicleParams)
    state: VehicleState = field(default_factory=VehicleState)
    commands: Commands = field(default_factory=Commands)
    logical_enable: bool = True
    injected_inhibit: int = 0
    _speed_reference: float = 0.0     # apos a rampa
    _steering_reference: float = 0.0  # apos o limite de taxa
    _in_deadband: bool = True

    INHIBIT_LOGICAL_ENABLE_FALSE = 4
    INHIBIT_DRIVE_REFERENCE_NOT_FRESH = 16
    INHIBIT_STEERING_REFERENCE_NOT_FRESH = 32

    def inhibit_flags(self, now):
        flags = self.injected_inhibit
        if not self.logical_enable:
            flags |= self.INHIBIT_LOGICAL_ENABLE_FALSE
        if now - self.commands.drive_time > self.params.reference_timeout_s:
            flags |= self.INHIBIT_DRIVE_REFERENCE_NOT_FRESH
        if now - self.commands.steering_time > self.params.reference_timeout_s:
            flags |= self.INHIBIT_STEERING_REFERENCE_NOT_FRESH
        return flags

    def step(self, now, dt):
        p, s, c = self.params, self.state, self.commands
        permitted = self.inhibit_flags(now) == 0

        # Esterçamento: bloqueado = motor parado (a direcao fica onde esta).
        if permitted:
            requested = float(np.clip(c.steering_deg, -p.steering_reference_max_deg,
                                      p.steering_reference_max_deg))
            max_step = p.steering_reference_rate_deg_s * dt
            self._steering_reference += float(np.clip(requested - self._steering_reference,
                                                      -max_step, max_step))
            reference = self._steering_reference
            error = reference - s.steering_deg
            if self._in_deadband and abs(error) > p.steering_deadband_exit_deg:
                self._in_deadband = False
            elif not self._in_deadband and abs(error) < p.steering_deadband_enter_deg:
                self._in_deadband = True
            if not self._in_deadband:
                rate = float(np.clip(p.steering_gain_1_s * error, -p.steering_rate_max_deg_s,
                                     p.steering_rate_max_deg_s))
                s.steering_deg = float(np.clip(s.steering_deg + rate * dt, -p.steering_max_deg,
                                               p.steering_max_deg))

        # Velocidade: rampa da referencia, malha fechada de 1a ordem, freio.
        if permitted:
            accel = c.accel_m_s2 if c.accel_m_s2 > 0 else p.drive_default_accel_m_s2
            accel = min(accel, p.drive_max_accel_m_s2)
            target = max(c.speed_m_s, 0.0)
            if target > self._speed_reference:
                self._speed_reference = max(self._speed_reference, min(target, s.speed))
            step = accel * dt
            self._speed_reference += float(np.clip(target - self._speed_reference, -step, step))
            brake = float(np.clip(c.brake, 0.0, 1.0))
        else:
            self._speed_reference = 0.0
            self._steering_reference = s.steering_deg
            brake = 1.0
        if brake > 0.0:
            self._speed_reference = min(self._speed_reference, s.speed)
            dv = -(p.rolling_decel_m_s2 + brake * p.brake_decel_max_m_s2) * dt
        elif self._speed_reference <= 0.05:
            dv = -(p.rolling_decel_m_s2 + p.brake_decel_max_m_s2) * dt   # freio de parada (total)
        else:
            a = (self._speed_reference - s.speed) / p.drive_time_constant_s
            dv = float(np.clip(a, -p.brake_decel_max_m_s2, p.drive_traction_accel_max_m_s2)) * dt
        s.speed = max(0.0, s.speed + dv)

        # Cinematica (bicicleta, ponto no eixo traseiro).
        ds = s.speed * dt
        s.x += ds * math.cos(s.yaw)
        s.y += ds * math.sin(s.yaw)
        s.yaw = float(wrap_angle(s.yaw + ds * math.tan(math.radians(s.steering_deg)) / p.wheelbase_m))
        s.distance += ds
        return permitted

    @property
    def yaw_rate(self):
        s = self.state
        return s.speed * math.tan(math.radians(s.steering_deg)) / self.params.wheelbase_m
