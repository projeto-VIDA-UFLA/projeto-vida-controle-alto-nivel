"""Conversao lat/lon <-> plano local (leste, norte) em metros.

Mesma aproximacao equiretangular do robot_state_ekf (raios de curvatura WGS84
na latitude da origem), para que caminho e estimativa usem o mesmo frame.
"""
import math
from dataclasses import dataclass

import numpy as np

_SEMI_MAJOR_AXIS = 6378137.0       # WGS84 a
_ECCENTRICITY_SQ = 6.69437999014e-3  # WGS84 e^2


@dataclass(frozen=True)
class Origin:
    latitude_deg: float
    longitude_deg: float

    @property
    def meters_per_rad(self):
        """(metros por radiano de latitude, metros por radiano de longitude)."""
        lat = math.radians(self.latitude_deg)
        denom = 1.0 - _ECCENTRICITY_SQ * math.sin(lat) ** 2
        n = _SEMI_MAJOR_AXIS / math.sqrt(denom)
        m = _SEMI_MAJOR_AXIS * (1.0 - _ECCENTRICITY_SQ) / denom ** 1.5
        return m, n * math.cos(lat)


def to_local(origin, latitude_deg, longitude_deg):
    """lat/lon (graus) -> (x leste, y norte) em metros relativos a origem."""
    m_lat, m_lon = origin.meters_per_rad
    x = np.radians(np.asarray(longitude_deg, dtype=float) - origin.longitude_deg) * m_lon
    y = np.radians(np.asarray(latitude_deg, dtype=float) - origin.latitude_deg) * m_lat
    return x, y


def to_geodetic(origin, x, y):
    """(x leste, y norte) em metros -> lat/lon (graus)."""
    m_lat, m_lon = origin.meters_per_rad
    lat = origin.latitude_deg + np.degrees(np.asarray(y, dtype=float) / m_lat)
    lon = origin.longitude_deg + np.degrees(np.asarray(x, dtype=float) / m_lon)
    return lat, lon


def wrap_angle(angle):
    """Angulo em [-pi, pi)."""
    return (np.asarray(angle) + math.pi) % (2.0 * math.pi) - math.pi
