"""Utilitarios ROS comuns aos nos de navegacao."""
import math

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from vida_interfaces.msg import EkfStatus

from vida_navigation.geo import Origin

# Topicos publicados uma vez e guardados para quem assinar depois (ex.: caminho).
LATCHED_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def origin_from_status(status):
    """Origem do frame local do EKF, ou None se ainda nao definida."""
    if not status.origin_valid:
        return None
    return Origin(status.origin_latitude_deg, status.origin_longitude_deg)


def ekf_ready(status):
    """EKF rodando com heading vindo do GPS."""
    return (status.phase == EkfStatus.PHASE_RUNNING and
            status.heading_source == EkfStatus.HEADING_GPS)
