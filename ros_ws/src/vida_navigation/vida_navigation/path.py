"""Caminhos do VIDA: arquivo, tratamento, geometria e validacao.

Formato do arquivo (CSV em lat/lon, independente da origem do EKF, que muda
a cada partida):

    # vida_path v1
    # name: pista_ufla
    # source: google_earth | ekf_recording
    # closed: true
    # spacing_m: 0.25
    # created: 2026-10-21T10:00:00
    lat,lon
    -21.22939281,-44.97329562
    ...

Os pontos ja estao tratados (suavizados e reamostrados a cada spacing_m); o
primeiro ponto e o inicio do caminho. Em caminho fechado o ultimo ponto nao
repete o primeiro.
"""
import datetime
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

import numpy as np
from scipy.interpolate import splev, splprep

from vida_navigation.geo import Origin, to_geodetic, to_local, wrap_angle

FORMAT_TAG = "vida_path v1"


# ---------------------------------------------------------------- arquivo

@dataclass
class PathFile:
    latitude_deg: np.ndarray
    longitude_deg: np.ndarray
    closed: bool
    meta: dict = field(default_factory=dict)

    @classmethod
    def load(cls, filename):
        meta = {}
        rows = []
        with open(filename, encoding="utf-8") as stream:
            for line in stream:
                line = line.strip()
                if not line:
                    continue
                if line.startswith("#"):
                    key, sep, value = line[1:].partition(":")
                    if sep:
                        meta[key.strip()] = value.strip()
                    continue
                if line.lower().startswith("lat"):
                    continue
                lat, lon = line.split(",")[:2]
                rows.append((float(lat), float(lon)))
        if len(rows) < 2:
            raise ValueError(f"{filename}: caminho com menos de 2 pontos")
        data = np.array(rows)
        closed = meta.get("closed", "false").lower() in ("true", "1", "yes", "sim")
        return cls(data[:, 0], data[:, 1], closed, meta)

    def save(self, filename):
        meta = dict(self.meta)
        meta["closed"] = "true" if self.closed else "false"
        meta.setdefault("created", datetime.datetime.now().isoformat(timespec="seconds"))
        with open(filename, "w", encoding="utf-8") as stream:
            stream.write(f"# {FORMAT_TAG}\n")
            for key, value in meta.items():
                stream.write(f"# {key}: {value}\n")
            stream.write("lat,lon\n")
            for lat, lon in zip(self.latitude_deg, self.longitude_deg):
                stream.write(f"{lat:.9f},{lon:.9f}\n")

    @property
    def start_origin(self):
        return Origin(float(self.latitude_deg[0]), float(self.longitude_deg[0]))

    def to_local(self, origin):
        x, y = to_local(origin, self.latitude_deg, self.longitude_deg)
        return LocalPath(x, y, self.closed)


def read_kml(filename):
    """Le a primeira linha (LineString) ou poligono (LinearRing) de um KML.

    Retorna (lat, lon, fechado). Coordenadas KML sao lon,lat[,alt].
    """
    root = ET.parse(filename).getroot()
    for element in root.iter():
        tag = element.tag.split("}")[-1]
        if tag not in ("LineString", "LinearRing"):
            continue
        coordinates = next((child for child in element.iter()
                            if child.tag.split("}")[-1] == "coordinates"), None)
        if coordinates is None or not coordinates.text:
            continue
        points = [token.split(",") for token in coordinates.text.split()]
        lon = np.array([float(p[0]) for p in points])
        lat = np.array([float(p[1]) for p in points])
        return lat, lon, tag == "LinearRing"
    raise ValueError(f"{filename}: nenhuma LineString/LinearRing encontrada")


# ---------------------------------------------------------------- tratamento

def build_path(latitude_deg, longitude_deg, closed, spacing_m=0.25, smoothing_m=0.0):
    """Trata pontos brutos (cliques ou gravacao) e devolve (lat, lon) reamostrados.

    - remove pontos repetidos (< 1 cm) e, se fechado, o ultimo igual ao primeiro;
    - ajusta uma spline cubica (periodica se fechado). smoothing_m = 0 passa
      exatamente pelos pontos (cliques esparsos); > 0 suaviza ruido de
      gravacao (desvio-padrao esperado dos pontos, em metros);
    - reamostra a cada spacing_m ao longo do comprimento, comecando no 1o ponto
      (fechado: espacamento ajustado para dividir o laco em partes iguais).
    """
    origin = Origin(float(latitude_deg[0]), float(longitude_deg[0]))
    x, y = to_local(origin, latitude_deg, longitude_deg)

    keep = np.r_[True, np.hypot(np.diff(x), np.diff(y)) > 0.01]
    x, y = x[keep], y[keep]
    if closed and len(x) > 2 and math.hypot(x[-1] - x[0], y[-1] - y[0]) < 0.01:
        x, y = x[:-1], y[:-1]
    if len(x) < 4:
        raise ValueError("sao necessarios pelo menos 4 pontos distintos")

    if closed:
        x, y = np.r_[x, x[0]], np.r_[y, y[0]]
    tck, _ = splprep([x, y], s=len(x) * smoothing_m ** 2, per=closed, k=3)

    dense_u = np.linspace(0.0, 1.0, max(20000, 40 * len(x)))
    dense_x, dense_y = splev(dense_u, tck)
    arc = np.r_[0.0, np.cumsum(np.hypot(np.diff(dense_x), np.diff(dense_y)))]
    length = arc[-1]
    if closed:
        count = max(4, int(round(length / spacing_m)))
        targets = np.arange(count) * (length / count)
    else:
        count = int(length // spacing_m)
        targets = np.r_[np.arange(count) * spacing_m, length]
    u = np.interp(targets, arc, dense_u)
    px, py = splev(u, tck)
    return to_geodetic(origin, px, py)


# ---------------------------------------------------------------- geometria

@dataclass
class Projection:
    index: int               # inicio do segmento mais proximo
    fraction: float          # posicao no segmento [0, 1]
    x: float                 # ponto projetado
    y: float
    s: float                 # distancia ao longo do caminho (m)
    heading: float           # direcao do caminho no ponto (rad), interpolada entre os vertices
    curvature: float         # 1/m, positiva para a esquerda
    cross_track: float       # distancia lateral, positiva com o veiculo a ESQUERDA do caminho
    distance_to_end: float   # metros ate o fim (caminho aberto) ou ate o ponto de partida (fechado)


class LocalPath:
    """Caminho em metros (frame local do EKF) com tangente, curvatura e busca."""

    def __init__(self, x, y, closed):
        self.x = np.asarray(x, dtype=float)
        self.y = np.asarray(y, dtype=float)
        self.closed = closed
        n = len(self.x)
        if closed:
            nx, ny = np.roll(self.x, -1), np.roll(self.y, -1)
        else:
            nx, ny = np.r_[self.x[1:], self.x[-1]], np.r_[self.y[1:], self.y[-1]]
        self.segment_length = np.hypot(nx - self.x, ny - self.y)
        if not closed:
            self.segment_length[-1] = 0.0
        self.s = np.r_[0.0, np.cumsum(self.segment_length)[:-1]]
        self.length = float(np.sum(self.segment_length))

        # Tangente e curvatura por diferencas centrais.
        if closed:
            dx = np.roll(self.x, -1) - np.roll(self.x, 1)
            dy = np.roll(self.y, -1) - np.roll(self.y, 1)
        else:
            dx, dy = np.gradient(self.x), np.gradient(self.y)
        self.heading = np.arctan2(dy, dx)
        if closed:
            dpsi = wrap_angle(np.roll(self.heading, -1) - np.roll(self.heading, 1))
            ds = np.roll(self.s, -1) - np.roll(self.s, 1)
            ds[0] += self.length
            ds[-1] += self.length
        else:
            dpsi = np.gradient(np.unwrap(self.heading))
            ds = np.gradient(self.s)
        self.curvature = dpsi / np.maximum(ds, 1e-6)
        self._n = n

    def __len__(self):
        return self._n

    def _segments(self, indices):
        nxt = (indices + 1) % self._n if self.closed else np.minimum(indices + 1, self._n - 1)
        return self.x[indices], self.y[indices], self.x[nxt], self.y[nxt]

    def project(self, px, py, hint=None, back_m=2.0, ahead_m=10.0):
        """Projeta (px, py) no segmento mais proximo.

        Sem hint busca no caminho todo. Com hint (indice anterior) busca so
        numa janela [hint - back_m, hint + ahead_m]: evita saltar para outro
        trecho do caminho que passe perto (cruzamentos, grampos).
        """
        if hint is None:
            indices = np.arange(self._n if self.closed else self._n - 1)
        else:
            spacing = max(self.length / max(self._n, 1), 1e-3)
            back = int(math.ceil(back_m / spacing))
            ahead = int(math.ceil(ahead_m / spacing))
            indices = np.arange(hint - back, hint + ahead + 1)
            indices = indices % self._n if self.closed else indices[(indices >= 0) & (indices < self._n - 1)]
            if len(indices) == 0:
                indices = np.array([max(0, min(hint, self._n - 2))])

        ax, ay, bx, by = self._segments(indices)
        vx, vy = bx - ax, by - ay
        seg2 = np.maximum(vx * vx + vy * vy, 1e-12)
        t = np.clip(((px - ax) * vx + (py - ay) * vy) / seg2, 0.0, 1.0)
        qx, qy = ax + t * vx, ay + t * vy
        best = int(np.argmin((px - qx) ** 2 + (py - qy) ** 2))
        i = int(indices[best])
        frac = float(t[best])

        seg_heading = math.atan2(vy[best], vx[best])
        cross = math.cos(seg_heading) * (py - qy[best]) - math.sin(seg_heading) * (px - qx[best])
        s = float(self.s[i] + frac * self.segment_length[i])
        j = (i + 1) % self._n if self.closed else min(i + 1, self._n - 1)
        curvature = float((1 - frac) * self.curvature[i] + frac * self.curvature[j])
        # Rumo continuo: o do segmento muda em degrau a cada vertice (3 graus a cada 0,25 m
        # numa curva de 5 m) e faria o Stanley trepidar.
        heading = float(wrap_angle(self.heading[i] + frac * wrap_angle(self.heading[j] - self.heading[i])))
        return Projection(i, frac, float(qx[best]), float(qy[best]), s, heading, curvature,
                          float(cross), float(self.length - s))

    def point_ahead(self, index, distance_m):
        """Indice distance_m a frente de index (satura no fim se aberto)."""
        spacing = max(self.length / max(self._n, 1), 1e-3)
        target = index + int(round(distance_m / spacing))
        return target % self._n if self.closed else min(target, self._n - 1)


# ---------------------------------------------------------------- validacao

@dataclass
class ValidationReport:
    length_m: float
    min_radius_m: float
    infeasible_m: float          # comprimento com raio < min_turn_radius_m
    tight_m: float               # comprimento com raio < recommended_radius_m
    start_straight_m: float      # reta inicial (|curvatura| < straight_curvature)
    closure_gap_m: float
    direction: str
    self_intersections: int
    problems: list
    warnings: list

    @property
    def ok(self):
        return not self.problems


def validate(path, min_turn_radius_m=4.0, recommended_radius_m=6.0,
             start_straight_required_m=10.0, straight_curvature=1.0 / 30.0):
    """Confere se o VIDA consegue seguir o caminho.

    min_turn_radius_m: raio minimo real do veiculo (+-20 graus; teste de 12/10/2026).
    recommended_radius_m: raio com folga para o Stanley corrigir erros.
    A reta inicial e exigida pela manobra de inicializacao (heading pelo GPS).
    """
    radius = 1.0 / np.maximum(np.abs(path.curvature), 1e-9)
    infeasible = float(np.sum(path.segment_length[radius < min_turn_radius_m]))
    tight = float(np.sum(path.segment_length[radius < recommended_radius_m]))

    straight = np.abs(path.curvature) < straight_curvature
    first_curve = np.argmin(straight) if not straight.all() else len(straight)
    start_straight = float(path.s[first_curve]) if first_curve < len(path.s) else path.length

    gap = math.hypot(path.x[-1] - path.x[0], path.y[-1] - path.y[0])
    area = 0.5 * np.sum(path.x * np.roll(path.y, -1) - np.roll(path.x, -1) * path.y)
    direction = "anti-horario" if area > 0 else "horario"

    intersections = _count_self_intersections(path)

    problems, warnings = [], []
    if infeasible > 0:
        problems.append(f"{infeasible:.1f} m com raio < {min_turn_radius_m} m (impossivel com +-20 graus)")
    if tight > infeasible:
        warnings.append(f"{tight - infeasible:.1f} m com raio entre {min_turn_radius_m} e "
                        f"{recommended_radius_m} m (pouca folga para o Stanley)")
    if start_straight < start_straight_required_m:
        problems.append(f"reta inicial de {start_straight:.1f} m < {start_straight_required_m} m "
                        "(manobra de inicializacao)")
    if intersections:
        warnings.append(f"{intersections} cruzamento(s) do caminho consigo mesmo")
    if path.closed and gap > 3 * np.median(path.segment_length):
        problems.append(f"caminho fechado com {gap:.2f} m entre o fim e o inicio")

    return ValidationReport(path.length, float(radius.min()), infeasible, tight, start_straight,
                            gap, direction, intersections, problems, warnings)


def _count_self_intersections(path, step=4):
    """Cruzamentos entre segmentos nao vizinhos (amostrado a cada step pontos)."""
    x, y = path.x[::step], path.y[::step]
    if path.closed:
        x, y = np.r_[x, x[0]], np.r_[y, y[0]]
    ax, ay, bx, by = x[:-1], y[:-1], x[1:], y[1:]
    count = 0
    n = len(ax)
    for i in range(n):
        j = np.arange(i + 2, n)
        if path.closed and i == 0:
            j = j[j != n - 1]
        if len(j) == 0:
            continue
        d1 = _orient(ax[i], ay[i], bx[i], by[i], ax[j], ay[j])
        d2 = _orient(ax[i], ay[i], bx[i], by[i], bx[j], by[j])
        d3 = _orient(ax[j], ay[j], bx[j], by[j], ax[i], ay[i])
        d4 = _orient(ax[j], ay[j], bx[j], by[j], bx[i], by[i])
        count += int(np.sum((d1 * d2 < 0) & (d3 * d4 < 0)))
    return count


def _orient(ax, ay, bx, by, cx, cy):
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)


# ---------------------------------------------------------------- grafico

def plot(path, filename, title="", raw_xy=None, min_turn_radius_m=4.0, recommended_radius_m=6.0,
         other=None, other_label=""):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection

    fig, ax = plt.subplots(figsize=(10, 9))
    pts = np.c_[path.x, path.y]
    ends = np.roll(pts, -1, axis=0) if path.closed else np.r_[pts[1:], pts[-1:]]
    radius = np.clip(1.0 / np.maximum(np.abs(path.curvature), 1e-9), 0, 2 * recommended_radius_m)
    lc = LineCollection(np.stack([pts, ends], axis=1), cmap="RdYlGn",
                        norm=plt.Normalize(min_turn_radius_m * 0.6, 2 * recommended_radius_m), lw=4)
    lc.set_array(radius)
    ax.add_collection(lc)
    if raw_xy is not None:
        ax.plot(raw_xy[0], raw_xy[1], "o", color="#888888", ms=3, label="pontos de entrada")
    if other is not None:
        ax.plot(np.r_[other.x, other.x[:1]] if other.closed else other.x,
                np.r_[other.y, other.y[:1]] if other.closed else other.y,
                "-", color="#2a6fb0", lw=1.5, label=other_label)
    ax.plot(path.x[0], path.y[0], "k^", ms=13, label="inicio")
    step = max(1, len(path) // 25)
    ax.quiver(path.x[::step], path.y[::step], np.cos(path.heading[::step]), np.sin(path.heading[::step]),
              angles="xy", scale=40, width=0.003, color="#333333")
    cb = fig.colorbar(lc, ax=ax, shrink=0.7)
    cb.set_label(f"raio de curva (m) · < {min_turn_radius_m} m impossível · < {recommended_radius_m} m sem folga")
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.legend(loc="best")
    ax.set_xlabel("leste (m)")
    ax.set_ylabel("norte (m)")
    ax.set_title(title)
    fig.savefig(filename, dpi=100, bbox_inches="tight")
    plt.close(fig)
