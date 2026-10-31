"""Ferramentas de caminho (linha de comando).

    ros2 run vida_navigation path_tools from-kml pista.kml pista.csv --name pista_ufla
    ros2 run vida_navigation path_tools from-recording volta_bruta.csv pista.csv
    ros2 run vida_navigation path_tools validate pista.csv
    ros2 run vida_navigation path_tools compare pista_google.csv pista_gravada.csv

Cada comando que gera ou le um caminho salva um grafico (.png) ao lado do
arquivo, colorido pelo raio de curva, e imprime a validacao.
"""
import argparse
import math
import sys

import numpy as np

from vida_navigation.geo import to_local
from vida_navigation.path import PathFile, build_path, plot, read_kml, validate


def _add_validation_args(parser):
    parser.add_argument("--min-radius", type=float, default=4.0,
                        help="raio minimo real do veiculo em metros (padrao 4,0: L efetivo 1,42 m a 20 graus)")
    parser.add_argument("--recommended-radius", type=float, default=6.0,
                        help="raio com folga para o Stanley a 2,5 m/s (padrao 6,0)")
    parser.add_argument("--start-straight", type=float, default=10.0,
                        help="reta inicial exigida em metros (padrao 10: manobra de inicializacao a 2 m/s)")


def _report(path_file, args, png, raw_xy=None, title=None):
    local = path_file.to_local(path_file.start_origin)
    report = validate(local, args.min_radius, args.recommended_radius, args.start_straight)
    print(f"  pontos: {len(local)}  comprimento: {report.length_m:.1f} m  "
          f"{'fechado' if path_file.closed else 'aberto'} ({report.direction})")
    print(f"  raio minimo: {report.min_radius_m:.2f} m  reta inicial: {report.start_straight_m:.1f} m  "
          f"cruzamentos: {report.self_intersections}")
    for problem in report.problems:
        print(f"  PROBLEMA: {problem}")
    for warning in report.warnings:
        print(f"  aviso: {warning}")
    print("  resultado:", "OK" if report.ok else "NAO UTILIZAVEL NO VEICULO")
    plot(local, png, title or path_file.meta.get("name", ""), raw_xy=raw_xy,
         min_turn_radius_m=args.min_radius, recommended_radius_m=args.recommended_radius)
    print(f"  grafico: {png}")
    return report


def _png(filename):
    return filename.rsplit(".", 1)[0] + ".png"


def _gaps(lat, lon, closed, max_gap_m):
    """Lacunas entre pontos consecutivos da entrada (ex.: trecho gravado sem RTK FIXED)."""
    x, y = to_local(PathFile(np.asarray(lat), np.asarray(lon), closed).start_origin, lat, lon)
    if closed:
        x, y = np.r_[x, x[0]], np.r_[y, y[0]]
    d = np.hypot(np.diff(x), np.diff(y))
    return [(int(i), float(d[i])) for i in np.where(d > max_gap_m)[0]]


def _build_and_save(lat, lon, closed, args, source, max_gap_m=None):
    gaps = _gaps(lat, lon, closed, max_gap_m) if max_gap_m else []
    out_lat, out_lon = build_path(lat, lon, closed, args.spacing, args.smoothing)
    path_file = PathFile(np.asarray(out_lat), np.asarray(out_lon), closed, {
        "name": args.name or args.output.rsplit("/", 1)[-1].rsplit(".", 1)[0],
        "source": source,
        "spacing_m": f"{args.spacing}",
        "smoothing_m": f"{args.smoothing}",
    })
    path_file.save(args.output)
    print(f"{args.output}:")
    origin = path_file.start_origin
    raw_x, raw_y = to_local(origin, lat, lon)
    report = _report(path_file, args, _png(args.output), raw_xy=(raw_x, raw_y))
    for index, gap in gaps:
        print(f"  PROBLEMA: lacuna de {gap:.1f} m na gravacao (apos o ponto {index}): trecho sem "
              "RTK FIXED ou gravacao interrompida; o caminho ali foi inventado pela spline")
    return 0 if report.ok and not gaps else 2


def cmd_from_kml(args):
    lat, lon, closed = read_kml(args.input)
    if args.closed is not None:
        closed = args.closed
    return _build_and_save(lat, lon, closed, args, "google_earth")


def cmd_from_recording(args):
    raw = PathFile.load(args.input)
    closed = raw.closed if args.closed is None else args.closed
    return _build_and_save(raw.latitude_deg, raw.longitude_deg, closed, args, "ekf_recording",
                           max_gap_m=args.max_gap)


def cmd_validate(args):
    path_file = PathFile.load(args.input)
    print(f"{args.input}:")
    report = _report(path_file, args, _png(args.input))
    return 0 if report.ok else 2


def cmd_compare(args):
    a = PathFile.load(args.first)
    b = PathFile.load(args.second)
    origin = a.start_origin
    pa, pb = a.to_local(origin), b.to_local(origin)
    distances, offsets = [], []
    for x, y in zip(pa.x, pa.y):
        proj = pb.project(x, y)
        distances.append(math.hypot(x - proj.x, y - proj.y))
        offsets.append((x - proj.x, y - proj.y))
    distances = np.array(distances)
    offset = np.mean(offsets, axis=0)
    print(f"{args.first} x {args.second}:")
    print(f"  distancia entre os caminhos: media {distances.mean():.2f} m, "
          f"p95 {np.percentile(distances, 95):.2f} m, maxima {distances.max():.2f} m")
    print(f"  deslocamento medio de '{args.first}' em relacao a '{args.second}': "
          f"{offset[0]:+.2f} m leste, {offset[1]:+.2f} m norte "
          f"(tipico de imagem de satelite deslocada)")
    png = args.plot or _png(args.first).replace(".png", "_comparacao.png")
    plot(pa, png, f"{a.meta.get('name', 'A')} (cor) x {b.meta.get('name', 'B')} (azul)",
         other=pb, other_label=b.meta.get("name", args.second))
    print(f"  grafico: {png}")
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(prog="path_tools", description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)

    def build_args(p):
        p.add_argument("input")
        p.add_argument("output")
        p.add_argument("--name", default="")
        p.add_argument("--spacing", type=float, default=0.25, help="espacamento dos pontos (m)")
        group = p.add_mutually_exclusive_group()
        group.add_argument("--closed", dest="closed", action="store_true", default=None)
        group.add_argument("--open", dest="closed", action="store_false")
        _add_validation_args(p)

    p = sub.add_parser("from-kml", help="Google Earth (KML) -> caminho")
    build_args(p)
    p.add_argument("--smoothing", type=float, default=0.0,
                   help="suavizacao (m); 0 passa pelos cliques (padrao)")
    p.set_defaults(func=cmd_from_kml)

    p = sub.add_parser("from-recording", help="volta gravada (path_recorder) -> caminho")
    build_args(p)
    p.add_argument("--smoothing", type=float, default=0.15,
                   help="desvio-padrao do ruido da gravacao (m), padrao 0.15 (degraus do GPS a 1 Hz)")
    p.add_argument("--max-gap", type=float, default=1.0,
                   help="maior distancia aceita entre pontos gravados (m), padrao 1.0")
    p.set_defaults(func=cmd_from_recording)

    p = sub.add_parser("validate", help="valida um caminho e gera o grafico")
    p.add_argument("input")
    _add_validation_args(p)
    p.set_defaults(func=cmd_validate)

    p = sub.add_parser("compare", help="compara dois caminhos (ex.: Google x gravado)")
    p.add_argument("first")
    p.add_argument("second")
    p.add_argument("--plot", default="")
    p.set_defaults(func=cmd_compare)

    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
