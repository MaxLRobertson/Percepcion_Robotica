"""Grafica un CSV del registrador.

Uso: graficar_comparacion archivo.csv [--ref /verdad/odom] [--salida carpeta]

Si hay un topico de referencia en el CSV, ademas del trazado de cada topico se
grafica el error de los demas contra esa referencia. Sin referencia (por ejemplo
con el MPU real) se comparan los topicos entre si.
"""
import argparse
import os

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def leer(archivo):
    with open(archivo) as f:
        cab = f.readline().strip().split(",")
        datos = {}
        for linea in f:
            c = linea.strip().split(",")
            d = datos.setdefault(c[1], [])
            d.append([float(x) for x in c[:1] + c[2:]])
    return cab, {k: np.array(v) for k, v in datos.items()}


def _err_angulo(a, b):
    return (a - b + 180.0) % 360.0 - 180.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--ref", default="/verdad/odom")
    ap.add_argument("--salida", default=".")
    a = ap.parse_args()

    _, datos = leer(a.csv)
    # columnas de cada array: t x y z vx vy vz roll pitch yaw
    nombres = {"x": 1, "y": 2, "z": 3, "vx": 4, "vy": 5, "vz": 6,
               "roll": 7, "pitch": 8, "yaw": 9}
    ref = datos.get(a.ref)
    otros = [k for k in datos if k != a.ref] if ref is not None else list(datos)
    os.makedirs(a.salida, exist_ok=True)

    # --- Figura 1: magnitudes de cada topico ---------------------------------
    fig, ax = plt.subplots(3, 3, figsize=(15, 9), sharex=True)
    grupos = [["roll", "pitch", "yaw"], ["x", "y", "z"], ["vx", "vy", "vz"]]
    unidades = ["[deg]", "[m]", "[m/s]"]
    titulos = ["Actitud", "Posicion", "Velocidad"]
    for i, (g, u, tt) in enumerate(zip(grupos, unidades, titulos)):
        for j, n in enumerate(g):
            for k, d in datos.items():
                ax[i, j].plot(d[:, 0], d[:, nombres[n]], label=k,
                              lw=2 if k == a.ref else 1)
            ax[i, j].set_title(f"{tt}: {n}")
            ax[i, j].set_ylabel(u)
            ax[i, j].grid(alpha=0.3)
    for j in range(3):
        ax[2, j].set_xlabel("t [s]")
    ax[0, 0].legend()
    fig.tight_layout()
    f1 = os.path.join(a.salida, "magnitudes.png")
    fig.savefig(f1, dpi=110)
    plt.close(fig)

    # --- Figura 2: error contra la referencia (o entre topicos) --------------
    if ref is None and len(datos) >= 2:
        ref_nombre, ref = list(datos.items())[0]
        otros = list(datos)[1:]
    else:
        ref_nombre = a.ref
    f2 = None
    if ref is not None and otros:
        fig, ax = plt.subplots(1, 3, figsize=(15, 4.5), sharex=True)
        for k in otros:
            d = datos[k]
            t = d[:, 0]
            interp = lambda col: np.interp(t, ref[:, 0], ref[:, col])  # noqa: E731
            e_ang = np.sqrt(sum(_err_angulo(d[:, nombres[n]], interp(nombres[n])) ** 2
                                for n in ("roll", "pitch", "yaw")))
            e_vel = np.linalg.norm(d[:, 4:7] - np.column_stack([interp(c) for c in (4, 5, 6)]), axis=1)
            e_pos = np.linalg.norm(d[:, 1:4] - np.column_stack([interp(c) for c in (1, 2, 3)]), axis=1)
            for axi, e in zip(ax, (e_ang, e_vel, e_pos)):
                axi.plot(t, e, label=k)
            print(f"{k} vs {ref_nombre}: al final err actitud={e_ang[-1]:.2f} deg, "
                  f"vel={e_vel[-1]:.3f} m/s, pos={e_pos[-1]:.3f} m")
        for axi, tt, u in zip(ax, ("Error de actitud", "Error de velocidad", "Error de posicion"),
                              ("[deg]", "[m/s]", "[m]")):
            axi.set_title(f"{tt} (respecto a {ref_nombre})")
            axi.set_ylabel(u)
            axi.set_xlabel("t [s]")
            axi.grid(alpha=0.3)
        ax[0].legend()
        fig.tight_layout()
        f2 = os.path.join(a.salida, "errores.png")
        fig.savefig(f2, dpi=110)
        plt.close(fig)

    print("Guardado:", f1, f2 or "(sin figura de errores: hace falta una referencia o dos topicos)")
