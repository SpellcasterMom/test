"""Форма рабочей зоны: сэмплинг FK -> воксели -> marching cubes -> STL.
Два файла: reach_outer.stl (наружная поверхность) и reach_inner.stl
(стенки внутренних пустот — куда рука НЕ достаёт: труба вокруг оси,
зона под брюхом, (0,0,0) внутри базы).

Запуск из этого каталога: python3 workspace_stl.py  (~3-5 мин)
Треки: numpy + skimage + scipy.
"""
import math
import os
import random
import struct
import sys

import numpy as np
from scipy import ndimage
from skimage import measure

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fk import fk_chain

DH2 = [[0, 90, 134, 0], [165, 0, 0, 90], [161, 0, 0, 0],
       [0, -90, 0, -90], [53, 90, 54, 90], [0, 0, 27, 0]]
F = 105.0
LIM = [(-135, 135)] * 3 + [(-90, 90)] * 3

VOX = 12.0
XMIN, XMAX = -600.0, 600.0
YMIN, YMAX = -600.0, 600.0
ZMIN, ZMAX = 0.0, 700.0
N = 800000

HERE = os.path.dirname(os.path.abspath(__file__))


def write_stl(path, verts, faces):
    with open(path, "wb") as f:
        header = b"6DOF workspace (twin)"
        f.write(header + b"\0" * (80 - len(header)))
        f.write(struct.pack("<I", len(faces)))
        for tri in faces:
            a, b, c = verts[tri[0]], verts[tri[1]], verts[tri[2]]
            n = np.cross(b - a, c - a)
            L = np.linalg.norm(n)
            n = n / L if L > 0 else np.array([0.0, 0.0, 1.0])
            f.write(struct.pack("<3f", *n))
            f.write(struct.pack("<3f", *a))
            f.write(struct.pack("<3f", *b))
            f.write(struct.pack("<3f", *c))
            f.write(struct.pack("<H", 0))


def main():
    random.seed(3)
    nx = int((XMAX - XMIN) / VOX)
    ny = int((YMAX - YMIN) / VOX)
    nz = int((ZMAX - ZMIN) / VOX)
    occ = np.zeros((nx, ny, nz), dtype=bool)
    for _ in range(N):
        q = [random.uniform(*LIM[i]) for i in range(6)]
        r = fk_chain(q, DH2, finger=F)
        x, y, z = r["tcp"]
        if XMIN <= x < XMAX and YMIN <= y < YMAX and ZMIN <= z < ZMAX:
            occ[int((x - XMIN) / VOX), int((y - YMIN) / VOX),
                int((z - ZMIN) / VOX)] = True
    print(f"вокселей: {occ.sum()} (сетка {nx}x{ny}x{nz}, воксель {VOX}мм)")

    # закрыть одиночные дырки сэмплинга (настоящие пустоты много больше)
    occ = ndimage.binary_closing(occ, structure=np.ones((3, 3, 3)), iterations=1)

    # фон, связанный с границей = экстерьер; остальной фон = пустоты
    inv = ~occ
    lab, nlab = ndimage.label(inv, structure=np.ones((3, 3, 3)))
    border = set(np.unique(lab[0, :, :])) | set(np.unique(lab[-1, :, :])) | \
        set(np.unique(lab[:, 0, :])) | set(np.unique(lab[:, -1, :])) | \
        set(np.unique(lab[:, :, 0])) | set(np.unique(lab[:, :, -1]))
    border.discard(0)
    void_mask = np.zeros_like(inv)
    for lb in range(1, nlab + 1):
        if lb not in border:
            void_mask |= (lab == lb)
    print(f"компонент пустот внутри: {nlab - len(border)}, "
          f"вокселей пустот: {int(void_mask.sum())}")
    # где пустоты (центроиды крупняка)
    sizes = [(int((lab == lb).sum()),
              tuple(round(float(x), 0) for x in ndimage.center_of_mass(lab == lb)))
             for lb in range(1, nlab + 1) if lb not in border]
    sizes.sort(reverse=True)
    for s, c in sizes[:5]:
        ix, iy, iz = (int(v) for v in c)
        print(f"  пустота {s} вокселей около "
              f"({XMIN + ix * VOX:.0f},{YMIN + iy * VOX:.0f},{ZMIN + iz * VOX:.0f})")

    origin = np.array([XMIN, YMIN, ZMIN])
    sp = (VOX, VOX, VOX)
    verts, faces, _, _ = measure.marching_cubes(occ.astype(np.float32),
                                                level=0.5, spacing=sp)
    verts = verts + origin
    po = os.path.join(HERE, "reach_outer.stl")
    write_stl(po, verts, faces)
    print(f"outer: {len(faces)} тр., {os.path.getsize(po)/1e6:.1f} МБ -> {po}")

    if void_mask.sum() > 0:
        verts, faces, _, _ = measure.marching_cubes(void_mask.astype(np.float32),
                                                    level=0.5, spacing=sp)
        verts = verts + origin
        pi = os.path.join(HERE, "reach_inner.stl")
        write_stl(pi, verts, faces)
        print(f"inner: {len(faces)} тр., {os.path.getsize(pi)/1e6:.1f} МБ -> {pi}")
    else:
        print("внутренних пустот нет")


if __name__ == "__main__":
    main()
