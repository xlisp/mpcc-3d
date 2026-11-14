#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MPCC_APSC_P1 替代件: ZWO ASI533MC + 1.25" 母口版  (纯 Python, 无第三方依赖, 输出二进制 STL)

背景
  莴苣的 APS-C MPCC 里, P1 是"相机侧压盖 + 尼康 F 卡口"。
  P2 (2" 镜筒) / P3 (隔圈) / 两片镜片 不变, 只把 P1 换成能接 ZWO 相机的版本。

本件保留 (数值全部从原版 MPCC_APSC_P1.STL 读出, z 以进光端面为 0):
  * 进光端 M46.x 内螺纹: 螺距 1.0, 牙顶 R23.12 / 牙底 R23.67, 长 5.0, 右旋, 相位与原版一致
  * 压住平凸镜的平面台阶: z=5.0, 通光孔 R17.7 (镜片接触圆不变, 所以光学位置不变)
  * 外径 ø50.6

本件新增:
  * 1.25" (31.75mm) 母口: 相机的 1.25" 接筒插进去, 3 颗 M4 顶丝锁紧
  * 相机端面 (接筒法兰贴合的圆环面) 到感光面 = CAM_FACE_TO_SENSOR (ZWO ASI533 为 17.5mm)
  * 总长由"尼康法兰距 46.5"反推, 保证平凸镜凸面顶点到感光面仍约 50.4~50.5mm

坐标: z' = 0 在螺纹进光端面, 沿光路增加, 感光面在 z' = SENSOR_Z。

打印建议: 出光口(1.25"母口)朝下平放, 进光螺纹朝上, 层高 0.12~0.16, 无支撑。
用法:  python3 mpcc_p1_zwo.py [--cam-face-to-sensor 17.5] [--socket-d 32.2] [--screw-d 3.4] [--thread-clr 0.0]
"""
import argparse
import math
import os
import shutil
import struct
from collections import Counter

# ======================= 参数 (单位 mm) =======================
NTH = 360                       # 圆周分段数 (顶丝孔要靠它保证圆度)
DTH = 2 * math.pi / NTH

# --- 与 P2 配合的内螺纹 (从原版 P1 读出) ---
THR_PITCH = 1.0
THR_R_CREST = 23.12             # 牙顶 (最靠近轴线)
THR_R_ROOT = 23.67              # 牙底
THR_CREST_FLAT = 0.20           # 牙顶平台宽 (占螺距的比例)
THR_ROOT_FLAT = 0.10            # 牙底平台宽
THR_CREST_Z0 = 0.875            # θ=0 处牙顶中心 z' (模 1 个螺距), 右旋
THR_LEN = 5.0                   # 螺纹长度 = 压镜台阶所在 z'
THR_CLR = 0.0                   # 额外半径间隙: 打出来拧不进 P2 就加 0.05~0.1
CLAMP_R = 17.7                  # 压镜孔半径 (保持不变!)
BODY_R, BODY_LEN, CONE_LEN = 25.3, 7.8, 8.0

# --- 光学基准 (原版尺寸) ---
ORIG_FACE_Z, ORIG_MOUNT_Z, NIKON_FLANGE = 0.325, 11.13, 46.5
SENSOR_Z = (ORIG_MOUNT_Z - ORIG_FACE_Z) + NIKON_FLANGE      # 感光面 z' = 57.305
FA, N_GLASS = 163.0, 1.5168     # 平凸镜 f, K9 折射率 (只用来核对 50.5)

# --- 相机侧 (ZWO ASI533MC Pro) ---
CAM_FACE_TO_SENSOR = 17.5       # 接筒法兰贴合面 -> 感光面 (ZWO 标称 17.5)
SOCKET_D = 32.2                 # 1.25" 母口孔径 (31.75 + 0.45 打印余量)
SOCKET_DEPTH = 28.0             # 母口深度 (大于接筒长度, 这样法兰能贴到端面)
FLOOR_R = 14.25                 # 母口底部挡肩的通光半径 (ø28.5, 接筒内孔约 ø28)
SOCK_OUT_R = 21.0               # 母口外半径 (ø42)
SCREW_N = 3                     # 顶丝数量
SCREW_D = 3.4                   # 顶丝孔径 (M4 自攻; 太紧用 3.5 钻头过一下)
SCREW_FROM_RIM = 7.0            # 顶丝距端面
SCREW_ANGLE0 = 90               # 第一颗顶丝角度 (度)
HOLE_DZ = 0.2                   # 顶丝孔区域的 z 网格

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "stl")
DATA_DIR = "/Users/xlisp/Documents/APS-C_MPCC资料/3D打印文件"
OUT_NAME = "MPCC_APSC_P1_ZWO125.STL"


# ======================= 几何内核 =======================
def const(r):
    return lambda th, z: r


def thread_in(th, z):
    """内螺纹半径 r(θ,z): 牙顶 THR_R_CREST, 牙底 THR_R_ROOT, 梯形牙型, 右旋"""
    p = (z - THR_CREST_Z0) / THR_PITCH - th / (2 * math.pi)
    d = abs(p - round(p))                      # 到牙顶中心的距离 (螺距为单位, 0..0.5)
    half = THR_CREST_FLAT / 2
    flank = (1.0 - THR_CREST_FLAT - THR_ROOT_FLAT) / 2
    if d <= half:
        t = 0.0
    elif d >= half + flank:
        t = 1.0
    else:
        t = (d - half) / flank
    return THR_R_CREST + THR_CLR + (THR_R_ROOT - THR_R_CREST) * t


def build(outer, inner, mask=None, dense_o=0, dense_i=0, ncell=0):
    """outer/inner: [(z, fn(θ,z)->r)], 首尾 z 必须相同; 同 z 连续两行 = 一个平台阶面。
    mask: {(k,j)} 通孔单元 (k=致密区第 k 层, j=第 j 列), 同时从内外表面挖掉并补孔壁。"""
    assert abs(outer[0][0] - inner[0][0]) < 1e-9 and abs(outer[-1][0] - inner[-1][0]) < 1e-9
    V, T = [], []

    def rows(prof):
        res = []
        for z, fn in prof:
            row = []
            for j in range(NTH):
                th = DTH * j
                r = fn(th, z)
                V.append((r * math.cos(th), r * math.sin(th), z))
                row.append(len(V) - 1)
            res.append(row)
        return res

    O, I = rows(outer), rows(inner)
    mask = mask or set()
    for rws, flip, d0 in ((O, False, dense_o), (I, True, dense_i)):
        for i in range(len(rws) - 1):
            k = i - d0
            a, b = rws[i], rws[i + 1]
            for j in range(NTH):
                if 0 <= k < ncell and (k, j) in mask:
                    continue
                j2 = (j + 1) % NTH
                t1 = (a[j], a[j2], b[j])
                t2 = (a[j2], b[j2], b[j])
                if flip:
                    t1, t2 = t1[::-1], t2[::-1]
                T.append(t1)
                T.append(t2)
    # 端面
    for j in range(NTH):
        j2 = (j + 1) % NTH
        T.append((O[0][j], I[0][j], O[0][j2]))
        T.append((O[0][j2], I[0][j], I[0][j2]))
        T.append((O[-1][j], O[-1][j2], I[-1][j]))
        T.append((O[-1][j2], I[-1][j2], I[-1][j]))

    # 孔壁: 掩膜区域边界上, 连接内外表面对应顶点
    def sub(p, q):
        return (p[0] - q[0], p[1] - q[1], p[2] - q[2])

    def cross(u, w):
        return (u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0])

    def cell_center(k, j):
        pts = [V[I[dense_i + k + dk][(j + dj) % NTH]] for dk in (0, 1) for dj in (0, 1)]
        return tuple(sum(p[c] for p in pts) / 4 for c in range(3))

    for (k, j) in mask:
        j1 = (j + 1) % NTH
        for dk, dj, edge in ((0, -1, "L"), (0, 1, "R"), (-1, 0, "B"), (1, 0, "T")):
            nk, nj = k + dk, (j + dj) % NTH
            if (nk, nj) in mask:
                continue
            if edge == "L":
                ends = ((k, j), (k + 1, j))
            elif edge == "R":
                ends = ((k, j1), (k + 1, j1))
            elif edge == "B":
                ends = ((k, j), (k, j1))
            else:
                ends = ((k + 1, j), (k + 1, j1))
            (ka, ja), (kb, jb) = ends
            Ia, Ib = I[dense_i + ka][ja], I[dense_i + kb][jb]
            Oa, Ob = O[dense_o + ka][ja], O[dense_o + kb][jb]
            n = cross(sub(V[Ib], V[Ia]), sub(V[Ob], V[Ib]))
            want = sub(cell_center(k, j), cell_center(nk, nj))      # 材料 -> 空洞
            if n[0] * want[0] + n[1] * want[1] + n[2] * want[2] < 0:
                T.append((Ia, Ob, Ib))
                T.append((Ia, Oa, Ob))
            else:
                T.append((Ia, Ib, Ob))
                T.append((Ia, Ob, Oa))
    return V, T


def weld(V, T):
    key, nv, remap = {}, [], []
    for v in V:
        k = (round(v[0], 5), round(v[1], 5), round(v[2], 5))
        if k not in key:
            key[k] = len(nv)
            nv.append(v)
        remap.append(key[k])
    nt = []
    for a, b, c in T:
        a, b, c = remap[a], remap[b], remap[c]
        if a == b or b == c or a == c:
            continue
        nt.append((a, b, c))
    return nv, nt


def check(V, T):
    e = Counter()
    for a, b, c in T:
        for u, v in ((a, b), (b, c), (c, a)):
            e[(u, v)] += 1
    bad = sum(1 for (u, v), n in e.items() if n != 1 or e.get((v, u), 0) != 1)
    vol = 0.0
    for a, b, c in T:
        A, B, C = V[a], V[b], V[c]
        vol += (A[0] * (B[1] * C[2] - B[2] * C[1])
                - A[1] * (B[0] * C[2] - B[2] * C[0])
                + A[2] * (B[0] * C[1] - B[1] * C[0]))
    return bad, vol / 6.0


def write_stl(path, V, T, name):
    with open(path, "wb") as f:
        f.write(name.encode("ascii", "ignore")[:79].ljust(80, b" "))
        f.write(struct.pack("<I", len(T)))
        for a, b, c in T:
            A, B, C = V[a], V[b], V[c]
            u = (B[0] - A[0], B[1] - A[1], B[2] - A[2])
            w = (C[0] - A[0], C[1] - A[1], C[2] - A[2])
            n = (u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0])
            m = math.sqrt(n[0] ** 2 + n[1] ** 2 + n[2] ** 2) or 1.0
            f.write(struct.pack("<12fH", n[0] / m, n[1] / m, n[2] / m, *A, *B, *C, 0))


def sag(R, r):
    return R - math.sqrt(R * R - r * r)


# ======================= 主程序 =======================
def main():
    global CAM_FACE_TO_SENSOR, SOCKET_D, SCREW_D, THR_CLR
    ap = argparse.ArgumentParser()
    ap.add_argument("--cam-face-to-sensor", type=float, default=CAM_FACE_TO_SENSOR)
    ap.add_argument("--socket-d", type=float, default=SOCKET_D)
    ap.add_argument("--screw-d", type=float, default=SCREW_D)
    ap.add_argument("--thread-clr", type=float, default=THR_CLR)
    ap.add_argument("--no-copy", action="store_true", help="不复制到资料目录")
    a = ap.parse_args()
    CAM_FACE_TO_SENSOR, SOCKET_D, SCREW_D, THR_CLR = a.cam_face_to_sensor, a.socket_d, a.screw_d, a.thread_clr

    sock_in_r = SOCKET_D / 2
    L = SENSOR_Z - CAM_FACE_TO_SENSOR                    # 总长 = 相机端面 z'
    z_f = L - SOCKET_DEPTH                               # 母口底部挡肩
    z_a = z_f - (CLAMP_R - FLOOR_R)                      # 45° 漏斗起点
    rc = (N_GLASS - 1.0) * FA
    bulge = sag(rc, CLAMP_R)                             # 凸面顶点探出压镜台阶的距离
    assert z_a >= THR_LEN + bulge + 0.6, "母口太深/总长太短: 平凸镜凸面会顶到漏斗, 减小 SOCKET_DEPTH"

    # --- 顶丝孔网格 (内外表面共用同一组单元) ---
    zh = L - SCREW_FROM_RIM
    K = int(math.ceil((SCREW_D / 2 + 0.6) / HOLE_DZ))
    dense = [zh - (K + 0.5) * HOLE_DZ + k * HOLE_DZ for k in range(2 * K + 2)]
    ncell = len(dense) - 1
    col = 360.0 / NTH
    thetas = []
    for n in range(SCREW_N):
        j0 = int(round((SCREW_ANGLE0 + 360.0 * n / SCREW_N) / col))
        thetas.append((j0 + 0.5) * DTH)
    mask = set()
    for k in range(ncell):
        zc = 0.5 * (dense[k] + dense[k + 1])
        for j in range(NTH):
            thc = (j + 0.5) * DTH
            for t0 in thetas:
                d = (thc - t0 + math.pi) % (2 * math.pi) - math.pi
                if (d * sock_in_r) ** 2 + (zc - zh) ** 2 <= (SCREW_D / 2) ** 2:
                    mask.add((k, j))
                    break

    # --- 外轮廓 ---
    outer = [(0.0, const(BODY_R - 0.3)), (0.3, const(BODY_R)), (BODY_LEN, const(BODY_R)),
             (BODY_LEN + CONE_LEN, const(SOCK_OUT_R))]
    dense_o = len(outer)
    outer += [(z, const(SOCK_OUT_R)) for z in dense]
    outer += [(L - 0.5, const(SOCK_OUT_R)), (L, const(SOCK_OUT_R - 0.5))]

    # --- 内轮廓 ---
    inner = [(0.0, const(THR_R_ROOT + THR_CLR + 0.25))]          # 进光口倒角
    nthr = int(round((THR_LEN - 0.5) / 0.1))
    inner += [(0.5 + (THR_LEN - 0.5) * i / nthr, thread_in) for i in range(nthr + 1)]
    inner += [(THR_LEN, const(CLAMP_R)),                         # 压镜台阶
              (z_a, const(CLAMP_R)),
              (z_f, const(FLOOR_R)),                             # 45° 漏斗
              (z_f, const(sock_in_r))]                           # 母口底部挡肩
    dense_i = len(inner)
    inner += [(z, const(sock_in_r)) for z in dense]
    inner += [(L - 0.4, const(sock_in_r)), (L, const(sock_in_r + 0.4))]

    V, T = build(outer, inner, mask, dense_o, dense_i, ncell)
    V, T = weld(V, T)
    bad, vol = check(V, T)
    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, OUT_NAME)
    write_stl(path, V, T, "MPCC_APSC_P1_ZWO125 1.25in socket")
    xs, ys, zs = zip(*V)

    print("=" * 60)
    print("输出:", path)
    print(f"三角面 {len(T)}  非流形/开放边 {bad}  体积 {vol:.1f} mm3 (约 {vol * 1.24e-3:.1f} g PLA)")
    print(f"外形 {max(xs) - min(xs):.2f} x {max(ys) - min(ys):.2f} x {max(zs) - min(zs):.2f} mm")
    print("-" * 60)
    print(f"总长(进光端面 -> 相机端面)       L        = {L:.2f}")
    print(f"感光面位置(原版尼康推算)         SENSOR_Z = {SENSOR_Z:.3f}")
    print(f"压镜台阶 z' = {THR_LEN:.2f}, 平凸镜凸面顶点 z' = {THR_LEN + bulge:.2f} (探出 {bulge:.2f})")
    print(f"凸面顶点 -> 感光面 = {SENSOR_Z - THR_LEN - bulge:.2f} mm (莴苣标称 50.5)")
    print(f"相机端面 -> 感光面 = {CAM_FACE_TO_SENSOR:.2f} mm; 母口 ø{SOCKET_D}, 深 {SOCKET_DEPTH}, 底部挡肩 z'={z_f:.2f}")
    print(f"顶丝孔 {SCREW_N} x ø{SCREW_D}, 距端面 {SCREW_FROM_RIM}, 起始角 {SCREW_ANGLE0}°, 掩膜单元 {len(mask)}")
    print("=" * 60)
    if bad:
        print("!! 网格不是封闭流形, 不要直接打印")
    if not a.no_copy and os.path.isdir(DATA_DIR):
        shutil.copy(path, os.path.join(DATA_DIR, OUT_NAME))
        print("已复制到:", os.path.join(DATA_DIR, OUT_NAME))


if __name__ == "__main__":
    main()
