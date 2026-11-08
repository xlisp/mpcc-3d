#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MPCC 彗差改正镜 3D 打印件生成器 (纯 Python, 无第三方依赖, 输出二进制 STL)

用于: 信达老黑 10 寸牛反 F5
光路 (从进光口到出光口):  2" 插入筒 -> 压圈 -> 平凸镜(A) -> 隔圈 -> 负透镜(B) -> M48 / 2" 出口
装配: 镜片都从进光口(2"插入筒这一端)装入, 先放 B, 再放隔圈, 再放 A, 最后拧压圈。

零件:
  body.stl            主体: 2" 插入筒 + 止口法兰 + 出口(M48x0.75 外螺纹)
  ring.stl            压圈 (M44x1 外螺纹, 进光面有 4 个缺口方便拧)
  spacer_XX.stl       不同长度的隔圈 (用来调两镜间距, 实测找最佳)
  coupon_m44_int.stl  M44 内螺纹试配小块 (配 ring 试螺纹松紧)
  coupon_m48_ext.stl  M48 外螺纹试配小块 (配你的 M48 接口试松紧)

!! 请用卡尺实测镜片"边缘厚度", 改下面 LA_T / LB_T 再运行 !!
"""
import math
import os
import struct
from collections import Counter

# ======================= 参数 (单位 mm) =======================
# --- 镜片 ---
LA_D, LA_T = 41.5, 4.0     # 平凸镜 f=163 : 直径, 边缘厚度(请实测)
LB_D, LB_T = 27.0, 3.0     # 负透镜 f=-185: 直径, 边缘厚度(请实测)
LENS_CLR = 0.3             # 镜片座直径余量
FA, FB = 163.0, -185.0     # 焦距, 仅用于近轴估算表

# --- 2 寸进光插入筒 ---
BARREL_OD = 50.4           # 调焦座内径 50.8, 留 0.4 打印余量
BARREL_LEN = 30.0
GROOVE_AT, GROOVE_W, GROOVE_D = 22.0, 2.0, 0.5   # 顶丝槽 (距进光口位置, 宽, 深)
FLANGE_OD, FLANGE_T = 58.0, 2.0                  # 止口法兰 (下方 45° 锥面, 免支撑)

# --- 出光口 ---
OUT_MODE = "m48"           # "m48": M48x0.75 外螺纹(接 2" 滤镜口/转接环) | "barrel": 2" 插入筒
NECK_LEN = 8.0
M48_MAJOR_D, M48_PITCH, M48_DEPTH = 47.7, 0.75, 0.45   # 外螺纹大径留 0.3 间隙

# --- 镜片座 ---
SHOULDER_AP_D = 24.0       # 出光端 B 镜后的通光孔
SHOULDER_T = 1.5
SPACERS = [10, 14, 18, 22, 26]   # 隔圈长度 (近似两镜间距)
SPACER_AP_A, SPACER_AP_B = 35.0, 25.0   # 隔圈内锥孔直径 (A端 -> B端)

# --- 压圈 (M44x1) ---
RING_T = 4.5
RING_AP_D = 35.0
RET_MINOR_D, RET_PITCH, RET_DEPTH, RET_CLR = 43.0, 1.0, 0.6, 0.3

NTH = 180                  # 圆周分段
OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "stl")


# ======================= 几何内核 =======================
def wave(phi):
    p = phi - math.floor(phi)
    return min(1.0, max(0.0, 1.25 - 2.5 * abs(p - 0.5)))


def const(r):
    return lambda th, z: r


def thread(r0, depth, pitch):
    """右旋螺纹: 半径 = r0(牙底) + depth * wave"""
    return lambda th, z: r0 + depth * wave(z / pitch - th / (2 * math.pi))


def notched(r0, r1, n, half_deg):
    half = math.radians(half_deg)

    def f(th, z):
        for k in range(n):
            c = 2 * math.pi * k / n
            d = abs((th - c + math.pi) % (2 * math.pi) - math.pi)
            if d <= half:
                return r1
        return r0
    return f


def add_rows(prof, z0, z1, fn, step):
    n = max(1, int(round((z1 - z0) / step)))
    for k in range(n + 1):
        prof.append((z0 + (z1 - z0) * k / n, fn))


def tube(outer, inner):
    """outer/inner: [(z, fn(theta,z)->r)], 首尾 z 必须一致。返回 (V, T) 封闭实体"""
    assert abs(outer[0][0] - inner[0][0]) < 1e-9
    assert abs(outer[-1][0] - inner[-1][0]) < 1e-9
    V, T = [], []

    def rows(prof):
        res = []
        for z, fn in prof:
            row = []
            for j in range(NTH):
                th = 2 * math.pi * j / NTH
                r = fn(th, z)
                V.append((r * math.cos(th), r * math.sin(th), z))
                row.append(len(V) - 1)
            res.append(row)
        return res

    O, I = rows(outer), rows(inner)
    for rws, flip in ((O, False), (I, True)):
        for i in range(len(rws) - 1):
            a, b = rws[i], rws[i + 1]
            for j in range(NTH):
                j2 = (j + 1) % NTH
                t1 = (a[j], a[j2], b[j])
                t2 = (a[j2], b[j2], b[j])
                if flip:
                    t1, t2 = t1[::-1], t2[::-1]
                T.append(t1)
                T.append(t2)
    for j in range(NTH):
        j2 = (j + 1) % NTH
        # 底面 (-z)
        T.append((O[0][j], I[0][j], O[0][j2]))
        T.append((O[0][j2], I[0][j], I[0][j2]))
        # 顶面 (+z)
        T.append((O[-1][j], O[-1][j2], I[-1][j]))
        T.append((O[-1][j2], I[-1][j2], I[-1][j]))
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


# ======================= 零件 =======================
R_IMIN = RET_MINOR_D / 2
R_IMAJ = R_IMIN + RET_DEPTH
R_EMAJ = R_IMAJ - RET_CLR
R_EROOT = R_EMAJ - RET_DEPTH
R_A_SEAT = (LA_D + LENS_CLR) / 2
R_B_SEAT = (LB_D + LENS_CLR) / 2
R_SP = R_A_SEAT - 0.2          # 隔圈外半径, 能滑入 A 座


def body_dims():
    RB, RF = BARREL_OD / 2, FLANGE_OD / 2
    cone = RF - RB                                # 45° 锥
    z_neck0 = BARREL_LEN + cone + FLANGE_T
    cb = LB_T + 0.1                               # B 座深度
    L = z_neck0 + NECK_LEN
    z_b_top = L - SHOULDER_T - cb                 # B 座顶 (隔圈从这里往进光口方向排)
    z_thr_end = z_b_top - min(SPACERS) - LA_T + 0.5
    ring_front_min = z_b_top - max(SPACERS) - LA_T - RING_T
    return dict(RB=RB, RF=RF, cone=cone, z_neck0=z_neck0, cb=cb, L=L,
                z_b_top=z_b_top, z_thr_end=z_thr_end, ring_front_min=ring_front_min)


def make_body():
    d = body_dims()
    RB, RF, L = d["RB"], d["RF"], d["L"]
    assert d["ring_front_min"] > 0.5, "压圈会伸出进光口, 请减小 SPACERS 最大值"
    O = [(0.0, const(RB - 1.0)), (1.0, const(RB)),
         (GROOVE_AT, const(RB)), (GROOVE_AT, const(RB - GROOVE_D)),
         (GROOVE_AT + GROOVE_W, const(RB - GROOVE_D)), (GROOVE_AT + GROOVE_W, const(RB)),
         (BARREL_LEN, const(RB)),
         (BARREL_LEN + d["cone"], const(RF)),
         (d["z_neck0"], const(RF))]
    if OUT_MODE == "m48":
        r_root = M48_MAJOR_D / 2 - M48_DEPTH
        O.append((d["z_neck0"], thread(r_root, M48_DEPTH, M48_PITCH)))
        add_rows(O, d["z_neck0"], L, thread(r_root, M48_DEPTH, M48_PITCH), M48_PITCH / 10)
    else:
        O.append((d["z_neck0"], const(RB)))
        O.append((L, const(RB)))

    I = []
    add_rows(I, 0.0, d["z_thr_end"], thread(R_IMIN, RET_DEPTH, RET_PITCH), RET_PITCH / 10)
    I += [(d["z_thr_end"], const(R_A_SEAT)),
          (d["z_b_top"], const(R_A_SEAT)),
          (d["z_b_top"], const(R_B_SEAT)),
          (d["z_b_top"] + d["cb"], const(R_B_SEAT)),
          (d["z_b_top"] + d["cb"], const(SHOULDER_AP_D / 2)),
          (L, const(SHOULDER_AP_D / 2))]
    return O, I


def make_ring():
    O = []
    add_rows(O, 0.0, RING_T, thread(R_EROOT, RET_DEPTH, RET_PITCH), RET_PITCH / 10)
    ra = RING_AP_D / 2
    I = [(0.0, notched(ra, ra + 2.3, 4, 8)), (2.0, notched(ra, ra + 2.3, 4, 8)),
         (2.0, const(ra)), (RING_T, const(ra))]
    return O, I


def make_spacer(S):
    O = [(0.0, const(R_SP)), (float(S), const(R_SP))]
    I = [(0.0, const(SPACER_AP_A / 2)), (float(S), const(SPACER_AP_B / 2))]
    return O, I


def make_coupon_m44():
    O = [(0.0, const(26.0)), (8.0, const(26.0))]
    I = []
    add_rows(I, 0.0, 8.0, thread(R_IMIN, RET_DEPTH, RET_PITCH), RET_PITCH / 10)
    return O, I


def make_coupon_m48():
    r_root = M48_MAJOR_D / 2 - M48_DEPTH
    O = []
    add_rows(O, 0.0, 8.0, thread(r_root, M48_DEPTH, M48_PITCH), M48_PITCH / 10)
    I = [(0.0, const(R_IMIN)), (8.0, const(R_IMIN))]
    return O, I


# ======================= 近轴估算 (仅供参考) =======================
def paraxial(D0, S):
    """D0: 无改正镜时 A 镜到焦点的距离; S: 两镜间距。忽略厚度和像差。"""
    l1 = 1.0 / (1.0 / D0 + 1.0 / FA)
    if l1 <= S:
        return None
    v = 1.0 / (l1 - S) + 1.0 / FB
    if v <= 0:
        return None
    l2 = 1.0 / v
    return l2, l2 * l1 / (D0 * (l1 - S))


# ======================= 主程序 =======================
def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    parts = {"body": make_body(), "ring": make_ring(),
             "coupon_m44_int": make_coupon_m44(), "coupon_m48_ext": make_coupon_m48()}
    for s in SPACERS:
        parts["spacer_%02d" % s] = make_spacer(s)

    print("%-16s %9s %8s %10s  %s" % ("part", "tris", "bad_edge", "vol(mm3)", "file"))
    for name, (O, I) in parts.items():
        V, T = tube(O, I)
        V, T = weld(V, T)
        bad, vol = check(V, T)
        path = os.path.join(OUT_DIR, name + ".stl")
        write_stl(path, V, T, "mpcc_" + name)
        print("%-16s %9d %8d %10.0f  %s" % (name, len(T), bad, vol, path))

    d = body_dims()
    print("\n主体总长 %.1f mm, 插入筒 %.0f mm, 压圈最前位置 %.1f mm (>0 即不会伸出)" %
          (d["L"], BARREL_LEN, d["ring_front_min"]))
    print("近轴估算 (忽略厚度/像差, 仅供参考): 出光后焦距离 BFL(mm) / 焦比缩放系数")
    print("%-8s" % "S\\D0" + "".join("%18s" % ("D0=%d" % x) for x in (80, 100, 130)))
    for s in SPACERS:
        row = "%-8d" % s
        for D0 in (80, 100, 130):
            r = paraxial(D0, s)
            row += "%18s" % ("-" if r is None else "%.1f / x%.2f" % r)
        print(row)


if __name__ == "__main__":
    main()
