#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MPCC 彗差改正镜 3D 打印件生成器 v2 (纯 Python, 无第三方依赖, 输出二进制 STL)

v2 按牧夫天文网 莴苣 的 DIY 1X MPCC (帖子 thread-811154) 公布的数据重做:
  * 光路顺序: 进光口 -> 负透镜(27mm) -> 平凸镜(41.5mm, 平面朝负透镜, 凸面朝相机) -> 相机
  * 两镜间距: 中心 7.754mm / 边缘 5.337mm  (隔圈边缘厚度 5.337)
  * 后截距 50.5mm: 平凸镜凸面顶点 -> 相机感光面
  本项目只是按该数据为 "2寸插入筒进 + M48 出" 做镜筒, 并非莴苣的原文件。

零件:
  body.stl            主体: 2" 插入筒 + 法兰 + M48x0.75 外螺纹出口 + 平凸镜座
  spacer_gap*.stl     隔圈 (内含负透镜座), 名字里的数字 = 两镜边缘间距 mm
  ring.stl            M44x1 压圈
  coupon_*.stl        螺纹试配块

装配 (镜片都从进光口装入, 依次): 平凸镜(凸面朝出光口) -> 隔圈 -> 负透镜(放进隔圈的座里) -> 压圈
!! 请用卡尺实测镜片"边缘厚度", 改 LA_T / LB_T 再运行 !!
"""
import math
import os
import struct
from collections import Counter

# ======================= 参数 (单位 mm) =======================
# --- 镜片 ---
LA_D, LA_T = 41.5, 3.4     # 平凸镜 f=163 : 直径, 边缘厚度 (由莴苣 P2/P3 推算约 3.4, 请实测)
LB_D, LB_T = 27.0, 3.0     # 负透镜 f=-185: 直径, 边缘厚度 (莴苣 P2 镜座深 3.0, 请实测)
LENS_CLR = 0.3             # 镜片座直径余量
FA, FB = 163.0, -185.0
N_GLASS = 1.5168           # K9 玻璃折射率, 仅用于计算凸面矢高

# --- 莴苣帖子数据 ---
GAP_EDGE, GAP_CENTER = 5.337, 7.754     # 两镜边缘/中心间距
BACK_FOCUS = 50.5                       # 平凸镜凸面顶点 -> 感光面
GAP_VARIANTS = [GAP_EDGE - 0.5, GAP_EDGE, GAP_EDGE + 0.5]   # 多打两个隔圈供微调
CAM_FLANGE = 17.5          # 相机法兰距, 这里是帖子里 ASI533 的数值, 请改成你的相机

# --- 2 寸进光插入筒 ---
BARREL_OD = 50.4
BARREL_LEN = 25.0
GROOVE_AT, GROOVE_W, GROOVE_D = 17.0, 2.0, 0.5
FLANGE_OD, FLANGE_T = 58.0, 2.0

# --- 出光口 ---
OUT_MODE = "m48"           # "m48" | "barrel"
NECK_LEN = 8.0
M48_MAJOR_D, M48_PITCH, M48_DEPTH = 47.7, 0.75, 0.45

# --- 镜片座 ---
SHOULDER_T = 2.0           # 平凸镜出光端挡肩厚度
SHOULDER_AP_D = 37.0       # 挡肩通光孔 (平凸镜凸面从这里探出)
B_AP_RING_D = 26.4         # 压圈一侧负透镜通光孔径 (莴苣 P2 挡边 26.4)
B_AP_SPACER_D = 25.5       # 隔圈一侧负透镜通光孔径 (莴苣 P3 内径 25.5)
LIP_FAR_D = 30.0           # 隔圈内锥孔在平凸镜一侧的直径

# --- 压圈 (M44x1) ---
RING_T = 4.5
RET_MINOR_D, RET_PITCH, RET_DEPTH, RET_CLR = 43.0, 1.0, 0.6, 0.3

NTH = 180
OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "stl")


# ======================= 几何内核 =======================
def wave(phi):
    p = phi - math.floor(phi)
    return min(1.0, max(0.0, 1.25 - 2.5 * abs(p - 0.5)))


def const(r):
    return lambda th, z: r


def thread(r0, depth, pitch):
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
        T.append((O[0][j], I[0][j], O[0][j2]))
        T.append((O[0][j2], I[0][j], I[0][j2]))
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


# ======================= 尺寸推导 =======================
R_IMIN = RET_MINOR_D / 2
R_IMAJ = R_IMIN + RET_DEPTH
R_EMAJ = R_IMAJ - RET_CLR
R_EROOT = R_EMAJ - RET_DEPTH
R_PLAIN = R_IMAJ + 0.2             # 进光端光孔 (压圈要能通过)
R_A_SEAT = (LA_D + LENS_CLR) / 2
R_B_SEAT = (LB_D + LENS_CLR) / 2
R_SP = R_A_SEAT - 0.2
R_B_AP_RING = B_AP_RING_D / 2
R_B_AP_SP = B_AP_SPACER_D / 2
R_AP_A = SHOULDER_AP_D / 2
RC_A = (N_GLASS - 1.0) * FA        # 平凸镜凸面曲率半径


def sag(R, r):
    return R - math.sqrt(R * R - r * r)


def sp_len(gap):
    return gap + LB_T + 0.1        # 隔圈长 = 边缘间距 + 负透镜座深度


def body_dims():
    RB, RF = BARREL_OD / 2, FLANGE_OD / 2
    cone = RF - RB
    z_neck0 = BARREL_LEN + cone + FLANGE_T
    L = z_neck0 + NECK_LEN
    z_sh = L - SHOULDER_T                       # 挡肩进光侧表面
    sp_max, sp_min = sp_len(max(GAP_VARIANTS)), sp_len(min(GAP_VARIANTS))
    z_t0 = z_sh - LA_T - sp_max - RING_T - 1.0
    z_t1 = z_sh - LA_T - sp_min + 2.0
    vertex_z = z_sh + sag(RC_A, R_AP_A)         # 平凸镜凸面顶点位置
    return dict(RB=RB, RF=RF, cone=cone, z_neck0=z_neck0, L=L, z_sh=z_sh,
                z_t0=z_t0, z_t1=z_t1, vertex_z=vertex_z)


# ======================= 零件 =======================
def make_body():
    d = body_dims()
    RB, RF, L = d["RB"], d["RF"], d["L"]
    assert d["z_t0"] > 1.0, "进光端预留不足"
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

    I = [(0.0, const(R_PLAIN)), (d["z_t0"], const(R_PLAIN))]
    add_rows(I, d["z_t0"], d["z_t1"], thread(R_IMIN, RET_DEPTH, RET_PITCH), RET_PITCH / 10)
    I += [(d["z_t1"], const(R_A_SEAT)),
          (d["z_sh"], const(R_A_SEAT)),
          (d["z_sh"], const(R_AP_A)),
          (L, const(R_AP_A))]
    return O, I


def make_ring():
    O = []
    add_rows(O, 0.0, RING_T, thread(R_EROOT, RET_DEPTH, RET_PITCH), RET_PITCH / 10)
    r0 = 17.0
    nf = notched(r0, r0 + 2.6, 4, 8)
    I = [(0.0, nf), (2.0, nf), (2.0, const(r0)),
         (RING_T - 0.6, const(R_B_AP_RING)), (RING_T, const(R_B_AP_RING))]
    return O, I


def make_spacer(gap):
    """z=0 为平凸镜一侧端面, z=L 为进光口一侧端面"""
    S = sp_len(gap)
    O = [(0.0, const(R_SP)), (S, const(R_SP))]
    I = [(0.0, const(LIP_FAR_D / 2)),
         (gap, const(R_B_AP_SP)),          # 内锥孔: 平凸镜侧宽, 负透镜侧窄
         (gap, const(R_B_SEAT)),           # 负透镜座
         (S, const(R_B_SEAT))]
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


# ======================= 主程序 =======================
def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    parts = {"body": make_body(), "ring": make_ring(),
             "coupon_m44_int": make_coupon_m44(), "coupon_m48_ext": make_coupon_m48()}
    for g in GAP_VARIANTS:
        parts["spacer_gap%.2f" % g] = make_spacer(g)

    # 清理 v1 遗留的旧隔圈
    for fn in os.listdir(OUT_DIR):
        if fn.startswith("spacer_") and not fn.startswith("spacer_gap"):
            os.remove(os.path.join(OUT_DIR, fn))

    print("%-20s %9s %8s %10s" % ("part", "tris", "bad_edge", "vol(mm3)"))
    for name, (O, I) in parts.items():
        V, T = tube(O, I)
        V, T = weld(V, T)
        bad, vol = check(V, T)
        write_stl(os.path.join(OUT_DIR, name + ".stl"), V, T, "mpcc_" + name)
        print("%-20s %9d %8d %10.0f" % (name, len(T), bad, vol))

    d = body_dims()
    to_shoulder = BACK_FOCUS + (d["vertex_z"] - d["z_neck0"])
    print("\n--- 尺寸核对 ---")
    print("主体总长 %.1f mm; 平凸镜凸面顶点比出光端面%s %.2f mm" %
          (d["L"], "多伸出" if d["vertex_z"] > d["L"] else "缩进", abs(d["vertex_z"] - d["L"])))
    print("M48 台阶面(转接环拧到底的位置) -> 感光面 应为 %.1f mm" % to_shoulder)
    print("  = 相机法兰距 %.1f + 延长筒/转接环 %.1f mm  (法兰距请按你的相机改 CAM_FLANGE)" %
          (CAM_FLANGE, to_shoulder - CAM_FLANGE))
    print("\n--- 说明 ---")
    print("镜片件号 356 / 482 与莴苣装配图一致; 356 为负弯月形, 所以中心-边缘间距差 %.2f mm 合理" % (GAP_CENTER - GAP_EDGE))
    print("LA_T=%.1f 由莴苣 P2/P3 推算 (8.78-5.337), LB_T=%.1f 来自其 P2 镜座深度; 都请用卡尺实测" % (LA_T, LB_T))

if __name__ == "__main__":
    main()
