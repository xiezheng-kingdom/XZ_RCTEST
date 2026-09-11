"""
开环辨识结果计算: 从各档 duty 的稳态速度解出电机静态参数 (死区 D 与增益 G)。

背景: 之前所有调参失败都源于 G 是猜的 (先后按 11 / 30 / 94 估)。
本脚本用实测的两点法直接算出, 并给出速度环前馈基线的唯一正确形式。

用法:
    python tools/identify.py 450 550 650 750 850 --vel 0 4200 9800 15000 20500
    (duty 档位必须与固件 main.c 的 IDENT_DUTY 一致; --vel 填各档稳态速度)
"""
import sys, argparse


def polyfit(xs, ys):
    """最小二乘一次拟合 y = a*x + b, 返回 (a, b)"""
    n = len(xs)
    sx, sy = sum(xs), sum(ys)
    sxx = sum(x * x for x in xs)
    sxy = sum(x * y for x, y in zip(xs, ys))
    den = n * sxx - sx * sx
    if den == 0:
        return 0.0, 0.0
    a = (n * sxy - sx * sy) / den
    b = (sy - a * sx) / n
    return a, b


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('duty', nargs='+', type=float, help='施加的 duty 档位')
    ap.add_argument('--vel', nargs='+', type=float, required=True,
                    help='对应各档的稳态速度 counts/s (从遥测通道1读)')
    a = ap.parse_args()
    if len(a.duty) != len(a.vel):
        sys.exit(f"档位数量不一致: {len(a.duty)} duty vs {len(a.vel)} vel")

    print("=" * 66)
    print("实测点:")
    for d, v in zip(a.duty, a.vel):
        print(f"    duty={d:6.0f}  ->  vel={v:9.0f} counts/s")

    # 只用"能转动"的点 (速度明显 > 0) 拟合直线 V = G*(duty - D) = G*duty - G*D
    pts = [(d, v) for d, v in zip(a.duty, a.vel) if v > 200.0]
    if len(pts) < 2:
        print("\n可用点不足 2 个 (电机几乎没转) —— 请加大 duty 档位重测。")
        return
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    G, b = polyfit(xs, ys)
    D = -b / G if G != 0 else float('nan')

    print()
    print("=" * 66)
    print("拟合结果  V = G * (duty - D)")
    print("=" * 66)
    print(f"    G (速度增益) = {G:9.2f} counts/s per duty")
    print(f"    D (静摩擦死区) = {D:9.1f} duty")

    resid = max(abs(v - (G * d + b)) for d, v in pts)
    print(f"    最大拟合残差 = {resid:9.0f} counts/s "
          f"({resid / max(abs(v) for _, v in pts) * 100:.1f}% of 峰值)")

    print()
    print("=" * 66)
    print("由此得到的前馈基线 (速度环只需修小残差, 不再靠积分爬坡)")
    print("=" * 66)
    for v in (1000.0, 3000.0, 6000.0):
        print(f"    vel_ref={v:6.0f}  需要 duty = D + vel_ref/G = {D + v / G:7.1f}")
    print()
    print("对应固件宏:")
    print(f"    #define MC_FF_STIC    {D:.0f}.0f      /* 静摩擦死区前馈 */")
    print(f"    #define MC_FF_BEMF    {1.0 / G:.5f}f     /* 反电动势前馈 = 1/G */")
    print()
    print("* 注意: 前馈的斜率必须等于 1/G。此前用 0.09 (隐含 G≈11) 和")
    print(f"  0.037 (隐含 G≈27) 都与实测 G≈{G:.0f} 差很远 —— 这就是历次振荡的根源。")
    print()
    print("建议的 duty 限幅下限 (保证巡航可达, 只留 1.3 倍裕度):")
    print(f"    MC_MAX_DUTY = {(D + 6000.0 / G) * 1.3:.0f}")


if __name__ == '__main__':
    main()
