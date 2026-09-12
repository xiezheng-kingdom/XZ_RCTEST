"""
阶段 A: 用"两组实测波形联立"反推电机参数

两个版本的固件参数都是已知的, 实测速度摆幅也已知, 联立可把 G 钉死:
  版本1 (Kp_vel=0.08 Ki=0.05 ffB=0.09 无斜率限制 duty±1500): 速度 +30000 ~ -16000
  版本2 (Kp_vel=0.005 Ki=0.40 ffB=0.0 平滑静摩擦 斜率限制 duty±1200): +66000 ~ -32000

版本2 的 +66000 直接给出量级: G×(1200-500) ≈ 66000 ⇒ G ≈ 94

用法: python tools/fit_plant.py
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sim_control import Plant, Ctrl, run, report, TARGET

# 版本1: 之前的固件 (figure 1)
V1 = dict(VMAX=6000.0, KP_POS=2.0, KP_VEL=0.08, KI_VEL=0.05,
          FF_BEMF=0.09, FF_STIC=500.0, FF_SMOOTH=None,
          AMAX=None, DECEL=None, INT_MAX=1500.0, MAX_DUTY=1500)
OBS1 = dict(vpeak=30000.0, vrev=-16000.0)

# 版本2: 当前固件 (figure 2)
V2 = dict(VMAX=6000.0, KP_POS=2.0, KP_VEL=0.005, KI_VEL=0.40,
          FF_BEMF=0.0, FF_STIC=500.0, FF_SMOOTH=150.0,
          AMAX=8000.0, DECEL=9000.0, INT_MAX=800.0, MAX_DUTY=1200)
OBS2 = dict(vpeak=66000.0, vrev=-32000.0)


def err_for(params, obs, kw):
    r = report("", run(Ctrl(TARGET, **params), Plant(**kw)), TARGET, True)
    e = ((r['vpeak'] - obs['vpeak']) / obs['vpeak']) ** 2
    e += ((r['vrev'] - obs['vrev']) / abs(obs['vrev'])) ** 2
    return e, r


print("=" * 84)
print("反推电机参数 (联立两组实测波形: G 由 +66000 钉死)")
print("=" * 84)
print(f"{'cost':>7}{'D':>5}{'G':>5}{'tau':>7}{'asym':>6} | "
      f"{'v1峰值':>8}{'v1反向':>8} | {'v2峰值':>8}{'v2反向':>8} | 目标 30000/-16000, 66000/-32000")

best = []
for D in (400.0, 500.0, 600.0):
    for G in range(30, 165, 5):
        for tau in (0.003, 0.005, 0.008, 0.012, 0.020):
            for asym in (1.0, 1.3, 1.6, 2.0):
                kw = dict(D=D, G=float(G), tau=tau, asym=asym)
                e1, r1 = err_for(V1, OBS1, kw)
                e2, r2 = err_for(V2, OBS2, kw)
                best.append((e1 + e2, D, G, tau, asym, r1, r2))

best.sort(key=lambda x: x[0])
for c, D, G, tau, asym, r1, r2 in best[:12]:
    print(f"{c:7.3f}{D:5.0f}{G:5.0f}{tau*1000:6.0f}m{asym:6.1f} | "
          f"{r1['vpeak']:8.0f}{r1['vrev']:8.0f} | {r2['vpeak']:8.0f}{r2['vrev']:8.0f}")

print()
e, D, G, tau, asym, r1, r2 = best[0]
print(f"最佳拟合: D={D:.0f}  G={G:.0f} counts/s per duty  tau={tau*1000:.0f}ms  asym={asym:.1f}")
print(f"  → 巡航 6000 counts/s 所需 duty = {D:.0f} + 6000/{G:.0f} = {D + 6000/G:.0f}")
print(f"  → 版本2 满限幅 1200 时的稳态速度 = ({1200-500:.0f})×{G:.0f} = {(1200-500)*G:.0f} counts/s")

# 把 top-N 拟合作为不确定域, 供寻优使用
GRID = []
for c, D, G, tau, asym in best[:12]:
    GRID.append(dict(D=float(D), G=float(G), tau=tau, asym=asym))
for G in (30.0, 50.0, 95.0, 140.0):
    for tau in (0.003, 0.008, 0.020):
        GRID.append(dict(D=500.0, G=G, tau=tau))
        GRID.append(dict(D=500.0, G=G, tau=tau, asym=1.6))
seen, PLANTS = set(), []
for g in GRID:
    k = (g.get('D'), g.get('G'), g.get('tau'), g.get('asym', 1.0))
    if k not in seen:
        seen.add(k); PLANTS.append(g)

with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'plants.py'), 'w') as f:
    f.write("# 自动生成 by fit_plant.py —— 电机参数不确定域\nPLANTS = [\n")
    for g in PLANTS:
        f.write(f"    {g!r},\n")
    f.write("]\n")
print(f"\n不确定域已写出: tools/plants.py ({len(PLANTS)} 个工况)")
