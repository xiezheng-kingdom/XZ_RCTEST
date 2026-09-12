"""
阶段 3: 在阶段 2 最优解附近做坐标下降精细寻优

阶段 2 最优: Kp=0.02 Ki=0.40 ffB=0.0 aMax=8000 decel=9000 dutyMax=1000
  (反电动势前馈被优化器置零 —— 积分器自身就是在线的 G 估计, 无需猜 G)

用法: python tools/refine_control.py
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sim_control import Plant, Ctrl, run, report, TARGET

# 不确定域 (阶段1 top-10 拟合 + 宽外推)
GRID = []
for e_D_G_tau in [(600, 70, 0.010), (400, 90, 0.015), (500, 95, 0.015),
                  (400, 95, 0.015), (400, 100, 0.015), (500, 100, 0.015),
                  (400, 85, 0.015), (400, 45, 0.010), (600, 100, 0.015),
                  (500, 90, 0.015)]:
    D, G, tau = e_D_G_tau
    GRID.append(dict(D=float(D), G=float(G), tau=tau))
for G in (20.0, 45.0, 90.0, 120.0):
    for tau in (0.008, 0.020, 0.040):
        GRID.append(dict(D=500.0, G=G, tau=tau))
        GRID.append(dict(D=500.0, G=G, tau=tau, asym=1.3))
seen, PLANTS = set(), []
for g in GRID:
    k = (g.get('D'), g.get('G'), g.get('tau'), g.get('asym', 1.0))
    if k not in seen:
        seen.add(k); PLANTS.append(g)

FIXED = dict(VMAX=6000.0, KP_POS=2.0, ARRIVE=80, EXIT=250)


def cost(params):
    worst = -1.0
    for kw in PLANTS:
        r = report("", run(Ctrl(TARGET, **params), Plant(**kw)), TARGET, True)
        peak_over = max(0.0, r['vpeak'] - 7000.0)
        pp_over = max(0.0, r['cruise_pp'] - 1200.0)
        rev_over = max(0.0, -1200.0 - r['vrev'])
        err_over = max(0.0, abs(r['final_err']) - 130.0)
        settle = r['settle'] if r['settle'] is not None else 20.0
        settle_over = max(0.0, settle - 7.2)
        c = ((peak_over / 1000.0) ** 2 + (pp_over / 1000.0) ** 2 +
             (rev_over / 1000.0) ** 2 + (err_over / 50.0) ** 2 +
             r['flips'] * 0.5 + settle_over ** 2)
        worst = max(worst, c)
    return worst


CANDIDATES = dict(
    KP_VEL=[0.005, 0.010, 0.020, 0.030, 0.040],
    KI_VEL=[0.15, 0.25, 0.40, 0.60, 0.90],
    FF_BEMF=[0.0, 0.005, 0.010],
    FF_STIC=[300.0, 400.0, 500.0, 600.0],
    FF_SMOOTH=[80.0, 150.0, 300.0],
    AMAX=[4000.0, 6000.0, 8000.0, 12000.0],
    DECEL=[6000.0, 9000.0, 14000.0, 20000.0],
    MAX_DUTY=[800, 1000, 1200],
    INT_MAX=[400.0, 600.0, 900.0],
    ARRIVE=[60, 80, 120],
)

best = dict(FIXED)
best.update(KP_VEL=0.02, KI_VEL=0.40, FF_BEMF=0.0, FF_STIC=500.0,
            FF_SMOOTH=150.0, AMAX=8000.0, DECEL=9000.0, MAX_DUTY=1000,
            INT_MAX=600.0, ARRIVE=80)
best_cost = cost(best)
print(f"起点 cost = {best_cost:.3f}")

for it in range(4):
    improved = False
    for k, vals in CANDIDATES.items():
        cur = best[k]
        for v in vals:
            if v == cur:
                continue
            trial = dict(best); trial[k] = v
            c = cost(trial)
            if c < best_cost - 1e-9:
                best_cost, best, cur, improved = c, trial, v, True
    print(f"  第{it+1}轮: cost = {best_cost:.3f}")
    if not improved:
        break

print()
print("=" * 78)
print("精细寻优结果")
print("=" * 78)
for k, v in best.items():
    print(f"    {k:10s} = {v}")
print()

print("=" * 78)
print("逐工况明细 (最优参数)")
print("=" * 78)
print(f"{'D':>5}{'G':>5}{'tau':>7}{'asym':>6} | {'峰值':>8}{'峰谷':>8}{'反向':>8}"
      f"{'翻转':>6}{'误差':>8}{'稳定':>8}")
bad = 0
for kw in PLANTS:
    r = report("", run(Ctrl(TARGET, **best), Plant(**kw)), TARGET, True)
    flag = ""
    if r['vpeak'] > 8000 or r['cruise_pp'] > 1500 or r['vrev'] < -1500 \
            or abs(r['final_err']) > 150:
        flag = " <== 未达标"; bad += 1
    s = f"{r['settle']:.2f}" if r['settle'] is not None else "  --"
    print(f"{kw.get('D',500):5.0f}{kw['G']:5.0f}{kw['tau']*1000:6.0f}m"
          f"{kw.get('asym',1.0):6.1f} | {r['vpeak']:8.0f}{r['cruise_pp']:8.0f}"
          f"{r['vrev']:8.0f}{r['flips']:6d}{r['final_err']:+8.0f}{s:>8}{flag}")
print()
print(f"未达标工况: {bad}/{len(PLANTS)}")
print()
print("固件宏:")
print(f"#define MC_KP_VEL     {best['KP_VEL']:.3f}f")
print(f"#define MC_KI_VEL     {best['KI_VEL']:.3f}f")
print(f"#define MC_FF_BEMF    {best['FF_BEMF']:.4f}f")
print(f"#define MC_FF_STIC    {best['FF_STIC']:.0f}.0f")
print(f"#define MC_FF_SMOOTH  {best['FF_SMOOTH']:.0f}.0f")
print(f"#define MC_AMAX       {best['AMAX']:.0f}.0f")
print(f"#define MC_DECEL      {best['DECEL']:.0f}.0f")
print(f"#define MC_MAX_DUTY   {best['MAX_DUTY']}")
print(f"#define MC_INT_MAX    {best['INT_MAX']:.0f}.0f")
print(f"#define MC_ARRIVE_BAND {best['ARRIVE']}")
