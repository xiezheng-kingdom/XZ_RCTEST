"""
两阶段: (1) 用实测症状反推电机参数  (2) 在参数不确定域上寻优控制器

实测症状 (用户从 VOFA+ 波形读出, 当前固件):
    启动速度峰值  ~30000+ counts/s      (duty 被限幅在 1500)
    巡航段速度峰谷 ~20000  counts/s
    到位反向尖峰  ~-20000 counts/s

用法: python tools/tune_control.py
"""
import sys, os, math, itertools
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sim_control import Plant, Ctrl, run, report, OLD, TARGET

# ==================== 阶段 1: 反推电机参数 ====================
OBS = dict(vpeak=30000.0, cruise_pp=20000.0, vrev=-20000.0)


def fit_plant():
    best = []
    for D in (400.0, 500.0, 600.0):
        for G in range(20, 105, 5):
            for tau in (0.008, 0.010, 0.015, 0.020, 0.030, 0.040):
                p = Plant(D=D, G=float(G), tau=tau)
                r = report("", run(Ctrl(TARGET, **OLD), p), TARGET, True)
                # 相对误差平方和
                e = ((r['vpeak'] - OBS['vpeak']) / 30000.0) ** 2
                e += ((r['cruise_pp'] - OBS['cruise_pp']) / 20000.0) ** 2
                e += ((r['vrev'] - OBS['vrev']) / 20000.0) ** 2
                best.append((e, D, G, tau, r))
    best.sort(key=lambda x: x[0])
    return best


print("=" * 78)
print("阶段1: 反推电机参数 (目标: 峰值30000 / 峰谷20000 / 反向-20000)")
print("=" * 78)
print(f"{'cost':>8}{'D':>6}{'G':>6}{'tau':>7} | {'峰值':>8}{'峰谷':>8}{'反向':>8}{'翻转':>6}")
fits = fit_plant()
for e, D, G, tau, r in fits[:8]:
    print(f"{e:8.3f}{D:6.0f}{G:6.0f}{tau*1000:6.0f}m | "
          f"{r['vpeak']:8.0f}{r['cruise_pp']:8.0f}{r['vrev']:8.0f}{r['flips']:6d}")

# ==================== 阶段 2: 控制器寻优 ====================
# 不确定域: 覆盖 top-10 拟合 + 更宽的外推, 保证鲁棒
GRID = []
for e, D, G, tau, r in fits[:10]:
    GRID.append(dict(D=D, G=G, tau=tau))
# 外推: 增益/时间常数的极值, 方向不对称
for G in (20.0, 45.0, 90.0, 120.0):
    for tau in (0.008, 0.020, 0.040):
        GRID.append(dict(D=500.0, G=G, tau=tau))
        GRID.append(dict(D=500.0, G=G, tau=tau, asym=1.3))
# 去重
seen, PLANTS = set(), []
for g in GRID:
    k = (g.get('D'), g.get('G'), g.get('tau'), g.get('asym', 1.0))
    if k not in seen:
        seen.add(k)
        PLANTS.append(g)

print()
print(f"不确定域: {len(PLANTS)} 个工况")

# 限速/到位死区固定; 搜索其余参数
FIXED = dict(VMAX=6000.0, KP_POS=2.0, ARRIVE=80, EXIT=250)
SEARCH = dict(
    KP_VEL=[0.02, 0.04, 0.06, 0.08],
    KI_VEL=[0.05, 0.10, 0.20, 0.40],
    FF_BEMF=[0.0, 0.010, 0.022, 0.033],
    AMAX=[8000.0, 15000.0, 30000.0],
    DECEL=[3000.0, 5000.0, 9000.0],
    MAX_DUTY=[700, 1000, 1500],
    FF_STIC=[500.0],
    FF_SMOOTH=[150.0],
    INT_MAX=[600.0],
)


def cost_one(params, plants):
    """在所有工况上评估; 返回 (最坏代价, 分项最坏值)"""
    worst = 0.0
    agg = dict(vpeak=0, cruise_pp=0, vrev=0, final_err=0, flips=0, settle=0)
    for kw in plants:
        r = report("", run(Ctrl(TARGET, **params), Plant(**kw)), TARGET, True)
        peak_over = max(0.0, r['vpeak'] - 7000.0)
        pp_over = max(0.0, r['cruise_pp'] - 1200.0)
        rev_over = max(0.0, -1200.0 - r['vrev'])
        err_over = max(0.0, abs(r['final_err']) - 130.0)
        flips = float(r['flips'])
        settle = r['settle'] if r['settle'] is not None else 20.0
        settle_over = max(0.0, settle - 6.5)
        c = (peak_over / 1000.0) ** 2 + (pp_over / 1000.0) ** 2 + \
            (rev_over / 1000.0) ** 2 + (err_over / 50.0) ** 2 + \
            flips * 0.5 + (settle_over / 1.0) ** 2
        if c > worst:
            worst = c
            cur = dict(vpeak=r['vpeak'], cruise_pp=r['cruise_pp'], vrev=r['vrev'],
                       final_err=r['final_err'], flips=r['flips'], settle=settle)
            agg = cur
    return worst, agg


keys = list(SEARCH.keys())
combos = list(itertools.product(*[SEARCH[k] for k in keys]))
print(f"搜索 {len(combos)} 组控制器参数 × {len(PLANTS)} 工况 "
      f"= {len(combos)*len(PLANTS)} 次仿真 ...")

results = []
for n, vals in enumerate(combos):
    params = dict(FIXED)
    params.update(dict(zip(keys, vals)))
    c, agg = cost_one(params, PLANTS)
    results.append((c, dict(zip(keys, vals)), agg))
    if (n + 1) % 500 == 0:
        print(f"  ... {n+1}/{len(combos)}")

results.sort(key=lambda x: x[0])
print()
print("=" * 78)
print("阶段2: 控制器寻优结果 (按最坏工况代价排序)")
print("=" * 78)
for c, p, a in results[:6]:
    print(f"  cost={c:9.3f}  Kp={p['KP_VEL']:.2f} Ki={p['KI_VEL']:.2f} "
          f"ffB={p['FF_BEMF']:.3f} aMax={p['AMAX']:.0f} decel={p['DECEL']:.0f} "
          f"dutyMax={p['MAX_DUTY']}")
    print(f"      最坏工况: 峰值{a['vpeak']:.0f} 峰谷{a['cruise_pp']:.0f} "
          f"反向{a['vrev']:.0f} 误差{a['final_err']:+.0f} 翻转{a['flips']} "
          f"稳定{a['settle']:.2f}s")

bestc, bestp, _ = results[0]
BEST = dict(FIXED)
BEST.update(bestp)
print()
print("最优参数:", {k: v for k, v in bestp.items()})
print()
print("=" * 78)
print("最优参数逐工况明细")
print("=" * 78)
print(f"{'D':>5}{'G':>5}{'tau':>7}{'asym':>6} | {'峰值':>8}{'峰谷':>8}{'反向':>8}"
      f"{'翻转':>6}{'误差':>8}{'稳定':>8}")
bad = 0
for kw in PLANTS:
    r = report("", run(Ctrl(TARGET, **BEST), Plant(**kw)), TARGET, True)
    flag = ""
    if r['vpeak'] > 8000 or r['cruise_pp'] > 1500 or r['vrev'] < -1500 \
            or abs(r['final_err']) > 150:
        flag = " <== 未达标"
        bad += 1
    s = f"{r['settle']:.2f}" if r['settle'] is not None else "  --"
    print(f"{kw.get('D',500):5.0f}{kw['G']:5.0f}{kw['tau']*1000:6.0f}m"
          f"{kw.get('asym',1.0):6.1f} | {r['vpeak']:8.0f}{r['cruise_pp']:8.0f}"
          f"{r['vrev']:8.0f}{r['flips']:6d}{r['final_err']:+8.0f}{s:>8}{flag}")
print()
print(f"未达标工况: {bad}/{len(PLANTS)}")
