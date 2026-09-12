"""
离线仿真: 复现并验证 XZ_RCtest 右电机"低速旋转一圈"位置控制。

电机模型 (近空载 JGB37-520 + 18:1 减速箱):
    duty → 稳态速度:  vel_ss = G * (|duty| - D) * sign(duty)      一阶惯性, 时间常数 tau
    D   = 静摩擦死区 duty            G = 速度增益 counts/s per duty
    tau = 机电时间常数

    ★ 测速环节 (固件 motor.c Encoder_Update): 10ms 差分 → EMA(α=0.60, τ≈17ms)
      控制器只能看到这个滞后速度, 这是闭环极限环的关键成因, 仿真必须包含。

纯 Python, 无依赖。用法: python tools/sim_control.py
"""
import math

DT = 0.010            # 控制周期 10ms
VEL_EMA_A = 0.60      # 固件 motor.c 的测速 EMA 系数


# ---------------- 电机模型 (被控对象) ----------------
class Plant:
    def __init__(self, D=500.0, G=30.0, tau=0.040, asym=1.0, v_stick=200.0):
        self.D, self.G, self.tau = D, G, tau
        self.asym = asym
        self.v_stick = v_stick
        self.vel = 0.0
        self.pos = 0.0

    def step(self, duty, dt):
        """只推进物理; 测速在 run() 里按固件的方式单独做 (见那里注释)"""
        d = abs(duty)
        if d <= self.D:
            decay = (self.D - d) / self.D * 4000.0
            if abs(self.vel) <= self.v_stick and decay * dt >= abs(self.vel):
                self.vel = 0.0
            else:
                self.vel -= math.copysign(decay * dt, self.vel)
            v_ss = 0.0
        else:
            g = self.G * (self.asym if duty > 0 else 1.0 / self.asym)
            v_ss = math.copysign(g * (d - self.D), duty)
        # 一阶惯性: 用精确指数更新 (dt/tau 可能 >2, 显式欧拉会数值发散)
        self.vel += (v_ss - self.vel) * (1.0 - math.exp(-dt / self.tau))
        self.pos += self.vel * dt
        return self.vel, self.pos


# ---------------- 控制器 (与 motor_control.c 对应) ----------------
class Ctrl:
    def __init__(self, target, VMAX=6000.0, KP_POS=2.0,
                 KP_VEL=0.08, KI_VEL=0.05,
                 FF_BEMF=0.09, FF_STIC=500.0, FF_SMOOTH=None,
                 AMAX=None, DECEL=None, INT_MAX=1500.0, MAX_DUTY=1500,
                 ARRIVE=80, EXIT=250, dt=DT):
        self.target = target
        self.VMAX, self.KP_POS = VMAX, KP_POS
        self.KP_VEL, self.KI_VEL = KP_VEL, KI_VEL
        self.FF_BEMF, self.FF_STIC, self.FF_SMOOTH = FF_BEMF, FF_STIC, FF_SMOOTH
        self.AMAX, self.DECEL = AMAX, DECEL
        self.INT_MAX, self.MAX_DUTY = INT_MAX, MAX_DUTY
        self.ARRIVE, self.EXIT = ARRIVE, EXIT
        self.dt = dt
        self.I = 0.0
        self.vel_ref = 0.0
        self.duty = 0.0
        self.arrived = False

    def update(self, pos, vel):
        err = self.target - pos

        if self.arrived:
            if abs(err) < self.EXIT:
                self.duty = 0.0
                return self.duty
            self.arrived = False
        if abs(err) < self.ARRIVE:
            self.arrived = True
            self.I = 0.0
            self.vel_ref = 0.0
            self.duty = 0.0
            return self.duty

        # --- 位置外环 → 速度参考 ---
        vel_ref = self.KP_POS * err
        if self.DECEL:
            v_dec = math.sqrt(2.0 * self.DECEL * abs(err))
            if abs(vel_ref) > v_dec:
                vel_ref = math.copysign(v_dec, err)
        vel_ref = max(-self.VMAX, min(self.VMAX, vel_ref))

        if self.AMAX:
            dv = self.AMAX * self.dt
            vel_ref = max(self.vel_ref - dv, min(self.vel_ref + dv, vel_ref))
        self.vel_ref = vel_ref

        # --- 前馈 ---
        if self.FF_SMOOTH:
            ff = (self.FF_BEMF * vel_ref
                  + self.FF_STIC * vel_ref / (abs(vel_ref) + self.FF_SMOOTH))
        else:
            ff = (self.FF_BEMF * vel_ref
                  + math.copysign(self.FF_STIC, vel_ref)) if vel_ref else 0.0

        # --- 速度环 PI ---
        vel_err = vel_ref - vel
        self.I += self.KI_VEL * vel_err * self.dt
        self.I = max(-self.INT_MAX, min(self.INT_MAX, self.I))
        duty = ff + self.KP_VEL * vel_err + self.I
        self.duty = max(-self.MAX_DUTY, min(self.MAX_DUTY, duty))
        return self.duty


def run(ctrl, plant, T=8.0):
    """
    ★ 测速必须按固件的行为建模 (motor.c Encoder_Update):
        第 k 拍控制用的速度 = 编码器在 t_k 读到的"上一整个 10ms 区间的差分平均"
        = [t_{k-1}, t_k] 的平均速度, 再经 EMA(α=0.60)。
      该平均值中心比 t_k 落后 5ms —— 这 5ms 是闭环能否稳定的关键。
      (早期版本误用"区间终点的瞬时速度", 少算这 5ms, 因此把危险的高 Ki 判成了稳定。)
    """
    n = int(T / ctrl.dt)
    log = []
    v_meas = 0.0                       # 上一步测得的滤波速度, 供本步控制使用
    for _ in range(n):
        duty = ctrl.update(plant.pos, v_meas)
        v0 = plant.vel
        v1, pos = plant.step(duty, DT)
        v_avg = 0.5 * (v0 + v1)                        # 区间平均 = 编码器差分
        v_q = round(v_avg * DT) / DT                   # 整数 count → 100 counts/s 量化
        v_meas += VEL_EMA_A * (v_q - v_meas)           # EMA 低通
        log.append(dict(t=None, pos=pos, vel=v1, vref=ctrl.vel_ref,
                        duty=duty, target=ctrl.target))
    return log


def report(name, log, target, quiet=False):
    pos = [r['pos'] for r in log]
    vel = [r['vel'] for r in log]
    duty = [r['duty'] for r in log]
    vref = [r['vref'] for r in log]
    cruise = [r['vel'] for r in log if 8000 < (target - r['pos']) < 20000]
    settle = None
    for i in range(len(pos)):
        if all(abs(target - p) < 200 for p in pos[i:]):
            settle = i * DT
            break
    # 振荡次数: 巡航段 |duty| 接近限幅的翻转次数
    flips = 0
    for i in range(1, len(duty)):
        if (duty[i - 1] > 0) != (duty[i] > 0) and abs(pos[i] - target) > 500:
            flips += 1
    res = dict(
        vpeak=max(vel), cruise_pp=(max(cruise) - min(cruise)) if cruise else 0.0,
        vmax=float(max(abs(v) for v in vel)), vrev=min(vel),
        final_err=target - pos[-1], settle=settle,
        duty_peak=float(max(abs(d) for d in duty)), flips=flips,
        vref_peak=float(max(abs(v) for v in vref)),
    )
    if not quiet:
        print(f"  {name}")
        print(f"    启动速度峰值     : {res['vpeak']:9.0f} counts/s")
        print(f"    巡航段速度峰谷   : {res['cruise_pp']:9.0f} counts/s")
        print(f"    全过程最大|速度| : {res['vmax']:9.0f} counts/s")
        print(f"    反向速度峰值     : {res['vrev']:9.0f} counts/s")
        print(f"    duty 峰值        : {res['duty_peak']:9.0f}")
        print(f"    巡航段方向翻转   : {res['flips']:9d} 次")
        print(f"    最终位置误差     : {res['final_err']:+9.0f} counts")
        s = f"{res['settle']:.2f} s" if res['settle'] is not None else "未稳定"
        print(f"    稳定时间(<200)   : {s:>9}")
    return res


# ---------------- 固件当前参数 ----------------
OLD = dict(VMAX=6000.0, KP_POS=2.0, KP_VEL=0.08, KI_VEL=0.05,
           FF_BEMF=0.09, FF_STIC=500.0, FF_SMOOTH=None,
           AMAX=None, DECEL=None, INT_MAX=1500.0, MAX_DUTY=1500)

# ---------------- 新方案参数 ----------------
# ff 坡度 = 1/G, 由"duty 1500 → 30000 counts/s"反推 G≈30
NEW = dict(VMAX=6000.0, KP_POS=2.0, KP_VEL=0.08, KI_VEL=0.15,
           FF_BEMF=0.033, FF_STIC=500.0, FF_SMOOTH=150.0,
           AMAX=15000.0, DECEL=5000.0, INT_MAX=600.0, MAX_DUTY=1000)

# 实测症状: 启动峰值 >30000, 巡航峰谷 ~20000, 到位反向尖峰 ~-20000
TARGET = 32256

if __name__ == "__main__":
    print("=" * 78)
    print("新方案鲁棒性扫描 (D=500 固定, 扫 G / tau / 方向不对称)")
    print("  判定标准: 启动峰值<8000, 巡航峰谷<1500, 无反向尖峰, 最终误差<150")
    print("=" * 78)
    hdr = (f"{'G':>4}{'tau(ms)':>8}{'asym':>6} | "
           f"{'旧:峰值':>9}{'峰谷':>8}{'反向':>8}{'翻转':>6}{'误差':>7} | "
           f"{'新:峰值':>9}{'峰谷':>8}{'反向':>8}{'翻转':>6}{'误差':>7}")
    print(hdr)
    print("-" * len(hdr))
    ok = True
    for G in (15.0, 25.0, 30.0, 45.0, 60.0, 90.0):
        for tau in (0.010, 0.020, 0.040, 0.080):
            for asym in (1.0, 1.3):
                kw = dict(D=500.0, G=G, tau=tau, asym=asym)
                ro = report("", run(Ctrl(TARGET, **OLD), Plant(**kw)), TARGET, True)
                rn = report("", run(Ctrl(TARGET, **NEW), Plant(**kw)), TARGET, True)
                flag = ""
                if (rn['vpeak'] > 8000 or rn['cruise_pp'] > 1500
                        or rn['vrev'] < -500 or abs(rn['final_err']) > 150):
                    flag = "  <== 未达标"
                    ok = False
                print(f"{G:4.0f}{tau*1000:8.0f}{asym:6.1f} | "
                      f"{ro['vpeak']:9.0f}{ro['cruise_pp']:8.0f}{ro['vrev']:8.0f}"
                      f"{ro['flips']:6d}{ro['final_err']:7.0f} | "
                      f"{rn['vpeak']:9.0f}{rn['cruise_pp']:8.0f}{rn['vrev']:8.0f}"
                      f"{rn['flips']:6d}{rn['final_err']:7.0f}{flag}")
    print("-" * len(hdr))
    print("新方案全参数域达标" if ok else "新方案存在未达标工况")
