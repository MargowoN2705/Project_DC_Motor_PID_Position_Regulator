import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
from scipy import signal
import pandas as pd
import glob
import os

FOLDER_PATH = r'C:\Users\barte\log'
PRE_TIME = 0.1
POST_TIME = 1.9
W_SIZE = 5

RPM2RAD = 2 * np.pi / 60
K = 21.483652 * RPM2RAD
t1 = 0.152334
t2 = 0.000331

Kp = 5.0
Ti = 0.2
Td = 0.0
Kp_pos = 2.0

FS = 5000.0
dt = 1.0 / FS
T_SIM = 2.0
N = int(T_SIM / dt)

REF = 2 * 3.14
LIM = 3
NOISE = 0.36 * RPM2RAD

b_iir, a_iir = signal.butter(2, 50.0, fs=FS)

a2 = t1 * t2
a1 = t1 + t2
A_mat = np.array([[0, 1], [-1 / a2, -a1 / a2]])
B_mat = np.array([[0], [K / a2]])
C_mat = np.array([[1, 0]])

current_u = 0.0


def model(x, t):
    return A_mat @ x + B_mat.flatten() * current_u


x_state = np.zeros(2)
th_sim = 0.0
t_arr = np.linspace(0, T_SIM, N)

th_list = []
w_list = []
u_list = []

integ = 0.0
prev_w = 0.0
xb = [0.0, 0.0]
yb = [0.0, 0.0]

for i in range(N):
    w = float(C_mat @ x_state)
    w_raw = w + np.random.normal(0, NOISE)

    w_filt = (b_iir[0] * w_raw + b_iir[1] * xb[0] + b_iir[2] * xb[1] - a_iir[1] * yb[0] - a_iir[2] * yb[1])
    xb = [w_raw, xb[0]]
    yb = [w_filt, yb[0]]

    w_set = Kp_pos * (REF - th_sim)
    e = w_set - w_filt

    prop = Kp * e
    integ_new = integ + (Kp / Ti * dt) * e
    deriv = -(Kp * Td / dt) * (w_filt - prev_w)

    u_val = prop + integ_new + deriv
    u_val = np.clip(u_val, -LIM, LIM)

    if u_val == (prop + integ_new + deriv):
        integ = integ_new

    prev_w = w_filt
    current_u = u_val

    x_state = odeint(model, x_state, [0, dt])[-1]
    th_sim += w * dt

    th_list.append(th_sim)
    w_list.append(w)
    u_list.append(u_val)

files = glob.glob(os.path.join(FOLDER_PATH, '*.csv'))
df_cut = None

if files:
    latest = max(files, key=os.path.getctime)
    try:
        df = pd.read_csv(latest, sep=';', decimal=',', on_bad_lines='skip')
        df['p'] = df['dbg_pos_rad'].rolling(W_SIZE, center=True).median().fillna(df['dbg_pos_rad'])

        if 'dbg_speed_filt' in df.columns:
            df['v'] = df['dbg_speed_filt'].rolling(W_SIZE, center=True).median().fillna(df['dbg_speed_filt'])
        else:
            df['v'] = df['dbg_target_speed']

        d = df['dbg_target_speed'].diff().abs()
        idx = d[d > 0.1].index
        start_idx = idx[0] if len(idx) > 0 else 0

        t0 = df.loc[start_idx, 'time']
        mask = (df['time'] >= t0 - PRE_TIME) & (df['time'] <= t0 + POST_TIME)
        df_cut = df.loc[mask].copy()

        if not df_cut.empty:
            df_cut['t_rel'] = df_cut['time'] - t0
            zero_idx = (df_cut['t_rel']).abs().idxmin()
            df_cut['p'] -= df_cut.loc[zero_idx, 'p']

    except:
        pass

plt.rcParams.update({"font.size": 11})
fig, ax = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

ax[0].plot(t_arr, th_list, 'k--', lw=2, label="Symulacja")
ax[0].axhline(REF, c="r", ls=":", label="Cel")
if df_cut is not None:
    ax[0].plot(df_cut['t_rel'], df_cut['p'], c='tab:orange', lw=2, alpha=0.8, label="Pomiar")
ax[0].set_ylabel("Pozycja [rad]")
ax[0].legend()
ax[0].grid(ls="--", alpha=0.6)

ax[1].plot(t_arr, w_list, 'k--', lw=1.5, label="Symulacja")
if df_cut is not None:
    ax[1].plot(df_cut['t_rel'], df_cut['v'], c='tab:blue', lw=1.5, alpha=0.8, label="Pomiar")
    ax[1].plot(df_cut['t_rel'], df_cut['dbg_target_speed'], c='r', ls=':', alpha=0.5)
ax[1].set_ylabel("Prędkość [rad/s]")
ax[1].legend()
ax[1].grid(ls="--", alpha=0.6)

ax[2].plot(t_arr, u_list, 'k--', lw=1.5, label="Symulacja")
if df_cut is not None:
    ax[2].plot(df_cut['t_rel'], df_cut['dbg_pwm_u'], c='tab:green', lw=1.5, alpha=0.8, label="Pomiar")
ax[2].set_ylabel("Sterowanie [V]")
ax[2].set_xlabel("Czas [s]")
ax[2].legend()
ax[2].grid(ls="--", alpha=0.6)

plt.tight_layout()
plt.show()