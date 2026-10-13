"""
usv_battery_dashboard.py
=========================================================================
Battery-Aware MPC Speed Control for an Autonomous Surface Vehicle (USV)
--- Live Visualization Dashboard ---

Reproduces (in Python) the same mathematical model used in
USV_Battery_Aware_MPC.m:

    m*dv/dt = T - (c1*v + c2*v^2)                      (surge dynamics)
    P = T*v/eta_p + k_loss*T^2 + P_hotel                (power draw)
    SOC(t+dt) = SOC(t) - P*dt/(3600*E_cap)*100          (battery SOC)

Instead of rendering a GIF after the simulation finishes, this version
executes the simulation one outer control interval at a time and updates
a Tkinter + Matplotlib dashboard while the code is running.

Dashboard:
    - Boat icon moving BASE -> DESTINATION -> BASE
    - Live battery gauge and mission mode
    - Live speed chart
    - Live SOC chart
    - MPC and PID traces retained for direct comparison

Produces:
    usv_summary.png   -- static 4-panel comparison figure after both runs

Run:
    python3 usv_battery_dashboard.py

No external files are required. If usv_mpc_results.csv or
usv_pid_results.csv exists, that run is replayed from the exported data;
otherwise the Python model is executed live.
"""



import os
import numpy as np
import matplotlib
try:
    matplotlib.use("TkAgg")  # desktop: live Tkinter dashboard
except ImportError:
    matplotlib.use("Agg")    # headless fallback for testing/importing the model
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import tkinter as tk
from tkinter import ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# =============================================================================
# 1. VEHICLE / BATTERY MODEL PARAMETERS  (mirrors USV_Battery_Aware_MPC.m)
# =============================================================================
class Params:
    m = 300.0        # effective mass [kg]
    c1 = 15.0         # linear drag [N/(m/s)]
    c2 = 25.0         # quadratic drag [N/(m/s)^2]
    eta_p = 0.70      # propulsion efficiency
    k_loss = 0.02     # electrical loss coeff [W/N^2]
    P_hotel = 40.0    # hotel load [W]
    E_cap_Wh = 650.0  # battery usable capacity [Wh] (~ a small 24V/27Ah pack)
    T_max = 400.0     # max thrust [N]
    v_max = 2.2       # max commandable speed [m/s]

p = Params()

D_total = 5000.0     # one-way distance Base -> Destination [m]
dt = 5.0             # outer control / decision time step [s]
dt_sub = 0.5         # inner physics-integration time step [s] (for numerical
                      # stability of the PID loop -- see run_mission)
n_sub = int(round(dt / dt_sub))
N_max = 6000

SOC_reserve_margin = 1.30
SOC_economy_trig = 1.70
SOC_emergency = 8.0
v_eco = 1.0
v_emg = 0.4

MODE_NAMES = {1: "CRUISE", 2: "ECONOMY", 3: "RETURN TO BASE", 4: "EMERGENCY"}
MODE_COLORS = {1: "#2ecc71", 2: "#f1c40f", 3: "#e67e22", 4: "#e74c3c"}


def specific_energy(v):
    """e(v) = P_ss(v)/v  [J/m], steady-state specific energy of travel."""
    v = max(v, 0.05)
    T_ss = p.c1 * v + p.c2 * v ** 2
    P_ss = T_ss * v / p.eta_p + p.k_loss * T_ss ** 2 + p.P_hotel
    return P_ss / v


def mpc_select_speed(v_cur, x, direction, phase, SOC):
    """Constraint-based receding-horizon (MPC) speed selector.

    For candidate cruise speeds (fastest -> slowest, restricted to the
    range allowed by the current battery-aware phase) this:
      1. Forward-simulates N steps ahead to predict energy used & position.
      2. Checks whether the PREDICTED remaining battery would still be
         enough (with margin) to finish the rest of the mission at the
         economical speed v_eco.
      3. Returns the FASTEST candidate that passes this feasibility check
         -- "go as fast as you safely can, no faster."
    Mirrors mpc_select_speed() in USV_Battery_Aware_MPC.m exactly.
    """
    N = 6
    tau_v = 8.0

    if phase == 1:
        v_lo, v_hi = 0.5, p.v_max
    elif phase == 2:
        v_lo, v_hi = 0.4, v_eco + 0.4
    elif phase == 3:
        v_lo, v_hi = 0.3, v_eco
    else:
        v_lo, v_hi = 0.0, v_emg

    candidates = np.linspace(v_hi, v_lo, 10)  # fastest first
    E_have = SOC / 100.0 * p.E_cap_Wh
    e_spec_eco = specific_energy(v_eco)

    v_cmd = v_lo  # fallback: best-effort / most conservative
    for vc in candidates:
        vp, xp, E_used = v_cur, x, 0.0
        for _ in range(N):
            dv = (vc - vp) / tau_v * dt
            vp = max(vp + dv, 0.0)
            Tp = p.m * dv / dt + p.c1 * vp + p.c2 * vp ** 2
            Tp = min(max(Tp, 0.0), p.T_max)
            Pp = Tp * vp / p.eta_p + p.k_loss * Tp ** 2 + p.P_hotel
            E_used += Pp * dt / 3600.0
            xp += direction * vp * dt

        E_after = E_have - E_used
        dist_to_dest_after = max(D_total - xp, 0.0)
        remaining_after = dist_to_dest_after + D_total if direction > 0 else max(xp, 0.0)
        E_need_after = e_spec_eco * remaining_after / 3600.0

        if E_after >= E_need_after * SOC_reserve_margin:
            v_cmd = vc
            break

    return v_cmd

# =============================================================================
# 2. STEP-BY-STEP SIMULATION
# =============================================================================
class MissionStepper:
    """Run one outer control interval at a time.

    This is the same mission model as run_mission(), but exposes one simulation
    step at a time so the Tkinter dashboard can render the result while the
    code is executing instead of first generating a GIF.
    """

    def __init__(self, ctrl_type="MPC", v_ref_pid=2.0):
        self.ctrl_type = ctrl_type
        self.v_ref_pid = v_ref_pid

        self.t = 0.0
        self.x = 0.0
        self.v = 0.0
        self.SOC = 100.0
        self.phase = 1
        self.direction = 1

        self.Kp, self.Ki, self.Kd = 90.0, 12.0, 6.0
        self.int_e, self.prev_e = 0.0, 0.0
        self.last_v_ref = 0.0
        self.alpha_smooth = 0.15

        self.k = 0
        self.mission_done = False
        self.log = {k: [] for k in ["t", "x", "v", "SOC", "P", "T", "mode"]}

    def step(self):
        """Advance the mission by one outer control interval and return a state."""
        if self.mission_done or self.k >= N_max:
            self.mission_done = True
            return None

        self.k += 1

        # ---------------- outer loop: supervisor + target speed ----------
        if self.ctrl_type == "MPC":
            dist_to_dest = max(D_total - self.x, 0.0)
            remaining_round_trip = dist_to_dest + D_total if self.direction > 0 else self.x

            e_spec = specific_energy(v_eco)
            E_need_Wh = e_spec * remaining_round_trip / 3600.0
            E_have_Wh = self.SOC / 100.0 * p.E_cap_Wh

            if self.SOC <= SOC_emergency:
                self.phase = 4
            elif E_have_Wh < E_need_Wh * SOC_reserve_margin:
                self.phase = 3
                self.direction = -1
            elif E_have_Wh < E_need_Wh * SOC_economy_trig:
                self.phase = 2
            else:
                self.phase = 1

            if self.direction > 0 and self.x >= D_total:
                self.direction = -1
            if self.direction < 0 and self.x <= 0:
                self.x = 0.0
                self.mission_done = True

            if not self.mission_done:
                v_ref_raw = mpc_select_speed(
                    self.v, self.x, self.direction, self.phase, self.SOC
                )
                self.v_ref = (
                    self.alpha_smooth * v_ref_raw
                    + (1 - self.alpha_smooth) * self.last_v_ref
                )
                self.last_v_ref = self.v_ref
            else:
                self.v_ref = 0.0

        else:  # conventional PID -- no battery awareness
            self.phase = 1
            if self.x >= D_total:
                self.direction = -1
            if self.direction < 0 and self.x <= 0:
                self.x = 0.0
                self.mission_done = True
            self.v_ref = self.v_ref_pid

        # ---------------- inner loop: physics + low-level control --------
        P = p.P_hotel
        for _ in range(n_sub):
            if self.mission_done:
                break

            if self.ctrl_type == "MPC":
                tau_v = 8.0
                dv = (self.v_ref - self.v) / tau_v * dt_sub
                v_new = max(self.v + dv, 0.0)
                T_cmd = p.m * dv / dt_sub + p.c1 * self.v + p.c2 * self.v ** 2
                T_cmd = min(max(T_cmd, 0.0), p.T_max)
            else:
                e = self.v_ref - self.v
                self.int_e += e * dt_sub
                der_e = (e - self.prev_e) / dt_sub
                T_cmd = self.Kp * e + self.Ki * self.int_e + self.Kd * der_e
                T_cmd = min(max(T_cmd, 0.0), p.T_max)
                self.prev_e = e
                dv = (
                    T_cmd - (p.c1 * self.v + p.c2 * self.v ** 2)
                ) / p.m * dt_sub
                v_new = max(self.v + dv, 0.0)

            P = (
                T_cmd * self.v / p.eta_p
                + p.k_loss * T_cmd ** 2
                + p.P_hotel
            )
            self.SOC = max(
                self.SOC
                - (P * dt_sub) / (3600.0 * p.E_cap_Wh) * 100.0,
                0.0,
            )

            if self.ctrl_type == "PID" and self.SOC <= 0:
                v_new = 0.0
                self.mission_done = True

            self.x = self.x + self.direction * self.v * dt_sub
            self.x = min(max(self.x, 0.0), D_total * 1.001)
            self.t += dt_sub
            self.v = v_new

        self.log["t"].append(self.t)
        self.log["x"].append(self.x)
        self.log["v"].append(self.v)
        self.log["SOC"].append(self.SOC)
        self.log["P"].append(P)
        self.log["T"].append(0.0)
        self.log["mode"].append(self.phase)

        return {
            "t": self.t,
            "x": self.x,
            "v": self.v,
            "SOC": self.SOC,
            "P": P,
            "mode": self.phase,
            "v_ref": self.v_ref,
            "direction": self.direction,
        }

    def result(self):
        return {kk: np.array(vv) for kk, vv in self.log.items()}


def run_mission(ctrl_type="MPC", v_ref_pid=2.0):
    """Run a complete mission immediately, mainly for scripting/reuse."""
    sim = MissionStepper(ctrl_type, v_ref_pid=v_ref_pid)
    while not sim.mission_done:
        if sim.step() is None:
            break
    return sim.result()


# =============================================================================
# 3. LOAD OPTIONAL MATLAB RESULTS + REPORT SUMMARY
# =============================================================================
def try_load_csv(path):
    """Load a previously exported MATLAB/Python result CSV when available."""
    if not os.path.exists(path):
        return None

    try:
        import csv

        data = {k: [] for k in ["t", "x", "v", "SOC", "P", "T", "mode_id"]}
        with open(path, newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                for k in data:
                    data[k].append(float(row[k]))

        if not data["t"]:
            return None

        data["mode"] = data.pop("mode_id")
        return {k: np.array(v) for k, v in data.items()}
    except Exception:
        return None


def print_stats(label, run):
    energy = np.trapezoid(run["P"], run["t"]) / 3600.0
    print(
        f"[{label}] mission time = {run['t'][-1]/60:.1f} min, "
        f"final SOC = {run['SOC'][-1]:.1f}%, "
        f"energy used = {energy:.1f} Wh"
    )


def make_summary_figure(mpc, pid):
    """Create the same 4-panel report figure once both runs are complete."""
    fig, axs = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle(
        "Battery-Aware MPC vs Conventional PID -- USV Speed Control",
        fontsize=14,
        fontweight="bold",
    )

    ax = axs[0, 0]
    ax.plot(mpc["t"]/60, mpc["SOC"], color="#2ecc71", lw=2,
            label="Battery-Aware MPC")
    ax.plot(pid["t"]/60, pid["SOC"], color="#e74c3c", lw=2, ls="--",
            label="Conventional PID")
    ax.axhline(SOC_emergency, color="k", ls=":", lw=1,
               label="Emergency threshold")
    ax.set_xlabel("Time [min]")
    ax.set_ylabel("Battery SOC [%]")
    ax.set_title("Battery Percentage Remaining")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    ax.set_ylim(0, 105)

    ax = axs[0, 1]
    ax.plot(mpc["t"]/60, mpc["v"], color="#2ecc71", lw=1.8,
            label="MPC (adaptive)")
    ax.plot(pid["t"]/60, pid["v"], color="#e74c3c", lw=1.8, ls="--",
            label="PID (fixed ref)")
    ax.set_xlabel("Time [min]")
    ax.set_ylabel("Speed [m/s]")
    ax.set_title("Speed Profile")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    ax = axs[1, 0]
    ax.plot(mpc["t"]/60, mpc["x"], color="#2ecc71", lw=1.8, label="MPC")
    ax.plot(pid["t"]/60, pid["x"], color="#e74c3c", lw=1.8, ls="--", label="PID")
    ax.axhline(D_total, color="k", ls=":", lw=1)
    ax.axhline(0, color="k", ls=":", lw=1)
    ax.set_xlabel("Time [min]")
    ax.set_ylabel("Distance from base [m]")
    ax.set_title("Mission Track: Base -> Destination -> Base")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    ax = axs[1, 1]
    E_mpc_cum = np.concatenate(
        ([0], np.cumsum((mpc["P"][1:] + mpc["P"][:-1]) / 2 * dt))
    ) / 3600
    E_pid_cum = np.concatenate(
        ([0], np.cumsum((pid["P"][1:] + pid["P"][:-1]) / 2 * dt))
    ) / 3600
    ax.plot(mpc["t"]/60, E_mpc_cum, color="#2ecc71", lw=2, label="MPC")
    ax.plot(pid["t"]/60, E_pid_cum, color="#e74c3c", lw=2, ls="--", label="PID")
    ax.set_xlabel("Time [min]")
    ax.set_ylabel("Cumulative energy [Wh]")
    ax.set_title("Battery Energy Consumption")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig("usv_summary.png", dpi=160)
    plt.close(fig)
    print("Saved usv_summary.png")


# =============================================================================
# 4. LIVE TKINTER DASHBOARD
# =============================================================================
def draw_boat(ax, x_pos, y=0.5, color="#2c3e50", scale=1.0):
    """Draw a simple boat glyph and return only the patches it created."""
    hull = np.array([
        [-0.5, 0.0], [0.5, 0.0], [0.38, 0.22], [-0.38, 0.22], [-0.5, 0.0]
    ]) * scale
    cabin = np.array([
        [-0.15, 0.22], [0.15, 0.22], [0.10, 0.42], [-0.10, 0.42], [-0.15, 0.22]
    ]) * scale

    hull[:, 0] += x_pos
    hull[:, 1] = hull[:, 1] * 0.35 + y
    cabin[:, 0] += x_pos
    cabin[:, 1] = cabin[:, 1] * 0.35 + y

    hull_patch, = ax.fill(hull[:, 0], hull[:, 1], color=color, zorder=5)
    cabin_patch, = ax.fill(
        cabin[:, 0], cabin[:, 1],
        color="#ecf0f1", zorder=6, edgecolor=color
    )
    return [hull_patch, cabin_patch]


class LiveDashboard:
    """Desktop dashboard that renders each simulation step as it executes."""

    def __init__(self, root):
        self.root = root
        self.root.title("USV Battery-Aware Speed Control — Live Simulation")
        self.root.geometry("1280x900")
        self.root.minsize(1050, 760)

        self.running = False
        self.paused = False
        self.auto_run_both = True
        self.current_name = None
        self.current_stepper = None
        self.current_playback = None
        self.completed = {}
        self.delay_ms = tk.IntVar(value=30)
        self.status_var = tk.StringVar(value="Ready — simulation will start automatically.")

        self._build_ui()
        self._reset_visuals()
        self.root.after(500, self.start_both)

    def _build_ui(self):
        top = ttk.Frame(self.root, padding=8)
        top.pack(fill="x")

        ttk.Label(
            top,
            text="USV BATTERY-AWARE CONTROL — LIVE EXECUTION",
            font=("TkDefaultFont", 16, "bold"),
        ).pack(side="left")

        controls = ttk.Frame(self.root, padding=(8, 0, 8, 6))
        controls.pack(fill="x")

        self.start_both_btn = ttk.Button(
            controls, text="Run Both", command=self.start_both
        )
        self.start_both_btn.pack(side="left", padx=3)

        self.mpc_btn = ttk.Button(
            controls, text="Start MPC", command=lambda: self.start_controller("MPC")
        )
        self.mpc_btn.pack(side="left", padx=3)

        self.pid_btn = ttk.Button(
            controls, text="Start PID", command=lambda: self.start_controller("PID")
        )
        self.pid_btn.pack(side="left", padx=3)

        self.pause_btn = ttk.Button(
            controls, text="Pause", command=self.toggle_pause, state="disabled"
        )
        self.pause_btn.pack(side="left", padx=3)

        ttk.Label(controls, text="Display delay:").pack(side="left", padx=(16, 4))
        ttk.Scale(
            controls,
            from_=10,
            to=150,
            variable=self.delay_ms,
            orient="horizontal",
            length=140,
        ).pack(side="left")
        ttk.Label(controls, textvariable=self.delay_ms, width=4).pack(side="left")
        ttk.Label(controls, text="ms/step").pack(side="left", padx=(0, 10))

        ttk.Label(
            self.root, textvariable=self.status_var, padding=(8, 2)
        ).pack(fill="x")

        self.fig = plt.Figure(figsize=(12, 7.3), dpi=100)
        gs = self.fig.add_gridspec(
            4, 2, height_ratios=[1.1, 2.2, 1.0, 1.0], hspace=0.55, wspace=0.3
        )
        self.ax_batt = self.fig.add_subplot(gs[0, :])
        self.ax_water = self.fig.add_subplot(gs[1, :])
        self.ax_speed = self.fig.add_subplot(gs[2, :])
        self.ax_soc = self.fig.add_subplot(gs[3, :])

        self.fig.suptitle(
            "USV Mission Dashboard — Live Results", fontsize=14, fontweight="bold"
        )

        self._setup_dashboard_axes()

        canvas_frame = ttk.Frame(self.root, padding=(8, 0, 8, 8))
        canvas_frame.pack(fill="both", expand=True)
        self.canvas = FigureCanvasTkAgg(self.fig, master=canvas_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    def _setup_dashboard_axes(self):
        self.ax_water.set_xlim(-200, D_total + 200)
        self.ax_water.set_ylim(0, 1)
        self.ax_water.axvspan(-200, D_total + 200, color="#aed6f1", zorder=0)
        for wy in np.linspace(0.1, 0.9, 6):
            self.ax_water.axhline(wy, color="white", lw=0.6, alpha=0.4, zorder=1)
        self.ax_water.scatter([0], [0.5], marker="s", color="#7f8c8d", s=120, zorder=4)
        self.ax_water.text(
            0, 0.75, "BASE", ha="center", fontsize=9, fontweight="bold"
        )
        self.ax_water.scatter(
            [D_total], [0.5], marker="*", color="#f39c12", s=250, zorder=4
        )
        self.ax_water.text(
            D_total, 0.75, "DESTINATION", ha="center", fontsize=9, fontweight="bold"
        )
        self.ax_water.set_yticks([])
        self.ax_water.set_xlabel("Distance along mission track [m]")

        self.mode_text = self.ax_water.text(
            0.02,
            0.05,
            "",
            transform=self.ax_water.transAxes,
            fontsize=11,
            fontweight="bold",
            bbox=dict(boxstyle="round", fc="white", ec="gray"),
        )

        self.ax_batt.set_xlim(0, 1)
        self.ax_batt.set_ylim(0, 1)
        self.ax_batt.axis("off")
        self.batt_outline = patches.FancyBboxPatch(
            (0.30, 0.25),
            0.42,
            0.5,
            boxstyle="round,pad=0.01,rounding_size=0.03",
            fill=False,
            lw=3,
            edgecolor="#2c3e50",
        )
        self.batt_tip = patches.Rectangle(
            (0.72, 0.40), 0.02, 0.20, fill=True, color="#2c3e50"
        )
        self.ax_batt.add_patch(self.batt_outline)
        self.ax_batt.add_patch(self.batt_tip)
        self.batt_fill = patches.Rectangle(
            (0.315, 0.265), 0.39, 0.47, color="#2ecc71"
        )
        self.ax_batt.add_patch(self.batt_fill)
        self.batt_label = self.ax_batt.text(
            0.51,
            0.5,
            "100%",
            ha="center",
            va="center",
            fontsize=16,
            fontweight="bold",
            color="#2c3e50",
            zorder=10,
        )
        self.time_label = self.ax_batt.text(
            0.10, 0.5, "t = 0.0 min", ha="left", va="center", fontsize=11
        )
        self.ax_batt.text(
            0.51,
            0.90,
            "BATTERY STATE OF CHARGE",
            ha="center",
            fontsize=10,
            fontweight="bold",
            color="#555",
        )

        self.ax_speed.set_ylabel("Speed [m/s]")
        self.ax_speed.set_ylim(0, p.v_max * 1.1)
        self.ax_speed.grid(alpha=0.3)
        self.speed_line_mpc, = self.ax_speed.plot(
            [], [], color="#2ecc71", lw=1.8, label="MPC"
        )
        self.speed_line_pid, = self.ax_speed.plot(
            [], [], color="#e74c3c", lw=1.8, ls="--", label="PID"
        )
        self.ax_speed.legend(fontsize=8, loc="upper right")

        self.ax_soc.set_ylabel("SOC [%]")
        self.ax_soc.set_xlabel("Time [min]")
        self.ax_soc.set_ylim(0, 105)
        self.ax_soc.grid(alpha=0.3)
        self.soc_line_mpc, = self.ax_soc.plot(
            [], [], color="#2ecc71", lw=1.8, label="MPC"
        )
        self.soc_line_pid, = self.ax_soc.plot(
            [], [], color="#e74c3c", lw=1.8, ls="--", label="PID"
        )
        self.ax_soc.legend(fontsize=8, loc="upper right")

        self.boat_artists = []

    def _reset_visuals(self):
        self.ax_speed.set_xlim(0, 1)
        self.ax_soc.set_xlim(0, 1)
        self.speed_line_mpc.set_data([], [])
        self.speed_line_pid.set_data([], [])
        self.soc_line_mpc.set_data([], [])
        self.soc_line_pid.set_data([], [])
        self.batt_fill.set_width(0.39)
        self.batt_fill.set_color("#2ecc71")
        self.batt_label.set_text("100%")
        self.time_label.set_text("t = 0.0 min")
        self.mode_text.set_text("Mode: READY")
        self.canvas.draw_idle()

    def _clear_boat(self):
        for artist in self.boat_artists:
            try:
                artist.remove()
            except ValueError:
                pass
        self.boat_artists.clear()

    def _playback_generator(self, run):
        for i in range(len(run["t"])):
            yield {
                "t": float(run["t"][i]),
                "x": float(run["x"][i]),
                "v": float(run["v"][i]),
                "SOC": float(run["SOC"][i]),
                "P": float(run["P"][i]),
                "mode": int(run["mode"][i]) if "mode" in run else 1,
            }

    def _start_source(self, name):
        self.current_name = name
        self.paused = False
        self.running = True
        self.pause_btn.configure(text="Pause", state="normal")
        self.start_both_btn.configure(state="disabled")
        self.mpc_btn.configure(state="disabled")
        self.pid_btn.configure(state="disabled")

        # Prefer an existing MATLAB/Python CSV so the displayed numbers can
        # exactly reproduce an exported reference run. Otherwise execute the
        # model one control interval at a time.
        reference = try_load_csv(
            "usv_mpc_results.csv" if name == "MPC" else "usv_pid_results.csv"
        )
        if reference is not None:
            self.current_stepper = None
            self.current_playback = self._playback_generator(reference)
            source_text = "replaying exported reference data"
        else:
            self.current_stepper = MissionStepper(
                "MPC" if name == "MPC" else "PID",
                v_ref_pid=2.0,
            )
            self.current_playback = None
            source_text = "executing the Python simulation live"

        self.current_log = {k: [] for k in ["t", "x", "v", "SOC", "P", "mode"]}
        self.status_var.set(
            f"{name}: {source_text} — watch the boat, battery, speed and SOC update."
        )
        self._clear_boat()
        self._tick()

    def start_controller(self, name):
        if self.running:
            return
        self.completed = {}
        self._reset_visuals()
        self._start_source(name)

    def start_both(self):
        if self.running:
            return
        self.completed = {}
        self.auto_run_both = True
        self._reset_visuals()
        self._start_source("MPC")

    def toggle_pause(self):
        if not self.running:
            return
        self.paused = not self.paused
        self.pause_btn.configure(text="Resume" if self.paused else "Pause")
        if not self.paused:
            self._tick()

    def _next_state(self):
        if self.current_playback is not None:
            try:
                return next(self.current_playback), False
            except StopIteration:
                return None, True

        state = self.current_stepper.step()
        return state, state is None

    def _tick(self):
        if not self.running or self.paused:
            return

        state, done = self._next_state()
        if done:
            self._finish_controller()
            return

        self.current_log["t"].append(state["t"])
        self.current_log["x"].append(state["x"])
        self.current_log["v"].append(state["v"])
        self.current_log["SOC"].append(state["SOC"])
        self.current_log["P"].append(state["P"])
        self.current_log["mode"].append(state["mode"])

        self._render_state(state)
        self.root.after(max(1, int(self.delay_ms.get())), self._tick)

    def _render_state(self, state):
        name = self.current_name
        mode = int(state.get("mode", 1))
        soc = state["SOC"]
        x = state["x"]

        self._clear_boat()
        self.boat_artists = draw_boat(
            self.ax_water,
            x,
            y=0.5,
            color=MODE_COLORS.get(mode, "#2c3e50"),
            scale=180,
        )

        label = "Battery-Aware MPC" if name == "MPC" else "Conventional PID"
        self.mode_text.set_text(
            f"{label} | Mode: {MODE_NAMES.get(mode, 'CRUISE')} | "
            f"Speed: {state['v']:.2f} m/s | "
            f"Distance: {x:.0f} / {D_total:.0f} m"
        )
        self.mode_text.set_bbox(
            dict(
                boxstyle="round",
                fc="white",
                ec=MODE_COLORS.get(mode, "gray"),
                lw=2,
            )
        )

        frac = max(min(soc / 100.0, 1.0), 0.0)
        self.batt_fill.set_width(0.39 * frac)
        if soc > 50:
            self.batt_fill.set_color("#2ecc71")
        elif soc > SOC_emergency:
            self.batt_fill.set_color("#f1c40f")
        else:
            self.batt_fill.set_color("#e74c3c")

        self.batt_label.set_text(f"{soc:.0f}%")
        self.time_label.set_text(f"t = {state['t']/60:.1f} min")

        t = np.asarray(self.current_log["t"]) / 60.0
        v = np.asarray(self.current_log["v"])
        s = np.asarray(self.current_log["SOC"])

        if name == "MPC":
            self.speed_line_mpc.set_data(t, v)
            self.soc_line_mpc.set_data(t, s)
        else:
            self.speed_line_pid.set_data(t, v)
            self.soc_line_pid.set_data(t, s)

        xmax = max(1.0, float(t[-1]) * 1.05)
        self.ax_speed.set_xlim(0, xmax)
        self.ax_soc.set_xlim(0, xmax)

        self.canvas.draw_idle()

    def _finish_controller(self):
        self.running = False
        self.pause_btn.configure(state="disabled")
        self.completed[self.current_name] = {
            k: np.array(v) for k, v in self.current_log.items()
        }
        self.completed[self.current_name]["T"] = np.zeros(
            len(self.completed[self.current_name]["t"])
        )

        run = self.completed[self.current_name]
        label = "Battery-Aware MPC" if self.current_name == "MPC" else "Conventional PID"
        print_stats(label, run)
        self.status_var.set(
            f"{self.current_name} complete — final SOC "
            f"{run['SOC'][-1]:.1f}% | energy "
            f"{np.trapezoid(run['P'], run['t'])/3600:.1f} Wh"
        )

        if self.auto_run_both and self.current_name == "MPC":
            self.root.after(400, lambda: self._start_source("PID"))
            return

        if self.auto_run_both and {"MPC", "PID"} <= set(self.completed):
            make_summary_figure(self.completed["MPC"], self.completed["PID"])
            self.status_var.set(
                "Both simulations complete — usv_summary.png has been generated. "
                "The live dashboard remains open for demonstration."
            )
            self.start_both_btn.configure(state="normal")
            self.mpc_btn.configure(state="normal")
            self.pid_btn.configure(state="normal")
        else:
            self.start_both_btn.configure(state="normal")
            self.mpc_btn.configure(state="normal")
            self.pid_btn.configure(state="normal")

    def close(self):
        self.root.destroy()


# =============================================================================
# 5. MAIN
# =============================================================================
if __name__ == "__main__":
    try:
        root = tk.Tk()
    except tk.TclError as exc:
        raise SystemExit(
            "Could not open the Tkinter desktop window. Run this script on a "
            "normal desktop Python environment with Tk support."
        ) from exc

    app = LiveDashboard(root)
    root.protocol("WM_DELETE_WINDOW", app.close)
    root.mainloop()
