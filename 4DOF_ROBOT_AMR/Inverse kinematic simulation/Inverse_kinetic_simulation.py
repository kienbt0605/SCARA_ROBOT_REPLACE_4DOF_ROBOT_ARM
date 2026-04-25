"""
=============================================================
  4-DOF Planar Robot Arm – Forward & Inverse Kinematics
  Interactive Simulator with Tkinter + Matplotlib
=============================================================
  • 4 revolute joints (θ1 … θ4) with adjustable sliders
  • Forward Kinematics via DH convention
  • Inverse Kinematics via Jacobian pseudo-inverse (numerical)
  • Click on the canvas to set a target → auto IK solve
  • Real-time end-effector coordinate display
  • Workspace boundary circle
=============================================================
"""

import numpy as np
import tkinter as tk
from tkinter import ttk, font as tkfont
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib.patches as patches
import math

# ──────────────────────────────────────────────────────────
#  Robot Parameters (link lengths in arbitrary units)
# ──────────────────────────────────────────────────────────
LINK_LENGTHS = [150, 120, 80, 50]          # L1, L2, L3, L4
JOINT_LIMITS = [(-180, 180)] * 4           # degrees

# ──────────────────────────────────────────────────────────
#  Color Palette
# ──────────────────────────────────────────────────────────
BG_DARK      = "#1a1a2e"
BG_PANEL     = "#16213e"
BG_CANVAS    = "#0f3460"
ACCENT       = "#e94560"
ACCENT2      = "#0ea5e9"
ACCENT3      = "#22d3ee"
ACCENT4      = "#a78bfa"
TEXT_LIGHT    = "#e2e8f0"
TEXT_DIM      = "#94a3b8"
JOINT_COLORS = ["#e94560", "#0ea5e9", "#22d3ee", "#a78bfa"]
LINK_COLORS  = ["#f87171", "#38bdf8", "#67e8f9", "#c4b5fd"]
GRID_COLOR   = "#1e3a5f"
TARGET_COLOR = "#facc15"


# ══════════════════════════════════════════════════════════
#  Kinematics helpers
# ══════════════════════════════════════════════════════════

def forward_kinematics(thetas_deg, lengths=LINK_LENGTHS):
    """Return list of (x, y) joint positions including base and end-effector."""
    points = [(0.0, 0.0)]
    angle_sum = 0.0
    for th, L in zip(thetas_deg, lengths):
        angle_sum += math.radians(th)
        x_prev, y_prev = points[-1]
        x_new = x_prev + L * math.cos(angle_sum)
        y_new = y_prev + L * math.sin(angle_sum)
        points.append((x_new, y_new))
    return points


def jacobian(thetas_deg, lengths=LINK_LENGTHS):
    """Compute 2×4 Jacobian for planar 4-DOF arm (position only)."""
    n = len(lengths)
    J = np.zeros((2, n))
    for i in range(n):
        angle_sum = sum(math.radians(thetas_deg[k]) for k in range(i + 1))
        # partial derivatives of end-effector x,y w.r.t. θ_i
        for j in range(i, n):
            a = sum(math.radians(thetas_deg[k]) for k in range(j + 1))
            J[0, i] += -lengths[j] * math.sin(a)
            J[1, i] +=  lengths[j] * math.cos(a)
    return J


def inverse_kinematics(target_xy, thetas_init_deg, lengths=LINK_LENGTHS,
                        max_iter=500, tol=1e-2, alpha=0.5, clamp=5.0):
    """Numerical IK using damped-least-squares (Jacobian pseudo-inverse)."""
    thetas = np.array(thetas_init_deg, dtype=float)
    for _ in range(max_iter):
        pts = forward_kinematics(thetas, lengths)
        ee = np.array(pts[-1])
        error = np.array(target_xy) - ee
        if np.linalg.norm(error) < tol:
            break
        J = jacobian(thetas, lengths)
        # Damped least squares
        lam = 0.5
        JT = J.T
        dtheta = JT @ np.linalg.solve(J @ JT + lam**2 * np.eye(2), error)
        dtheta_deg = np.degrees(dtheta) * alpha
        # Clamp step size
        dtheta_deg = np.clip(dtheta_deg, -clamp, clamp)
        thetas += dtheta_deg
        # Wrap angles to [-180, 180]
        thetas = (thetas + 180) % 360 - 180
    return thetas.tolist()


# ══════════════════════════════════════════════════════════
#  Main Application
# ══════════════════════════════════════════════════════════

class RobotArmSimulator:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("4-DOF Robot Arm — Forward & Inverse Kinematics Simulator")
        self.root.configure(bg=BG_DARK)
        self.root.state("zoomed")          # maximize on Windows
        self.root.minsize(1100, 700)

        # Joint angles (degrees)
        self.thetas = [45.0, -30.0, 20.0, -15.0]
        self.target = None                 # (x, y) or None

        # ── Fonts ──
        self.font_title = tkfont.Font(family="Segoe UI", size=16, weight="bold")
        self.font_label = tkfont.Font(family="Segoe UI", size=11)
        self.font_value = tkfont.Font(family="Consolas", size=12, weight="bold")
        self.font_info  = tkfont.Font(family="Consolas", size=10)
        self.font_small = tkfont.Font(family="Segoe UI", size=9)

        self._build_ui()
        self._draw()

    # ──────────────────────────────────────────────────────
    #  UI Construction
    # ──────────────────────────────────────────────────────
    def _build_ui(self):
        # Top title bar
        title_frame = tk.Frame(self.root, bg=BG_DARK, pady=8)
        title_frame.pack(fill="x")
        tk.Label(title_frame, text="🤖  4-DOF Robot Arm Simulator",
                 font=self.font_title, bg=BG_DARK, fg=TEXT_LIGHT).pack()
        tk.Label(title_frame,
                 text="Điều chỉnh thanh trượt hoặc click trên canvas để đặt target (Inverse Kinematics)",
                 font=self.font_small, bg=BG_DARK, fg=TEXT_DIM).pack()

        # Main horizontal container
        main = tk.Frame(self.root, bg=BG_DARK)
        main.pack(fill="both", expand=True, padx=10, pady=(0, 10))

        # ── LEFT: Matplotlib canvas ──
        canvas_frame = tk.Frame(main, bg=BG_CANVAS, bd=0, highlightthickness=2,
                                highlightbackground=ACCENT)
        canvas_frame.pack(side="left", fill="both", expand=True, padx=(0, 10))

        self.fig = Figure(facecolor=BG_CANVAS, dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, master=canvas_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        self.canvas.mpl_connect("button_press_event", self._on_canvas_click)

        # ── RIGHT: Control panel ──
        right = tk.Frame(main, bg=BG_PANEL, width=370, bd=0)
        right.pack(side="right", fill="y")
        right.pack_propagate(False)

        # Padding wrapper
        panel = tk.Frame(right, bg=BG_PANEL, padx=16, pady=12)
        panel.pack(fill="both", expand=True)

        # ── Joint sliders ──
        tk.Label(panel, text="🔧  Joint Angles", font=self.font_title,
                 bg=BG_PANEL, fg=TEXT_LIGHT).pack(anchor="w", pady=(0, 8))

        self.sliders = []
        self.angle_labels = []
        joint_names = ["θ₁  (Base)", "θ₂  (Shoulder)", "θ₃  (Elbow)", "θ₄  (Wrist)"]

        for i in range(4):
            f = tk.Frame(panel, bg=BG_PANEL)
            f.pack(fill="x", pady=4)

            # Color indicator + name
            hdr = tk.Frame(f, bg=BG_PANEL)
            hdr.pack(fill="x")
            dot = tk.Canvas(hdr, width=12, height=12, bg=BG_PANEL, highlightthickness=0)
            dot.pack(side="left", padx=(0, 6))
            dot.create_oval(2, 2, 12, 12, fill=JOINT_COLORS[i], outline="")
            tk.Label(hdr, text=joint_names[i], font=self.font_label,
                     bg=BG_PANEL, fg=TEXT_LIGHT).pack(side="left")

            val_label = tk.Label(hdr, text=f"{self.thetas[i]:+.1f}°",
                                 font=self.font_value, bg=BG_PANEL,
                                 fg=JOINT_COLORS[i])
            val_label.pack(side="right")
            self.angle_labels.append(val_label)

            # Slider
            slider = tk.Scale(f, from_=-180, to=180, orient="horizontal",
                              resolution=0.5, length=310, sliderlength=18,
                              bg=BG_PANEL, fg=TEXT_LIGHT, troughcolor=BG_DARK,
                              activebackground=JOINT_COLORS[i],
                              highlightthickness=0, bd=0,
                              showvalue=False,
                              command=lambda val, idx=i: self._on_slider(idx, val))
            slider.set(self.thetas[i])
            slider.pack(fill="x", pady=(2, 0))
            self.sliders.append(slider)

            # Min / Max labels
            mm = tk.Frame(f, bg=BG_PANEL)
            mm.pack(fill="x")
            tk.Label(mm, text="-180°", font=self.font_small, bg=BG_PANEL,
                     fg=TEXT_DIM).pack(side="left")
            tk.Label(mm, text=f"L{i+1}={LINK_LENGTHS[i]}", font=self.font_small,
                     bg=BG_PANEL, fg=TEXT_DIM).pack(side="left", expand=True)
            tk.Label(mm, text="180°", font=self.font_small, bg=BG_PANEL,
                     fg=TEXT_DIM).pack(side="right")

        # Separator
        ttk.Separator(panel, orient="horizontal").pack(fill="x", pady=12)

        # ── End-effector info ──
        tk.Label(panel, text="📍  End-Effector", font=self.font_title,
                 bg=BG_PANEL, fg=TEXT_LIGHT).pack(anchor="w", pady=(0, 6))

        self.info_frame = tk.Frame(panel, bg=BG_DARK, bd=1, relief="solid",
                                   padx=12, pady=8)
        self.info_frame.pack(fill="x")

        self.lbl_ee_x = tk.Label(self.info_frame, text="X: ---", font=self.font_info,
                                  bg=BG_DARK, fg=ACCENT3, anchor="w")
        self.lbl_ee_x.pack(fill="x")
        self.lbl_ee_y = tk.Label(self.info_frame, text="Y: ---", font=self.font_info,
                                  bg=BG_DARK, fg=ACCENT3, anchor="w")
        self.lbl_ee_y.pack(fill="x")
        self.lbl_ee_dist = tk.Label(self.info_frame, text="Dist: ---", font=self.font_info,
                                     bg=BG_DARK, fg=TEXT_DIM, anchor="w")
        self.lbl_ee_dist.pack(fill="x")

        # ── Target info ──
        ttk.Separator(panel, orient="horizontal").pack(fill="x", pady=12)
        tk.Label(panel, text="🎯  Target (click canvas)", font=self.font_title,
                 bg=BG_PANEL, fg=TEXT_LIGHT).pack(anchor="w", pady=(0, 6))

        self.target_frame = tk.Frame(panel, bg=BG_DARK, bd=1, relief="solid",
                                      padx=12, pady=8)
        self.target_frame.pack(fill="x")

        self.lbl_tgt = tk.Label(self.target_frame, text="No target set",
                                font=self.font_info, bg=BG_DARK, fg=TARGET_COLOR)
        self.lbl_tgt.pack(fill="x")
        self.lbl_ik_err = tk.Label(self.target_frame, text="",
                                    font=self.font_info, bg=BG_DARK, fg=ACCENT)
        self.lbl_ik_err.pack(fill="x")

        # Buttons
        btn_frame = tk.Frame(panel, bg=BG_PANEL)
        btn_frame.pack(fill="x", pady=(12, 0))

        self.btn_reset = tk.Button(btn_frame, text="↺  Reset Joints",
                                    font=self.font_label, bg=ACCENT, fg="white",
                                    activebackground="#c0392b", bd=0, padx=12, pady=6,
                                    cursor="hand2", command=self._reset_joints)
        self.btn_reset.pack(side="left", expand=True, fill="x", padx=(0, 4))

        self.btn_clear = tk.Button(btn_frame, text="✕  Clear Target",
                                    font=self.font_label, bg="#334155", fg=TEXT_LIGHT,
                                    activebackground="#475569", bd=0, padx=12, pady=6,
                                    cursor="hand2", command=self._clear_target)
        self.btn_clear.pack(side="right", expand=True, fill="x", padx=(4, 0))

        # ── DH Table ──
        ttk.Separator(panel, orient="horizontal").pack(fill="x", pady=12)
        tk.Label(panel, text="📋  DH Parameters", font=self.font_label,
                 bg=BG_PANEL, fg=TEXT_LIGHT).pack(anchor="w", pady=(0, 4))

        dh_frame = tk.Frame(panel, bg=BG_DARK, bd=1, relief="solid", padx=6, pady=4)
        dh_frame.pack(fill="x")

        headers = ["Link", "θ (var)", "d", "a", "α"]
        for c, h in enumerate(headers):
            tk.Label(dh_frame, text=h, font=self.font_small, bg=BG_DARK,
                     fg=TEXT_DIM, width=7, anchor="center").grid(row=0, column=c)

        for i in range(4):
            vals = [f"{i+1}", f"θ{i+1}", "0", f"{LINK_LENGTHS[i]}", "0°"]
            for c, v in enumerate(vals):
                fg = JOINT_COLORS[i] if c == 0 else TEXT_LIGHT
                tk.Label(dh_frame, text=v, font=self.font_small, bg=BG_DARK,
                         fg=fg, width=7, anchor="center").grid(row=i+1, column=c)

    # ──────────────────────────────────────────────────────
    #  Drawing
    # ──────────────────────────────────────────────────────
    def _draw(self):
        ax = self.ax
        ax.clear()

        total_reach = sum(LINK_LENGTHS)
        margin = total_reach * 0.15
        lim = total_reach + margin

        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim * 0.4, lim)
        ax.set_aspect("equal")
        ax.set_facecolor(BG_CANVAS)
        ax.grid(True, color=GRID_COLOR, linewidth=0.5, alpha=0.5)
        ax.tick_params(colors=TEXT_DIM, labelsize=8)
        for spine in ax.spines.values():
            spine.set_color(GRID_COLOR)

        # Workspace circle
        ws_circle = patches.Circle((0, 0), total_reach, fill=False,
                                    edgecolor=TEXT_DIM, linewidth=1,
                                    linestyle="--", alpha=0.3)
        ax.add_patch(ws_circle)

        # Ground hatch
        hatch_y = -8
        for xh in range(-int(lim), int(lim), 25):
            ax.plot([xh, xh - 15], [hatch_y, hatch_y - 15],
                    color=TEXT_DIM, linewidth=0.8, alpha=0.5)
        ax.plot([-lim, lim], [hatch_y, hatch_y], color=TEXT_DIM, linewidth=1.5)

        # Forward kinematics
        points = forward_kinematics(self.thetas)

        # Draw links
        link_widths = [7, 5.5, 4, 2.5]
        for i in range(4):
            x0, y0 = points[i]
            x1, y1 = points[i + 1]
            ax.plot([x0, x1], [y0, y1], color=LINK_COLORS[i],
                    linewidth=link_widths[i], solid_capstyle="round", zorder=3)
            # Shadow
            ax.plot([x0, x1], [y0, y1], color="black",
                    linewidth=link_widths[i] + 3, solid_capstyle="round",
                    alpha=0.15, zorder=2)

        # Draw joints
        joint_sizes = [14, 12, 10, 8]
        for i in range(5):
            x, y = points[i]
            if i < 4:
                # Joint circle
                ax.plot(x, y, 'o', color=JOINT_COLORS[i],
                        markersize=joint_sizes[i], zorder=5,
                        markeredgecolor="white", markeredgewidth=1.5)
                # Label
                ax.annotate(f"J{i+1}", (x, y), textcoords="offset points",
                            xytext=(10, 10), fontsize=8, color=JOINT_COLORS[i],
                            fontweight="bold",
                            bbox=dict(boxstyle="round,pad=0.2", fc=BG_CANVAS,
                                      ec=JOINT_COLORS[i], alpha=0.85))
            else:
                # End-effector
                ax.plot(x, y, 's', color=ACCENT3, markersize=10, zorder=5,
                        markeredgecolor="white", markeredgewidth=2)
                ax.annotate("EE", (x, y), textcoords="offset points",
                            xytext=(12, -12), fontsize=9, color=ACCENT3,
                            fontweight="bold",
                            bbox=dict(boxstyle="round,pad=0.2", fc=BG_CANVAS,
                                      ec=ACCENT3, alpha=0.85))

        # Base pedestal
        base_w, base_h = 40, 10
        rect = patches.FancyBboxPatch((-base_w/2, -base_h - 8), base_w, base_h,
                                       boxstyle="round,pad=2",
                                       facecolor="#334155", edgecolor=TEXT_DIM,
                                       linewidth=1.5, zorder=4)
        ax.add_patch(rect)

        # Target
        if self.target is not None:
            tx, ty = self.target
            ax.plot(tx, ty, '*', color=TARGET_COLOR, markersize=18, zorder=6,
                    markeredgecolor="white", markeredgewidth=0.8)
            ax.annotate("TARGET", (tx, ty), textcoords="offset points",
                        xytext=(12, 12), fontsize=8, color=TARGET_COLOR,
                        fontweight="bold",
                        bbox=dict(boxstyle="round,pad=0.2", fc=BG_CANVAS,
                                  ec=TARGET_COLOR, alpha=0.85))
            # Dashed line from EE to target
            ee = points[-1]
            ax.plot([ee[0], tx], [ee[1], ty], '--', color=TARGET_COLOR,
                    linewidth=1, alpha=0.5)

        # Title
        ax.set_title("Robot Arm Visualization", color=TEXT_LIGHT, fontsize=13,
                      fontweight="bold", pad=10)

        self.canvas.draw_idle()

        # Update info labels
        ee = points[-1]
        dist = math.sqrt(ee[0]**2 + ee[1]**2)
        self.lbl_ee_x.config(text=f"X : {ee[0]:>+8.2f}")
        self.lbl_ee_y.config(text=f"Y : {ee[1]:>+8.2f}")
        self.lbl_ee_dist.config(text=f"Dist: {dist:>8.2f}  /  Max: {total_reach}")

        if self.target is not None:
            tx, ty = self.target
            err = math.sqrt((ee[0]-tx)**2 + (ee[1]-ty)**2)
            self.lbl_tgt.config(text=f"Target  X:{tx:>+.1f}  Y:{ty:>+.1f}")
            self.lbl_ik_err.config(text=f"IK Error: {err:.2f} units")
        else:
            self.lbl_tgt.config(text="No target set")
            self.lbl_ik_err.config(text="")

    # ──────────────────────────────────────────────────────
    #  Callbacks
    # ──────────────────────────────────────────────────────
    def _on_slider(self, idx, val):
        self.thetas[idx] = float(val)
        self.angle_labels[idx].config(text=f"{float(val):+.1f}°")
        self._draw()

    def _on_canvas_click(self, event):
        if event.inaxes != self.ax:
            return
        self.target = (event.xdata, event.ydata)

        # Run IK from current pose
        result = inverse_kinematics(self.target, self.thetas)
        self.thetas = result
        # Update sliders
        for i in range(4):
            self.sliders[i].set(self.thetas[i])
            self.angle_labels[i].config(text=f"{self.thetas[i]:+.1f}°")
        self._draw()

    def _reset_joints(self):
        self.thetas = [45.0, -30.0, 20.0, -15.0]
        for i in range(4):
            self.sliders[i].set(self.thetas[i])
            self.angle_labels[i].config(text=f"{self.thetas[i]:+.1f}°")
        self._draw()

    def _clear_target(self):
        self.target = None
        self._draw()


# ══════════════════════════════════════════════════════════
#  Entry point
# ══════════════════════════════════════════════════════════
if __name__ == "__main__":
    root = tk.Tk()
    app = RobotArmSimulator(root)
    root.mainloop()
