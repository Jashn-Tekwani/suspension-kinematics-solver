"""Tkinter GUI for the suspension kinematics prototype."""
import json
import tkinter as tk
from tkinter import ttk, messagebox

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

from src.solver import solve_suspension_3d

DEFAULT_HARDPOINTS = {
    "CU_F": "[-150.0, 350.0, 250.0]",
    "CU_R": "[150.0, 350.0, 250.0]",
    "CL_F": "[-200.0, 250.0, 100.0]",
    "CL_R": "[200.0, 250.0, 100.0]",
    "UU": "[0.0, 600.0, 230.0]",
    "UL": "[0.0, 650.0, 80.0]",
    "TI_I": "[-100.0, 300.0, 120.0]",
    "TO": "[-100.0, 620.0, 110.0]",
}


class SuspensionApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Double-Wishbone Kinematics Solver — Prototype")
        self.root.geometry("1150x740")
        self.entries = {}
        self.create_layout()
        self.run_solver()

    def create_layout(self):
        left = ttk.Frame(self.root, padding=10)
        left.pack(side=tk.LEFT, fill=tk.Y)
        ttk.Label(
            left, text="Hardpoints [X, Y, Z] (mm)",
            font=("Arial", 11, "bold")
        ).pack(anchor="w", pady=(0, 10))

        for name, default in DEFAULT_HARDPOINTS.items():
            row = ttk.Frame(left)
            row.pack(fill=tk.X, pady=2)
            ttk.Label(row, text=f"{name}:", width=8, anchor="w").pack(side=tk.LEFT)
            entry = ttk.Entry(row, width=25)
            entry.insert(0, default)
            entry.pack(side=tk.RIGHT, expand=True, fill=tk.X)
            self.entries[name] = entry

        row = ttk.Frame(left)
        row.pack(fill=tk.X, pady=(10, 2))
        ttk.Label(row, text="Travel +/- (mm):", width=14, anchor="w").pack(side=tk.LEFT)
        self.travel_entry = ttk.Entry(row, width=10)
        self.travel_entry.insert(0, "60")
        self.travel_entry.pack(side=tk.RIGHT, expand=True, fill=tk.X)

        row = ttk.Frame(left)
        row.pack(fill=tk.X, pady=2)
        ttk.Label(row, text="Sample points:", width=14, anchor="w").pack(side=tk.LEFT)
        self.steps_entry = ttk.Entry(row, width=10)
        self.steps_entry.insert(0, "25")
        self.steps_entry.pack(side=tk.RIGHT, expand=True, fill=tk.X)

        ttk.Button(left, text="Run Solver", command=self.run_solver).pack(fill=tk.X, pady=15)
        self.status = ttk.Label(left, text="Ready", wraplength=225)
        self.status.pack(anchor="w", pady=(0, 10))
        ttk.Label(
            left,
            text=("Important: outputs are geometric projections and a toe proxy. "
                  "They are not validated wheel camber/toe values."),
            wraplength=225,
        ).pack(anchor="w")

        right = ttk.Frame(self.root)
        right.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        self.fig = Figure(figsize=(8, 6), dpi=100)
        self.ax_upright = self.fig.add_subplot(221)
        self.ax_toe = self.fig.add_subplot(222)
        self.ax_kpi = self.fig.add_subplot(223)
        self.ax_caster = self.fig.add_subplot(224)
        self.canvas = FigureCanvasTkAgg(self.fig, master=right)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def parse_inputs(self):
        hp = {}
        for name, entry in self.entries.items():
            try:
                value = json.loads(entry.get().strip())
                point = np.asarray(value, dtype=float)
            except (json.JSONDecodeError, ValueError, TypeError) as exc:
                raise ValueError(f"{name}: enter coordinates as [x, y, z].") from exc
            if point.shape != (3,) or not np.all(np.isfinite(point)):
                raise ValueError(f"{name}: enter exactly three finite numbers.")
            hp[name] = point

        try:
            travel = float(self.travel_entry.get().strip())
            steps = int(self.steps_entry.get().strip())
        except ValueError as exc:
            raise ValueError("Travel must be numeric; sample points must be an integer.") from exc
        if travel <= 0:
            raise ValueError("Travel must be greater than zero.")
        if steps < 5:
            raise ValueError("Sample points must be at least 5.")
        return hp, travel, steps

    def run_solver(self):
        try:
            hp, travel, steps = self.parse_inputs()
            results = solve_suspension_3d(hp, travel_range=travel, steps=steps)
            x = [r["travel"] for r in results]
            plots = [
                (self.ax_upright, [r["upright_front_angle"] for r in results],
                 "Upright front-view angle (deg)", "Upright angle estimate", "crimson"),
                (self.ax_toe, [r["toe_proxy"] for r in results],
                 "Tie-rod direction proxy (deg)", "Toe proxy (not wheel toe)", "navy"),
                (self.ax_kpi, [r["kpi_projection"] for r in results],
                 "Front-view projection (deg)", "KPI projection (not validated)", "forestgreen"),
                (self.ax_caster, [r["caster_projection"] for r in results],
                 "Side-view projection (deg)", "Caster projection (not validated)", "purple"),
            ]
            for ax, y, ylabel, title, color in plots:
                ax.clear()
                ax.plot(x, y, color=color, linewidth=2)
                ax.set_title(title)
                ax.set_xlabel("Lower upright joint vertical travel (mm)")
                ax.set_ylabel(ylabel)
                ax.grid(True, linestyle=":", alpha=0.6)
            self.fig.tight_layout(pad=2.5)
            self.canvas.draw()
            worst = max(r["max_constraint_residual"] for r in results)
            self.status.config(
                text=f"Completed {len(results)} samples.\n"
                     f"Max normalized residual: {worst:.2e}"
            )
        except Exception as exc:
            self.status.config(text="Solver failed. Check inputs and travel range.")
            messagebox.showerror("Solver Error", str(exc))


def main():
    root = tk.Tk()
    SuspensionApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
