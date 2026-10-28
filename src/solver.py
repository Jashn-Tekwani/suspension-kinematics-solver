"""Hardpoint-based double-wishbone kinematics prototype.

Coordinate convention:
    X = lateral, positive outboard
    Y = longitudinal, positive forwards
    Z = vertical, positive upwards

This is a prototype, not a validated vehicle-design model. In particular,
the code reports upright inclination, steering-axis projections and a
tie-rod-based toe proxy; it does not claim to calculate true wheel camber/toe.
"""
import numpy as np
from scipy.optimize import least_squares

REQUIRED = ("CU_F", "CU_R", "CL_F", "CL_R", "UU", "UL", "TI_I", "TO")


def _point(value, name):
    p = np.asarray(value, dtype=float)
    if p.shape != (3,) or not np.all(np.isfinite(p)):
        raise ValueError(f"{name} must be three finite coordinates [X, Y, Z].")
    return p


def solve_suspension_3d(hardpoints, travel_range=60.0, steps=25):
    """Return kinematic samples from droop to bump.

    The lower upright joint's Z coordinate is prescribed. All link lengths
    are derived from the supplied static hardpoints and treated as rigid.
    """
    if not np.isfinite(travel_range) or travel_range <= 0:
        raise ValueError("travel_range must be a positive finite number.")
    if not isinstance(steps, int) or steps < 5:
        raise ValueError("steps must be an integer of at least 5.")

    missing = [name for name in REQUIRED if name not in hardpoints]
    if missing:
        raise ValueError("Missing hardpoints: " + ", ".join(missing))
    hp = {name: _point(hardpoints[name], name) for name in REQUIRED}

    cu_f, cu_r = hp["CU_F"], hp["CU_R"]
    cl_f, cl_r = hp["CL_F"], hp["CL_R"]
    uu0, ul0 = hp["UU"], hp["UL"]
    ti_i, to0 = hp["TI_I"], hp["TO"]

    lengths = {
        "upper_front": np.linalg.norm(uu0 - cu_f),
        "upper_rear": np.linalg.norm(uu0 - cu_r),
        "lower_front": np.linalg.norm(ul0 - cl_f),
        "lower_rear": np.linalg.norm(ul0 - cl_r),
        "upright": np.linalg.norm(uu0 - ul0),
        "tie_rod": np.linalg.norm(to0 - ti_i),
        "tie_to_lower": np.linalg.norm(to0 - ul0),
        "tie_to_upper": np.linalg.norm(to0 - uu0),
    }
    if any(not np.isfinite(v) or v < 1e-8 for v in lengths.values()):
        raise ValueError("Invalid or zero-length link in the supplied hardpoints.")

    scale = max(lengths.values())

    def residual(q, lower_z):
        ux, uy, uz, lx, ly, tx, ty, tz = q
        uu = np.array([ux, uy, uz])
        ul = np.array([lx, ly, lower_z])
        to = np.array([tx, ty, tz])
        pairs = (
            (uu, cu_f, lengths["upper_front"]),
            (uu, cu_r, lengths["upper_rear"]),
            (ul, cl_f, lengths["lower_front"]),
            (ul, cl_r, lengths["lower_rear"]),
            (uu, ul, lengths["upright"]),
            (to, ti_i, lengths["tie_rod"]),
            (to, ul, lengths["tie_to_lower"]),
            (to, uu, lengths["tie_to_upper"]),
        )
        return np.array([
            (np.dot(a - b, a - b) - length * length) / (scale * scale)
            for a, b, length in pairs
        ])

    # Ensure a single zero-travel sample, even when steps is even.
    offsets = np.linspace(-travel_range, travel_range, steps)
    offsets[int(np.argmin(np.abs(offsets)))] = 0.0
    offsets = np.sort(offsets)
    offsets[int(np.argmin(np.abs(offsets)))] = 0.0
    z_targets = ul0[2] + offsets
    zero_i = int(np.argmin(np.abs(z_targets - ul0[2])))

    q_static = np.array([
        uu0[0], uu0[1], uu0[2],
        ul0[0], ul0[1],
        to0[0], to0[1], to0[2],
    ], dtype=float)
    solved = [None] * len(z_targets)
    solved[zero_i] = q_static

    # Continue outward from static height in both directions to reduce
    # the chance of jumping between alternate geometric solution branches.
    for direction in (1, -1):
        i = zero_i + direction
        while 0 <= i < len(z_targets):
            previous = i - direction
            guess = solved[previous]
            target_z = z_targets[i]
            opt = least_squares(
                lambda q: residual(q, target_z),
                guess,
                max_nfev=3000,
                xtol=1e-12, ftol=1e-12, gtol=1e-12,
            )
            max_resid = float(np.max(np.abs(residual(opt.x, target_z))))
            if not opt.success or max_resid > 1e-7:
                raise ValueError(
                    f"Solver failed at travel {target_z - ul0[2]:.1f} mm "
                    f"(max normalized constraint residual {max_resid:.2e}). "
                    "Try a smaller travel range or review the hardpoints."
                )
            solved[i] = opt.x
            i += direction

    results = []
    for target_z, q in zip(z_targets, solved):
        ux, uy, uz, lx, ly, tx, ty, tz = q
        uu = np.array([ux, uy, uz])
        ul = np.array([lx, ly, target_z])
        to = np.array([tx, ty, tz])

        # Steering-axis line from lower to upper upright joint.
        axis = uu - ul
        axis_len = np.linalg.norm(axis)
        if axis_len < 1e-10:
            raise ValueError("Upper and lower upright joints coincide.")
        axis = axis / axis_len

        # These are plane-projection angles of the upright/steering-axis line,
        # not wheel camber and not necessarily the steering geometry's true KPI.
        upright_front_angle = float(np.degrees(np.arctan2(axis[0], axis[2])))
        steering_axis_side_angle = float(np.degrees(np.arctan2(axis[1], axis[2])))

        # Proxy only: projected tie-rod direction relative to vehicle-forward +Y.
        tie_xy = to[:2] - ti_i[:2]
        toe_proxy = float(np.degrees(np.arctan2(tie_xy[0], tie_xy[1])))

        results.append({
            "travel": float(target_z - ul0[2]),
            "upright_front_angle": upright_front_angle,
            "toe_proxy": toe_proxy,
            "kpi_projection": upright_front_angle,
            "caster_projection": steering_axis_side_angle,
            "UU": uu,
            "UL": ul,
            "TO": to,
            "max_constraint_residual": float(np.max(np.abs(residual(q, target_z)))),
        })
    return results
