# Double-Wishbone Kinematics Solver (Prototype)

## Run
Install Python 3.10+ and run:

```bash
python -m pip install -r requirements.txt
python main.py
```

## Coordinate convention
- X: lateral, positive outboard
- Y: longitudinal, positive forwards
- Z: vertical, positive upwards
- Hardpoint coordinates are in millimetres.

## Current scope and limitations
This is a hardpoint-based educational prototype, not a validated suspension-design tool.

- It solves a simplified set of fixed-distance constraints as the lower upright joint is moved vertically.
- The upper/lower upright points and tie-rod outer point are represented by rigid distances derived from the static hardpoints.
- The output angles are labelled as projections or proxies. In particular, the tie-rod direction is **not** wheel toe, and upright inclination is **not** necessarily wheel camber.
- Correct wheel camber and toe require an explicit wheel centre/wheel-plane or wheel-heading definition. KPI and caster also depend on correctly defined steering-axis geometry and vehicle coordinate conventions.
- Numerical convergence and small residuals do not prove that the selected solution is physically unique or representative of a real car.
- The default hardpoints are illustrative, not measured vehicle data.

Before using results for design decisions, validate against a known suspension model or trusted kinematics software, and add explicit wheel/upright orientation geometry.
