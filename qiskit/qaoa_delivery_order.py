"""
QAOA picks the visiting order for a real 4-stop delivery mission.

What it does
- Fixes a 4-stop mission on the sim-3d map (seeded, same free-point rule
  and A* road distances as the simulator — see delivery_model.py).
- Writes delivery ordering as a QUBO over 16 binary variables x[i,p]
  ("stop i is visited p-th"): route distance (base -> first stop, stop ->
  stop, last stop -> base) plus penalties enforcing "each stop once, each
  position once". Unlike the Grover demo, the circuit is built from the
  pairwise distances — no route has to be enumerated or scored up front.
- Converts the QUBO to an Ising Hamiltonian and runs QAOA: p layers of
  cost phase RZ/RZZ gates and RX mixers on 16 qubits, from |+>^16.
- Trains the 2p angles with COBYLA on the CVaR objective (average energy
  of the best 20% of outcomes — Barkoutsos et al. 2020), layer by layer
  with INTERP warm starts (Zhou et al. 2020) plus an identity-layer
  extension of the previous optimum, p = 1..10.
- Rebuilds the trained circuit in Qiskit, checks it matches the training
  simulation (state fidelity), samples it on AerSimulator, and reads off
  the best valid route. Checked against the exact classical optimum.

Honest framing
- Training uses an exact NumPy state-vector simulation of the same circuit
  (the 16-qubit cost layer is diagonal, so it is fast and exact); the
  final circuit is then run in Qiskit Aer. Simulating 16 qubits means
  tracking all 65,536 basis states — that is the price of classical
  simulation, not something the algorithm itself needs.
- QAOA is a heuristic. At this size a classical computer solves the
  problem instantly, and no quantum speed-up is claimed. What is shown is
  the real algorithm doing its job: concentrating probability on valid,
  short routes far beyond random chance, improving with depth.
- The QUBO minimizes route distance. The simulator minimizes mission time
  (distance / speed + recharge time); they agree whenever no recharge is
  needed, which this script checks.

Output: qaoa_delivery_result.json (embedded into sim-3d as QAOA_DELIVERY)
and qaoa_delivery_chart.png.

Run:  qiskit/.venv/bin/python qiskit/qaoa_delivery_order.py   (~5 min)
"""

import json
import math
import os
import random
from itertools import permutations

import numpy as np
from scipy.optimize import minimize
from qiskit import QuantumCircuit, transpile
from qiskit.circuit import ParameterVector
from qiskit.quantum_info import Statevector
from qiskit_aer import AerSimulator

import delivery_model as dm

HERE = os.path.dirname(os.path.abspath(__file__))
SEED = 9            # optimum is not the input order, clear gap to 2nd best, no recharge needed
N = 4               # stops
NQ = N * N          # qubits: x[i,p] -> qubit i*N + p
PENALTY = 1.0       # smallest tested value with margin: A=0.5 made an invalid state the minimum
CVAR_ALPHA = 0.2
P_MAX = 10
RESTARTS = 8
SHOTS = 4096

# ---------------------------------------------------------------------------
# 1. Mission and exact classical answer
# ---------------------------------------------------------------------------
rng = random.Random(SEED)
stops = dm.pick_stops(rng, N)
dist = dm.distance_matrix([dm.BASE, *stops])
orders = list(permutations(range(1, N + 1)))
tours = {o: dm.walk_tour(o, dist) for o in orders}
assert not any(r[2] for r in tours.values()), "a recharge is needed: QUBO distance != mission time"
best_time = min(r[0] for r in tours.values())
optimal_orders = [o for o, r in tours.items() if abs(r[0] - best_time) < 1e-9]
worst_time = max(r[0] for r in tours.values())
print(f"{N} stops, {len(orders)} orders, {NQ} qubits. Classical optimum {best_time:.2f}s: {optimal_orders}")

# ---------------------------------------------------------------------------
# 2. QUBO -> Ising
# ---------------------------------------------------------------------------
D = np.array(dist)
Dn = D / D.max()                                  # normalize so the penalty scale is meaningful
q = lambda i, p: i * N + p                        # stop i (0-based) at position p

h = np.zeros(NQ)
J = {}
const = 0.0


def add_j(a, b, v):
    a, b = min(a, b), max(a, b)
    J[(a, b)] = J.get((a, b), 0.0) + v


for i in range(N):
    h[q(i, 0)] += Dn[0, i + 1]                    # base -> first stop
    h[q(i, N - 1)] += Dn[i + 1, 0]                # last stop -> base
for p in range(N - 1):
    for i in range(N):
        for j in range(N):
            if i != j:
                add_j(q(i, p), q(j, p + 1), Dn[i + 1, j + 1])
# PENALTY * (1 - sum x)^2 for every stop (row) and every position (column)
groups = [[q(i, p) for p in range(N)] for i in range(N)] + [[q(i, p) for i in range(N)] for p in range(N)]
for g in groups:
    const += PENALTY
    for a in g:
        h[a] -= PENALTY
    for x in range(len(g)):
        for y in range(x + 1, len(g)):
            add_j(g[x], g[y], 2 * PENALTY)

# x = (1 - Z) / 2  ->  Ising coefficients (constant dropped: a global phase)
z_coef = -h / 2
zz_coef = {}
for (a, b), v in J.items():
    zz_coef[(a, b)] = v / 4
    z_coef[a] -= v / 4
    z_coef[b] -= v / 4

# energy of every basis state (for the exact simulation and the checks)
DIM = 2 ** NQ
bits = ((np.arange(DIM)[:, None] >> np.arange(NQ)) & 1).astype(float)
E = const + bits @ h
for (a, b), v in J.items():
    E += v * bits[:, a] * bits[:, b]

feasible = {}                                     # basis index -> visiting order (1-based stops)
for perm in permutations(range(N)):
    feasible[sum(1 << q(perm[p], p) for p in range(N))] = tuple(s + 1 for s in perm)
optimal_idx = [z for z, o in feasible.items() if o in optimal_orders]
assert int(np.argmin(E)) in optimal_idx, "QUBO minimum is not the optimal route"
feas_E = sorted((E[z], tours[o][0]) for z, o in feasible.items())
assert all(feas_E[k][1] <= feas_E[k + 1][1] + 1e-9 for k in range(len(feas_E) - 1)), "QUBO energy order != time order"
print(f"QUBO: {len(h)} linear + {len(J)} quadratic terms; minimum = optimal route (checked)")

# ---------------------------------------------------------------------------
# 3. Train: exact state-vector simulation, CVaR objective, layer by layer
# ---------------------------------------------------------------------------
psi0 = np.full(DIM, 1 / math.sqrt(DIM), dtype=complex)


def qaoa_state(params):
    p = len(params) // 2
    psi = psi0.copy()
    for layer in range(p):
        psi = psi * np.exp(-1j * params[layer] * E)              # cost layer (diagonal)
        c, s = math.cos(params[p + layer]), math.sin(params[p + layer])
        t = psi.reshape((2,) * NQ)
        for ax in range(NQ):                                     # RX(2*beta) on every qubit
            t = c * t - 1j * s * np.flip(t, axis=ax)
        psi = t.reshape(DIM)
    return psi


order_E = np.argsort(E)
E_sorted = E[order_E]


def cvar(probs):
    ps = probs[order_E]
    cum = np.cumsum(ps)
    k = int(np.searchsorted(cum, CVAR_ALPHA))
    tail = CVAR_ALPHA - (cum[k - 1] if k else 0.0)
    return (np.dot(ps[:k], E_sorted[:k]) + tail * E_sorted[k]) / CVAR_ALPHA


def objective(x):
    return cvar(np.abs(qaoa_state(x)) ** 2)


rs = np.random.default_rng(SEED)
feas_arr, opt_arr = np.array(list(feasible)), np.array(optimal_idx)
history, best = [], None
for p in range(1, P_MAX + 1):
    if p == 1:
        inits = [rs.uniform(0, math.pi / 2, 2) for _ in range(RESTARTS)]
    else:                                                        # INTERP warm start
        g, b = best[:p - 1], best[p - 1:]
        old, new = np.linspace(0, 1, p - 1), np.linspace(0, 1, p)
        gi = np.interp(new, old, g) if p > 2 else np.repeat(g, 2)
        bi = np.interp(new, old, b) if p > 2 else np.repeat(b, 2)
        base = np.concatenate([gi, bi])
        # previous optimum + an identity layer (gamma=beta=0) scores exactly
        # what depth p-1 did, so going deeper can never train to a worse
        # objective. Without it, an unlucky INTERP start at p=3 once dropped
        # P(valid route) from 5% to 0.7% and never recovered.
        extend = np.concatenate([g, [0.0], b, [0.0]])
        inits = [base, extend] + [base + rs.normal(0, 0.1, 2 * p) for _ in range(2)]
    res = min((minimize(objective, x0, method="COBYLA", options={"maxiter": 400}) for x0 in inits),
              key=lambda r: r.fun)
    best = res.x
    probs = np.abs(qaoa_state(best)) ** 2
    history.append({"p": p, "cvar": float(res.fun), "p_feasible": float(probs[feas_arr].sum()), "p_optimal": float(probs[opt_arr].sum())})
    print(f"  p={p:2d}: CVaR={res.fun:.4f}  P(valid route)={history[-1]['p_feasible']:.1%}  P(optimal)={history[-1]['p_optimal']:.1%}")

# ---------------------------------------------------------------------------
# 4. The trained circuit in Qiskit: check it, then sample on AerSimulator
# ---------------------------------------------------------------------------
gamma, beta = ParameterVector("γ", P_MAX), ParameterVector("β", P_MAX)
qc = QuantumCircuit(NQ)
qc.h(range(NQ))
for layer in range(P_MAX):
    for k in range(NQ):
        qc.rz(2 * gamma[layer] * z_coef[k], k)
    for (a, b), v in zz_coef.items():
        qc.rzz(2 * gamma[layer] * v, a, b)
    qc.rx(2 * beta[layer], range(NQ))
bound = qc.assign_parameters(dict(zip(list(gamma) + list(beta), best)))

fidelity = abs(np.vdot(qaoa_state(best), Statevector(bound).data)) ** 2
assert fidelity > 0.999, f"Qiskit circuit does not match the training simulation (fidelity {fidelity})"
print(f"Qiskit circuit matches the training simulation: fidelity {fidelity:.6f}")

sim = AerSimulator(seed_simulator=SEED)
measured = bound.copy()
measured.measure_all()
tqc = transpile(measured, sim, seed_transpiler=SEED)
counts = {int(b, 2): c for b, c in sim.run(tqc, shots=SHOTS).result().get_counts().items()}

valid = {feasible[z]: c for z, c in counts.items() if z in feasible}
valid_shots = sum(valid.values())
optimal_shots = sum(c for o, c in valid.items() if o in optimal_orders)
qaoa_order = min(valid, key=lambda o: tours[o][0])               # best valid route sampled
most_likely = max(valid, key=valid.get)
mean_valid_time = sum(tours[o][0] * c for o, c in valid.items()) / valid_shots
print(f"\n{SHOTS} shots: {valid_shots} valid routes ({valid_shots / SHOTS:.1%}), "
      f"{optimal_shots} optimal ({optimal_shots / SHOTS:.1%})")
print(f"QAOA's route: {list(qaoa_order)} ({tours[qaoa_order][0]:.2f}s), most frequent: {list(most_likely)}")
print(f"Matches classical optimum: {qaoa_order in optimal_orders}  |  "
      f"mean valid route {mean_valid_time:.2f}s vs optimum {best_time:.2f}s vs worst {worst_time:.2f}s")

ops = tqc.count_ops()
result = {
    "seed": SEED,
    "stops": [{"x": round(x, 2), "y": round(y, 2)} for x, y in stops],
    "qubits": NQ, "layers": P_MAX, "cvar_alpha": CVAR_ALPHA, "penalty": PENALTY,
    "qubo_terms": {"linear": int(np.count_nonzero(h)), "quadratic": len(J)},
    "circuit": {"depth": tqc.depth(), "rzz": int(ops.get("rzz", 0)), "rz": int(ops.get("rz", 0)), "rx": int(ops.get("rx", 0))},
    "qaoa_order": list(qaoa_order),
    "qaoa_time_s": round(tours[qaoa_order][0], 2),
    "most_likely_order": list(most_likely),
    "classical_best_time_s": round(best_time, 2),
    "worst_time_s": round(worst_time, 2),
    "matches_classical": qaoa_order in optimal_orders,
    "shots": SHOTS, "valid_shots": valid_shots, "optimal_shots": optimal_shots,
    "mean_valid_time_s": round(mean_valid_time, 2),
    "random_p_valid": len(feasible) / DIM, "random_p_optimal": len(optimal_idx) / DIM,
    "history": [{k: (round(v, 5) if isinstance(v, float) else v) for k, v in h_.items()} for h_ in history],
    "angles": {"gamma": [round(float(v), 5) for v in best[:P_MAX]], "beta": [round(float(v), 5) for v in best[P_MAX:]]},
    "fidelity_vs_training": round(float(fidelity), 6),
}
with open(os.path.join(HERE, "qaoa_delivery_result.json"), "w") as f:
    json.dump(result, f, indent=2)

# ---------------------------------------------------------------------------
# 5. Chart: quality vs depth, and what the samples looked like
# ---------------------------------------------------------------------------
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 3.8))
ps = [h_["p"] for h_ in history]
ax1.plot(ps, [100 * h_["p_feasible"] for h_ in history], "o-", color="#7C9CB3", label="valid route")
ax1.plot(ps, [100 * h_["p_optimal"] for h_ in history], "o-", color="#D9A441", label="optimal route")
ax1.axhline(100 * len(optimal_idx) / DIM, color="#999", ls=":", label="optimal, random guess")
ax1.set_xlabel("QAOA layers (p)"); ax1.set_ylabel("probability (%)")
ax1.set_title("Probability mass QAOA puts on good routes"); ax1.legend(frameon=False)

ranked = sorted(orders, key=lambda o: tours[o][0])
ax2.bar(range(len(ranked)), [valid.get(o, 0) for o in ranked],
        color=["#D9A441" if o in optimal_orders else "#7C9CB3" for o in ranked])
ax2.set_xticks(range(len(ranked)))
ax2.set_xticklabels([f"{tours[o][0]:.0f}s" for o in ranked], rotation=90, fontsize=7)
ax2.set_xlabel("valid routes, fastest → slowest"); ax2.set_ylabel(f"shots of {SHOTS}")
ax2.set_title(f"Samples at p={P_MAX} (gold = optimal)")
fig.tight_layout()
fig.savefig(os.path.join(HERE, "qaoa_delivery_chart.png"), dpi=130)
print("Saved qaoa_delivery_result.json and qaoa_delivery_chart.png")
