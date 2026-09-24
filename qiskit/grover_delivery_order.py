"""
Grover's search picks the visiting order for a real 5-stop delivery mission.

What it does
- Fixes a 5-stop mission on the sim-3d map (stops chosen with a seeded RNG
  using the same "free point" rule as the sim).
- Scores all 5! = 120 visiting orders with the SAME cost model the 3D sim's
  planner uses: A* road distances between stops, then an estimated mission
  time (flight + any recharge detours) from a simulated battery walk.
- Encodes the orders as basis states of 7 qubits (128 states; the 8 spare
  states are invalid and never marked) and runs Dürr–Høyer minimum finding:
  repeated Grover searches for "an order faster than the best so far", with
  the BBHT randomized iteration schedule (no knowledge of how many orders
  are better). Every measured candidate is checked classically, and the
  threshold tightens until the search budget is spent.
- Runs on Qiskit's AerSimulator (a local state-vector simulator, not IBM
  hardware) with fixed seeds, so the result is reproducible.

Honest framing
- The oracle is a diagonal phase gate built from the 120 costs computed
  classically up front. Building it already required evaluating every
  order, so this demo cannot be faster than the classical planner, which
  evaluates all 120 in about a millisecond. A real speed-up would need an
  oracle that computes route cost *inside* the circuit (quantum
  arithmetic), plus far larger problems and fault-tolerant hardware.
- What IS genuine: the quantum search itself. Grover finds the optimum
  from a uniform superposition using ~sqrt(N) oracle queries per search,
  and its answer is checked against the exact classical optimum.

Output: grover_delivery_result.json (embedded into sim-3d as
GROVER_DELIVERY) and grover_delivery_histogram.png.

Run:  qiskit/.venv/bin/python qiskit/grover_delivery_order.py
"""

import json
import math
import os
import random
from itertools import permutations

from qiskit import QuantumCircuit, transpile
from qiskit.circuit.library import DiagonalGate
from qiskit_aer import AerSimulator

import delivery_model as dm

HERE = os.path.dirname(os.path.abspath(__file__))
SEED = 17  # chosen so the optimal order is not the stops' generation order (seed 7 was — a confusing demo)

# ---------------------------------------------------------------------------
# 1. Mission + cost model (shared with the QAOA demo — see delivery_model.py)
# ---------------------------------------------------------------------------
N_STOPS = 5
rng = random.Random(SEED)
stops = dm.pick_stops(rng, N_STOPS)
dist = dm.distance_matrix([dm.BASE, *stops])

orders = list(permutations(range(1, N_STOPS + 1)))            # 120 orders
N_QUBITS = math.ceil(math.log2(len(orders)))                   # 7
N_STATES = 2 ** N_QUBITS                                       # 128
cost = [dm.walk_tour(o, dist)[0] for o in orders] + [math.inf] * (N_STATES - len(orders))

best_cost = min(cost)
optimal = [i for i, c in enumerate(cost) if abs(c - best_cost) < 1e-9]
print(f"{N_STOPS} stops -> {len(orders)} orders, {N_QUBITS} qubits ({N_STATES} states)")
print(f"Classical exact optimum: {best_cost:.2f}s, states {optimal} "
      f"(a tour and its reverse tie when no recharge is needed)")

# ---------------------------------------------------------------------------
# 2. Grover circuits
# ---------------------------------------------------------------------------
sim = AerSimulator(seed_simulator=SEED)


def grover_circuit(marked, iterations):
    qc = QuantumCircuit(N_QUBITS)
    qubits = list(range(N_QUBITS))
    qc.h(qubits)
    oracle = DiagonalGate([-1 if i in marked else 1 for i in range(N_STATES)])
    for _ in range(iterations):
        qc.append(oracle, qubits)                 # phase-flip the marked orders
        qc.h(qubits); qc.x(qubits)                # diffuser: inversion about the mean
        qc.h(N_QUBITS - 1); qc.mcx(qubits[:-1], N_QUBITS - 1); qc.h(N_QUBITS - 1)
        qc.x(qubits); qc.h(qubits)
    qc.measure_all()
    return qc


def run(marked, iterations, shots):
    qc = transpile(grover_circuit(marked, iterations), sim)
    counts = sim.run(qc, shots=shots).result().get_counts()
    return {int(b, 2): c for b, c in counts.items()}


# ---------------------------------------------------------------------------
# 3. Dürr–Høyer minimum finding with BBHT iteration schedule
# ---------------------------------------------------------------------------
budget = math.ceil(22.5 * math.sqrt(N_STATES) + 1.4 * math.log2(N_STATES) ** 2)
y = rng.randrange(len(orders))
oracle_calls, measurements, rounds = 0, 0, [{"threshold_s": round(cost[y], 2), "state": y}]
print(f"\nDürr–Høyer: start at random order {y} ({cost[y]:.2f}s), oracle budget {budget}")

while oracle_calls < budget:
    threshold = cost[y]
    marked = {i for i in range(N_STATES) if cost[i] < threshold}
    m, found = 1.0, None
    while oracle_calls < budget:
        j = rng.randrange(math.ceil(m))           # BBHT: random iteration count < m
        x = next(iter(run(marked, j, shots=1)))   # one measurement
        oracle_calls += j
        measurements += 1
        if cost[x] < threshold:                   # classical check of the candidate
            found = x
            break
        m = min(m * 6 / 5, math.sqrt(N_STATES))
    if found is None:
        break                                     # budget spent: y is the minimum w.h.p.
    y = found
    rounds.append({"threshold_s": round(cost[y], 2), "state": y})
    print(f"  improved -> order {y}: {cost[y]:.2f}s  (oracle calls so far {oracle_calls})")

grover_order = list(orders[y])
matches = abs(cost[y] - best_cost) < 1e-9
print(f"\nGrover's pick: state {y}, order {grover_order}, {cost[y]:.2f}s")
print(f"Matches classical optimum: {matches}  |  {oracle_calls} oracle calls, {measurements} measurements")

# ---------------------------------------------------------------------------
# 4. Showcase run: one Grover search marking the optimal order(s), optimal
#    iteration count, many shots — shows the amplitude amplification.
# ---------------------------------------------------------------------------
SHOTS = 2048
k_opt = math.floor(math.pi / 4 * math.sqrt(N_STATES / len(optimal)))
showcase = run(set(optimal), k_opt, SHOTS)
hit = sum(showcase.get(i, 0) for i in optimal)
print(f"\nShowcase: {k_opt} Grover iterations, {hit}/{SHOTS} shots ({hit / SHOTS:.1%}) land on an optimal order "
      f"(uniform guessing: {len(optimal) / len(orders):.1%})")

# ---------------------------------------------------------------------------
# 5. Save for the 3D sim + a histogram
# ---------------------------------------------------------------------------
ranked = sorted(range(len(orders)), key=lambda i: cost[i])
result = {
    "seed": SEED,
    "stops": [{"x": round(p[0], 2), "y": round(p[1], 2)} for p in stops],
    "num_orders": len(orders),
    "qubits": N_QUBITS,
    "grover_state": y,
    "grover_order": grover_order,
    "grover_time_s": round(cost[y], 2),
    "classical_best_time_s": round(best_cost, 2),
    "worst_time_s": round(max(cost[:len(orders)]), 2),
    "matches_classical": matches,
    "oracle_calls": oracle_calls,
    "measurements": measurements,
    "rounds": rounds,
    "showcase": {"iterations": k_opt, "shots": SHOTS, "optimal_hits": hit,
                 "uniform_pct": round(len(optimal) / len(orders) * 100, 2)},
    "top_orders": [{"state": i, "order": list(orders[i]), "time_s": round(cost[i], 2),
                    "shots": showcase.get(i, 0)} for i in ranked[:6]],
}
with open(os.path.join(HERE, "grover_delivery_result.json"), "w") as f:
    json.dump(result, f, indent=2)

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

xs = list(range(len(orders)))
fig, ax = plt.subplots(figsize=(10, 3.6))
ax.bar(xs, [showcase.get(i, 0) for i in xs], color=["#D9A441" if i in optimal else "#7C9CB3" for i in xs], width=1.0)
ax.set_xlabel(f"visiting order (basis state 0–{len(orders) - 1})")
ax.set_ylabel(f"shots of {SHOTS}")
ax.set_title(f"Grover's search over {len(orders)} delivery orders — {k_opt} iterations, "
             f"{hit / SHOTS:.1%} of shots on an optimal order (gold: the tour + its reverse)")
fig.tight_layout()
fig.savefig(os.path.join(HERE, "grover_delivery_histogram.png"), dpi=130)
print("Saved grover_delivery_result.json and grover_delivery_histogram.png")
