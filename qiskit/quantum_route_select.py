"""
Genuine Qiskit demonstration: Grover's search algorithm used to select the
shortest among 4 candidate drone routes.

Honest framing:
- This runs on Qiskit's AerSimulator (a real quantum-circuit simulator that
  evolves actual quantum state vectors and samples measurements) — NOT on
  real IBM Quantum hardware. This sandbox's network access is restricted to
  a fixed allowlist of package registries and cannot reach IBM's cloud
  quantum service, so submitting to real hardware isn't possible here even
  with a valid IBM Quantum API token.
- At this problem size (choosing among 4 known candidates), classical
  comparison is trivially faster than building and running a quantum
  circuit. The point of this demo is to show Grover's algorithm genuinely
  implemented and correctly finding the right answer, not to claim a
  practical speedup at toy scale — real quantum advantage requires much
  larger search spaces than 4 items.
"""

import math
import random
from qiskit import QuantumCircuit
from qiskit_aer import AerSimulator
from qiskit.visualization import plot_histogram

# ---------------------------------------------------------------------------
# 1. Classical setup: same obstacle map and pathfinding as the JS simulator,
#    ported to Python, to generate 4 real candidate routes.
# ---------------------------------------------------------------------------
MARGIN, W, H = 8, 760, 460
OBSTACLES = [
    (60, 130, 70, 26), (60, 250, 46, 90), (160, 60, 90, 24),
    (170, 160, 24, 110), (150, 320, 100, 22), (290, 60, 22, 90),
    (260, 200, 90, 22), (300, 260, 22, 130), (380, 60, 130, 24),
    (400, 140, 24, 80), (430, 260, 100, 22), (400, 340, 24, 90),
    (540, 100, 24, 110), (560, 60, 100, 22), (600, 200, 24, 140),
    (500, 380, 110, 22), (650, 270, 80, 22), (690, 330, 22, 60),
]
PAD, CELL = 16, 10
COLS, ROWS = W // CELL, H // CELL


def cell_blocked(cx, cy):
    x, y = cx * CELL + CELL / 2, cy * CELL + CELL / 2
    if x < MARGIN + PAD or x > W - MARGIN - PAD or y < MARGIN + PAD or y > H - MARGIN - PAD:
        return True
    for ox, oy, ow, oh in OBSTACLES:
        if ox - PAD < x < ox + ow + PAD and oy - PAD < y < oy + oh + PAD:
            return True
    return False


def nearest_walkable(cx, cy):
    if not cell_blocked(cx, cy):
        return cx, cy
    for r in range(1, 26):
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                if max(abs(dx), abs(dy)) != r:
                    continue
                ncx, ncy = cx + dx, cy + dy
                if 0 <= ncx < COLS and 0 <= ncy < ROWS and not cell_blocked(ncx, ncy):
                    return ncx, ncy
    return cx, cy


def astar(sx, sy, gx, gy):
    start = nearest_walkable(sx // CELL, sy // CELL)
    goal = nearest_walkable(gx // CELL, gy // CELL)
    import heapq
    def h(cx, cy):
        dx, dy = abs(cx - goal[0]), abs(cy - goal[1])
        return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)
    open_heap = [(h(*start), start)]
    g = {start: 0}
    came_from = {}
    neighbors8 = [(1,0,1),(-1,0,1),(0,1,1),(0,-1,1),(1,1,math.sqrt(2)),(1,-1,math.sqrt(2)),(-1,1,math.sqrt(2)),(-1,-1,math.sqrt(2))]
    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current == goal:
            path = []
            node = current
            while node in came_from:
                path.append(node)
                node = came_from[node]
            path.append(start)
            path.reverse()
            return [(cx * CELL + CELL/2, cy * CELL + CELL/2) for cx, cy in path]
        for dx, dy, cost in neighbors8:
            ncx, ncy = current[0] + dx, current[1] + dy
            if not (0 <= ncx < COLS and 0 <= ncy < ROWS) or cell_blocked(ncx, ncy):
                continue
            if dx and dy and (cell_blocked(current[0]+dx, current[1]) or cell_blocked(current[0], current[1]+dy)):
                continue
            tentative = g[current] + cost
            if tentative < g.get((ncx, ncy), math.inf):
                g[(ncx, ncy)] = tentative
                came_from[(ncx, ncy)] = current
                heapq.heappush(open_heap, (tentative + h(ncx, ncy), (ncx, ncy)))
    return None


def segment_clear(p1, p2):
    dist = math.hypot(p2[0]-p1[0], p2[1]-p1[1])
    steps = max(1, math.ceil(dist / 4))
    for i in range(steps + 1):
        t = i / steps
        x, y = p1[0] + (p2[0]-p1[0])*t, p1[1] + (p2[1]-p1[1])*t
        if x < MARGIN+PAD or x > W-MARGIN-PAD or y < MARGIN+PAD or y > H-MARGIN-PAD:
            return False
        for ox, oy, ow, oh in OBSTACLES:
            if ox-PAD < x < ox+ow+PAD and oy-PAD < y < oy+oh+PAD:
                return False
    return True


def smooth_path(path):
    if len(path) <= 2:
        return path
    result = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not segment_clear(path[i], path[j]):
            j -= 1
        result.append(path[j])
        i = j
    return result


def anneal_path(path, iterations, rng):
    if len(path) <= 2:
        return path, 0.0
    pts = [list(p) for p in path]
    def total_len(p):
        return sum(math.hypot(p[i][0]-p[i-1][0], p[i][1]-p[i-1][1]) for i in range(1, len(p)))
    start_len = total_len(pts)
    best_pts, best_len, temp = [p[:] for p in pts], start_len, 12.0
    for _ in range(iterations):
        temp *= 0.99
        idx = rng.randint(1, len(pts) - 2)
        old = pts[idx][:]
        dx, dy = (rng.random()-0.5)*temp, (rng.random()-0.5)*temp
        cand = [old[0]+dx, old[1]+dy]
        if not segment_clear(pts[idx-1], cand) or not segment_clear(cand, pts[idx+1]):
            continue
        before = math.hypot(pts[idx][0]-pts[idx-1][0], pts[idx][1]-pts[idx-1][1]) + math.hypot(pts[idx+1][0]-pts[idx][0], pts[idx+1][1]-pts[idx][1])
        after = math.hypot(cand[0]-pts[idx-1][0], cand[1]-pts[idx-1][1]) + math.hypot(pts[idx+1][0]-cand[0], pts[idx+1][1]-cand[1])
        delta = after - before
        if delta < 0 or rng.random() < math.exp(-delta / max(0.001, temp*0.15)):
            pts[idx] = cand
            cur_len = total_len(pts)
            if cur_len < best_len:
                best_len, best_pts = cur_len, [p[:] for p in pts]
    if best_len >= start_len:
        return path, 0.0
    return best_pts, (start_len - best_len) / start_len * 100


START, DEST = (60, 60), (690, 400)
raw = astar(*START, *DEST)
smoothed = smooth_path(raw)

candidates = []
for seed in [1, 2, 3, 4]:
    rng = random.Random(seed)
    refined, improved = anneal_path(smoothed, 300, rng)
    length = sum(math.hypot(refined[i][0]-refined[i-1][0], refined[i][1]-refined[i-1][1]) for i in range(1, len(refined)))
    candidates.append({"seed": seed, "path": refined, "length": length})

for c in candidates:
    print(f"  candidate seed={c['seed']}: length={c['length']:.2f}px")

winner_idx = min(range(4), key=lambda i: candidates[i]["length"])
print(f"\nClassical ground truth: candidate {winner_idx} is shortest ({candidates[winner_idx]['length']:.2f}px)")

# ---------------------------------------------------------------------------
# 2. Real Qiskit circuit: Grover's search over 2 qubits (4 basis states),
#    oracle marks the classically-shortest candidate's index.
# ---------------------------------------------------------------------------
def grover_find(marked_index: int, shots: int = 2048):
    qc = QuantumCircuit(2, 2)
    qc.h([0, 1])  # uniform superposition over all 4 candidates

    # Oracle: flip the phase of |marked_index>. CZ on |11> naturally marks
    # index 3; X-gates on the 0-bits of the target sandwich the CZ to
    # retarget it at the desired index (standard technique).
    bits = format(marked_index, '02b')
    for i, b in enumerate(reversed(bits)):
        if b == '0':
            qc.x(i)
    qc.cz(0, 1)
    for i, b in enumerate(reversed(bits)):
        if b == '0':
            qc.x(i)

    # Diffuser (inversion about the mean) — the other half of one Grover iteration.
    qc.h([0, 1])
    qc.x([0, 1])
    qc.cz(0, 1)
    qc.x([0, 1])
    qc.h([0, 1])

    qc.measure([0, 1], [0, 1])

    sim = AerSimulator()
    result = sim.run(qc, shots=shots).result()
    counts = result.get_counts()
    return qc, counts


circuit, counts = grover_find(winner_idx)
print("\nQiskit circuit:")
print(circuit.draw(output='text'))

print("\nMeasurement counts (2048 shots):")
for bitstring, n in sorted(counts.items(), key=lambda kv: -kv[1]):
    idx = int(bitstring, 2)
    marker = "  <-- classical winner" if idx == winner_idx else ""
    print(f"  |{bitstring}> (candidate {idx}): {n:5d} shots ({n/2048*100:.1f}%){marker}")

found_idx = max(counts, key=counts.get)
found_idx = int(found_idx, 2)
print(f"\nGrover's algorithm found candidate {found_idx} as most probable.")
print(f"Matches classical ground truth: {found_idx == winner_idx}")

# Save histogram + winning path for the write-up
import json
import os
HERE = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(HERE, 'quantum_route_result.json'), 'w') as f:
    json.dump({
        "candidates": [{"seed": c["seed"], "length": c["length"]} for c in candidates],
        "winner_idx": winner_idx,
        "counts": counts,
        "found_idx": found_idx,
        "winning_path": candidates[winner_idx]["path"],
    }, f, indent=2)

fig = plot_histogram(counts, title="Grover's search — candidate route selection")
fig.savefig(os.path.join(HERE, 'quantum_route_histogram.png'), dpi=130, bbox_inches='tight')
print("\nSaved histogram.png and result.json")
