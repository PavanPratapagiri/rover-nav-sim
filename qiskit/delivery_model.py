"""
Delivery-mission cost model shared by the quantum demos — mirrors
planMission / walkTour in sim-3d/rover-sim-3d.html, so a route scored here
scores the same (to within A* tie-breaking, ~0.5%) in the simulator.
"""

import math

import nav_map as nm

BASE = (60.0, 60.0)
ROVER_SPEED = 62.0
FLY_DRAIN, ENERGY_MARGIN, RESERVE, CHARGE_RATE = 1.1, 1.3, 10.0, 25.0


def random_free_point(rng):
    # same rule as randomFreePoint() in the sim
    for _ in range(30):
        px = nm.MARGIN + 30 + rng.random() * (nm.W - nm.MARGIN * 2 - 60)
        py = nm.MARGIN + 30 + rng.random() * (nm.H - nm.MARGIN * 2 - 60)
        if not any(ox - 20 < px < ox + ow + 20 and oy - 20 < py < oy + oh + 20 for ox, oy, ow, oh in nm.OBSTACLES):
            return (px, py)
    return (px, py)


def pick_stops(rng, n_stops):
    stops = []
    while len(stops) < n_stops:
        p = random_free_point(rng)
        # keep stops visibly distinct and away from the base pad
        if math.dist(p, BASE) > 80 and all(math.dist(p, q) > 80 for q in stops):
            stops.append(p)
    return stops


def path_length(pts):
    return sum(math.dist(pts[i - 1], pts[i]) for i in range(1, len(pts)))


def route_length(a, b):
    raw = nm.astar(a[0], a[1], b[0], b[1])
    return path_length([a, *nm.smooth_path(raw), b]) if raw else math.inf


def distance_matrix(pts):
    """Symmetric A* road distances between all points (index 0 = base)."""
    n = len(pts)
    dist = [[0.0] * n for _ in range(n)]
    for i in range(n):
        for j in range(i + 1, n):
            dist[i][j] = dist[j][i] = route_length(pts[i], pts[j])
    return dist


def energy_for(length):
    return length / ROVER_SPEED * FLY_DRAIN * ENERGY_MARGIN


def walk_tour(order, dist, start_charge=100.0):
    """(estimated mission time s, length px, recharges) for visiting stops in `order` (1-based)."""
    charge, at, length, charge_time, recharges = start_charge, 0, 0.0, 0.0, 0
    for k in order:
        need = energy_for(dist[at][k]) + energy_for(dist[k][0]) + RESERVE
        if charge < need and charge < 99.9:
            length += dist[at][0]
            charge_time += (100 - max(0.0, charge - energy_for(dist[at][0]))) / CHARGE_RATE
            charge, at, recharges = 100.0, 0, recharges + 1
        charge -= energy_for(dist[at][k])
        length += dist[at][k]
        at = k
    length += dist[at][0]
    return length / ROVER_SPEED + charge_time, length, recharges
