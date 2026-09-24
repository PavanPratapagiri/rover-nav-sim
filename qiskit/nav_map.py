"""
Shared map + pathfinding for the Qiskit demos — a Python port of the nav
core in sim-3d/rover-sim-3d.html (same obstacle map, PAD, grid A*, and
line-of-sight smoothing). If you change the obstacle map in the 3D sim,
change it here too and re-run the demos: embedded results go stale.
"""

import math
import heapq

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
