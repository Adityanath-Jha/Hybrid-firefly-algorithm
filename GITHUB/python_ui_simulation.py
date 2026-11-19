import math, random, time
import tkinter as tk
from tkinter import ttk

AREA, BASE = 10.0, (0.0, 0.0)
ALPHA, BETA0, STEP = 0.25, 1.0, 0.45
FOUND_DIST, SENSE_RANGE, ENERGY_WEIGHT, PHASE_SWITCH = 0.55, 4.0, 0.5, 60
SPEED_MIN, SPEED_MAX, STOP_MIN, STOP_MAX = 0.04, 0.20, 50, 95
DRONE_MARGIN, SURV_MARGIN = 0.28, 0.35
MAX_SURV_TRAIL, MAX_DRONE_TRAIL = 5, 4
ESC_WINDOW, ESC_PROGRESS, ESC_STEPS = 20, 0.6, 14   # stagnation escape: window, max wander radius, escape length
SEP_DIST, SEP_GAIN, COVER_DIST, COVER_GAIN = 0.8, 0.9, 2.2, 1.6
ZONE_GROW = 12                                       # steps for a late hazard to reach full size
CHARGE_RATE = 6.0                                    # battery units a docked drone regains per step
FA_PULL_CAP = 1.0                                    # max combined swarm pull 

C_BG, C_PANEL, C_BORDER, C_GRID, C_CARD = "#F1F5F9", "#FFFFFF", "#CBD5E1", "#E2E8F0", "#F8FAFC"
C_TEXT, C_MUTED, C_PRIMARY = "#0F172A", "#64748B", "#0284C7"
C_BLUE, C_AMBER, C_GREEN, C_RED = "#2563EB", "#D97706", "#16A34A", "#DC2626"

def dist(ax, ay, bx, by): return math.hypot(ax - bx, ay - by)
def lerp(a, b, t): return a + (b - a) * t
def clamp(v, lo, hi): return max(lo, min(hi, v))
def rand_pt(r, m=1.5): return m + r.random() * (AREA - 2 * m), m + r.random() * (AREA - 2 * m)

def project_outside_hazards(x, y, zones, margin):
    """Hard boundary: (x, y) is pushed out of every hazard circle."""
    for _ in range(3):
        for z in zones:
            d, safe = dist(x, y, z["x"], z["y"]), z["r"] + margin
            if d < safe:
                if d > 0.001: k = safe / d; x, y = z["x"] + (x - z["x"]) * k, z["y"] + (y - z["y"]) * k
                else: x, y = z["x"] + safe, z["y"]
    return x, y

def zone(x, y, r, vx=0.0, vy=0.0, frm=0):
    return {"x": x, "y": y, "prev_x": x, "prev_y": y, "r": 0.0 if frm else r, "rf": r, "vx": vx, "vy": vy, "from": frm}

# ============================== SIMULATION ENGINE ==============================
class Simulation:
    def __init__(self, n_surv=8, n_dyn=4, n_drones=16, comm=4.0, battery=250, fixed_zone=True, moving_zone=True,
                 new_zone=False, comm_drop=False, manual_stop=False, seed=7):
        self.seed, self.rng = seed, random.Random(seed)
        r = self.rng
        self.n_dyn, self.comm, self.comm_drop, self.manual_stop = min(n_dyn, n_surv), comm, comm_drop, manual_stop
        self.gen, self.done, self.rtb, self.all_parked, self.static_pending, self.assign = 0, False, False, False, True, {}

        self.obs = []
        if fixed_zone: self.obs.append(zone(3.5 + r.random() * 3.5, 5.5 + r.random() * 2.5, 1.0 + r.random() * 0.3))
        if moving_zone:
            ang, spd = r.random() * 2 * math.pi, 0.025 + r.random() * 0.015
            self.obs.append(zone(4.5 + r.random() * 4.0, 2.5 + r.random() * 3.5, 0.9 + r.random() * 0.25,
                                 spd * math.cos(ang), spd * math.sin(ang)))
        if new_zone: self.obs.append(zone(2.5 + r.random() * 4.5, 3.0 + r.random() * 3.5, 0.85 + r.random() * 0.2, frm=50))

        self.survs, n_static = [], n_surv - self.n_dyn
        for i in range(n_surv):
            for _ in range(300):
                x, y = 1.0 + r.random() * (AREA - 2.0), 1.0 + r.random() * (AREA - 2.0)
                if dist(x, y, *BASE) > 2.5 and all(dist(x, y, o["x"], o["y"]) > o["rf"] + 1.2 for o in self.obs): break
            dyn = i >= n_static
            self.survs.append({"id": i + 1, "x": x, "y": y, "prev_x": x, "prev_y": y, "dyn": dyn, "moving": dyn,
                               "speed": SPEED_MIN + r.random() * (SPEED_MAX - SPEED_MIN) if dyn else 0.0,
                               "heading": r.random() * 2 * math.pi, "stop_gen": r.randint(STOP_MIN, STOP_MAX),
                               "marked": False, "mark_pos": None, "followers": 0, "trail": [],
                               "coord_log": [(0, round(x, 2), round(y, 2))]})

        n_active = max(1, math.ceil(n_drones / 2)) if battery else n_drones      # 50% deployed, rest on standby
        self.drones = []
        for i in range(n_drones):
            act = i < n_active
            sx = (0.2 + r.random() * 1.0) if act else BASE[0]
            sy = (0.2 + r.random() * 1.0) if act else BASE[1]
            wx, wy = rand_pt(r)
            self.drones.append({"id": i + 1, "x": sx, "y": sy, "prev_x": sx, "prev_y": sy, "wx": wx, "wy": wy,
                                "battery": float(battery) if battery else float("inf"), "alive": True,
                                "status": "active" if act else "standby", "parked": not act, "trail": [],
                                "stuck_cnt": 0, "hist": [], "esc": 0, "esc_dir": 1.0})
        self.cap, self.swaps, self.n_target = battery, 0, n_active

    def in_zone(self, x, y): return any(self.gen >= o["from"] and dist(x, y, o["x"], o["y"]) < o["r"] for o in self.obs)
    def active_zones(self): return [o for o in self.obs if self.gen >= o["from"]]
    def footprints(self): return [dict(o, r=o["rf"]) for o in self.obs]      # final size, even for a late zone not yet active

    def _update_zones(self):
        for o in self.obs:
            o["prev_x"], o["prev_y"] = o["x"], o["y"]
            if self.gen >= o["from"] > 0: o["r"] = o["rf"] * min(1.0, (self.gen - o["from"] + 1) / ZONE_GROW)
            if self.gen < o["from"] or (o["vx"] == 0 and o["vy"] == 0): continue
            lo, hi = o["r"] + 0.6, AREA - o["r"] - 0.6
            if (o["x"] + o["vx"] < lo and o["vx"] < 0) or (o["x"] + o["vx"] > hi and o["vx"] > 0): o["vx"] = -o["vx"]
            if (o["y"] + o["vy"] < lo and o["vy"] < 0) or (o["y"] + o["vy"] > hi and o["vy"] > 0): o["vy"] = -o["vy"]
            keep = [(BASE, o["r"] + 2.5)] + [((s["x"], s["y"]), o["r"] + SURV_MARGIN + 0.45) for s in self.survs if not s["moving"]]

            def blocker(vx, vy):                  # centre this move would close in on (base / standing or marked survivor)
                return next((c for c, m in keep if dist(o["x"] + vx, o["y"] + vy, *c) < m and
                             dist(o["x"] + vx, o["y"] + vy, *c) < dist(o["x"], o["y"], *c)), None)
            c = blocker(o["vx"], o["vy"])
            if c:                                 # slide around it (drop the inward velocity); reverse only if that is blocked too
                kd = max(dist(o["x"], o["y"], *c), 0.001); nx, ny = (o["x"] - c[0]) / kd, (o["y"] - c[1]) / kd
                dot, sp = o["vx"] * nx + o["vy"] * ny, math.hypot(o["vx"], o["vy"])
                tx, ty = o["vx"] - dot * nx, o["vy"] - dot * ny; tn = math.hypot(tx, ty)
                o["vx"], o["vy"] = (tx / tn * sp, ty / tn * sp) if tn > 1e-9 else (-o["vx"], -o["vy"])
                if blocker(o["vx"], o["vy"]): o["vx"], o["vy"] = -o["vx"], -o["vy"]
            if not blocker(o["vx"], o["vy"]): o["x"] += o["vx"]; o["y"] += o["vy"]     # boxed in > wait, never jitter
        az = self.active_zones()
        for s in self.survs:
            if s["moving"]: s["x"], s["y"] = project_outside_hazards(s["x"], s["y"], self.footprints(), SURV_MARGIN)
        for d in self.drones:
            nx, ny = project_outside_hazards(d["x"], d["y"], az, DRONE_MARGIN)
            if (nx, ny) != (d["x"], d["y"]): d["x"], d["y"], d["prev_x"], d["prev_y"] = nx, ny, nx, ny

    def _safe_drone_step(self, d, tx, ty):
        """Potential-field step with goal-directed deflection and a stagnation escape."""
        fx, fy, zones = tx, ty, self.active_zones()
        near = lambda edge: min(zones, key=lambda z: dist(d["x"], d["y"], z["x"], z["y"]) - (z["r"] if edge else 0))
        hist = d["hist"]
        hist.append((d["x"], d["y"], math.hypot(tx, ty) > 0.25)); del hist[:-ESC_WINDOW]
        if zones and d["esc"] == 0 and len(hist) == ESC_WINDOW and all(h[2] for h in hist) \
                and max(dist(hist[0][0], hist[0][1], h[0], h[1]) for h in hist) < ESC_PROGRESS:
            cl = near(True)                                   # trapped: slide around the nearest hazard edge
            kd = max(0.001, dist(d["x"], d["y"], cl["x"], cl["y"]))
            cross = ((d["x"] - cl["x"]) / kd) * ty - ((d["y"] - cl["y"]) / kd) * tx
            d["esc"], d["esc_dir"] = ESC_STEPS, (-1.0 if cross < 0 else 1.0)
            hist.clear()
        if d["esc"] > 0 and zones:
            d["esc"] -= 1
            cl = near(True)
            kd = max(0.001, dist(d["x"], d["y"], cl["x"], cl["y"]))
            rx, ry = (d["x"] - cl["x"]) / kd, (d["y"] - cl["y"]) / kd
            mx, my = (-ry * d["esc_dir"] + rx * 0.35) * STEP, (rx * d["esc_dir"] + ry * 0.35) * STEP
            if sum(dist(d["x"], d["y"], z["x"], z["y"]) < z["r"] + DRONE_MARGIN + 0.35 for z in zones) >= 2:   # wedged between two hazards: back straight out
                ax = sum((d["x"] - z["x"]) / max(dist(d["x"], d["y"], z["x"], z["y"]), 0.001) ** 2 for z in zones)
                ay = sum((d["y"] - z["y"]) / max(dist(d["x"], d["y"], z["x"], z["y"]), 0.001) ** 2 for z in zones)
                mx, my = ax / max(math.hypot(ax, ay), 1e-9) * STEP, ay / max(math.hypot(ax, ay), 1e-9) * STEP
            nx, ny = clamp(d["x"] + mx, 0.2, AREA - 0.2), clamp(d["y"] + my, 0.2, AREA - 0.2)
            nx, ny = project_outside_hazards(nx, ny, zones, DRONE_MARGIN)
            if dist(d["x"], d["y"], nx, ny) < 0.04: d["esc_dir"] = -d["esc_dir"]     # wall in the way
            return nx, ny
        for o in zones:
            dd, r_danger = dist(d["x"], d["y"], o["x"], o["y"]), o["r"] + DRONE_MARGIN + 1.35
            if 0.001 < dd < r_danger:
                rx, ry = (d["x"] - o["x"]) / dd, (d["y"] - o["y"]) / dd
                rep = ((r_danger - dd) / (r_danger - o["r"])) ** 2 * 3.6
                turn = -1.0 if rx * ty - ry * tx < 0 else 1.0
                if d["stuck_cnt"] > 8: turn, rep = -turn, rep * 1.3
                fx += rx * rep - ry * turn * rep * 1.25
                fy += ry * rep + rx * turn * rep * 1.25
        n = math.hypot(fx, fy)
        if n > STEP: fx, fy = fx / n * STEP, fy / n * STEP
        elif n < 0.001: return d["x"], d["y"]
        nx, ny = clamp(d["x"] + fx, 0.2, AREA - 0.2), clamp(d["y"] + fy, 0.2, AREA - 0.2)
        nx, ny = project_outside_hazards(nx, ny, zones, DRONE_MARGIN)
        if dist(d["x"], d["y"], nx, ny) < 0.04:
            d["stuck_cnt"] += 1
            if d["stuck_cnt"] > 10 and zones:
                cl = near(False)
                kd = max(0.001, dist(d["x"], d["y"], cl["x"], cl["y"]))
                nx = clamp(nx + ((d["x"] - cl["x"]) / kd) * STEP * 0.8, 0.2, AREA - 0.2)
                ny = clamp(ny + ((d["y"] - cl["y"]) / kd) * STEP * 0.8, 0.2, AREA - 0.2)
                nx, ny = project_outside_hazards(nx, ny, zones, DRONE_MARGIN)
                d["stuck_cnt"] = 0
        else: d["stuck_cnt"] = max(0, d["stuck_cnt"] - 1)
        return nx, ny
    def _move(self, d, pos):
        d["battery"] -= dist(d["x"], d["y"], *pos) * 1.5
        d["x"], d["y"] = pos
        if d["battery"] <= 0: d["alive"] = False
    def _fly_home(self, d):
        bd = max(dist(d["x"], d["y"], *BASE), 0.01)
        if bd < 0.40:
            d.update(status="parked", parked=True, x=BASE[0], y=BASE[1], trail=[]); return
        self._move(d, self._safe_drone_step(d, (BASE[0] - d["x"]) / bd * STEP, (BASE[1] - d["y"]) / bd * STEP))

    def _log(self, s): s["coord_log"].append((self.gen, round(s["x"], 2), round(s["y"], 2)))
    def toggle_stop(self, sid):
        s = self.survs[sid - 1]
        if s["dyn"] and not s["marked"]:
            s["moving"] = not s["moving"]
            if s["moving"]: s["stop_gen"] = max(s["stop_gen"], self.gen + STOP_MIN)    # resumed: don't re-halt at once
            else: s["trail"] = []; self._log(s)

    def stop_all(self):
        for s in self.survs:
            if s["dyn"] and not s["marked"] and s["moving"]: s["moving"], s["trail"] = False, []; self._log(s)

    def resume_all(self):
        for s in self.survs:
            if s["dyn"] and not s["marked"]: s["moving"], s["stop_gen"] = True, max(s["stop_gen"], self.gen + STOP_MIN)

    def _move_survivors(self):
        r, gen = self.rng, self.gen
        for s in self.survs:
            if s["moving"] and not self.manual_stop and gen >= s["stop_gen"]: s["moving"] = False; self._log(s)
            if not s["moving"]: continue
            s["heading"] += (r.random() - 0.5) * 0.6
            nx, ny = s["x"] + s["speed"] * math.cos(s["heading"]), s["y"] + s["speed"] * math.sin(s["heading"])
            if nx < 0.25 or nx > AREA - 0.25: s["heading"] = math.pi - s["heading"]
            if ny < 0.25 or ny > AREA - 0.25: s["heading"] = -s["heading"]
            fp = self.footprints()
            for o in fp:
                safe_r = o["r"] + SURV_MARGIN
                if dist(nx, ny, o["x"], o["y"]) < safe_r:
                    norm = math.atan2(ny - o["y"], nx - o["x"])
                    s["heading"] = norm + (r.random() - 0.5) * 0.4
                    nx, ny = o["x"] + safe_r * math.cos(norm), o["y"] + safe_r * math.sin(norm)
            nx, ny = project_outside_hazards(nx, ny, fp, SURV_MARGIN)
            s["x"], s["y"] = clamp(nx, 0.25, AREA - 0.25), clamp(ny, 0.25, AREA - 0.25)
            self._log(s)

    def _relay(self, unmarked, crowd):
        """Low battery -> return to base and recharge; a fully charged standby drone replaces it. Repeats until all are marked."""
        r, reserve = self.rng, 16.0 + 2.0 * len(self.active_zones())
        for d in self.drones:                                 # docked drones recharge, then rejoin the standby
            if d["status"] == "parked" and d["alive"] and self.cap:
                d["battery"] = min(self.cap, d["battery"] + CHARGE_RATE)
                if d["battery"] >= self.cap: d["status"] = "standby"
        for d in self.drones:
            if d["status"] == "active" and d["alive"] and d["battery"] < 2.0 * max(dist(d["x"], d["y"], *BASE), 0.01) + reserve:
                d["status"] = "returning"
        n_act = sum(1 for d in self.drones if d["alive"] and d["status"] == "active")
        for stb in [o for o in self.drones if o["status"] == "standby" and o["alive"]]:      # refill to full battery
            if n_act >= self.n_target: break
            stb["status"], stb["parked"] = "active", False; n_act += 1; self.swaps += 1
            if unmarked:                                     
                t = min(unmarked, key=lambda s: (crowd[s["id"] - 1], dist(s["x"], s["y"], *BASE)))
                stb["wx"] = clamp(t["x"] + (r.random() - 0.5) * 1.5, 1.0, AREA - 1.0)
                stb["wy"] = clamp(t["y"] + (r.random() - 0.5) * 1.5, 1.0, AREA - 1.0)
            else: stb["wx"], stb["wy"] = rand_pt(r)

    def step(self):
        if self.done: return
        r, survs, drones, gen = self.rng, self.survs, self.drones, self.gen
        for s in survs:
            s["prev_x"], s["prev_y"] = s["x"], s["y"]
            s["trail"] = (s["trail"] + [(s["prev_x"], s["prev_y"])])[-MAX_SURV_TRAIL:] if s["dyn"] and s["moving"] else []
        for d in drones:
            d["prev_x"], d["prev_y"] = d["x"], d["y"]
            d["trail"] = (d["trail"] + [(d["prev_x"], d["prev_y"])])[-MAX_DRONE_TRAIL:] \
                if d["alive"] and d["status"] in ("active", "returning") else []

        if not self.rtb and all(s["marked"] for s in survs): self.rtb = True      # all located > go home
        if self.rtb:
            for d in drones:
                if d["alive"] and not d["parked"] and d["status"] != "standby": self._fly_home(d)
            self._update_zones(); self.gen += 1
            if all(d["parked"] or not d["alive"] for d in drones): self.all_parked = self.done = True
            return

        self._move_survivors()
        self._update_zones()

        act = [d for d in drones if d["alive"] and d["status"] == "active"]
        unmarked = [s for s in survs if not s["marked"]]
        guard = max(1, min(3, len(act) // max(1, len(unmarked))))
        self.static_pending = any(not s["marked"] and not s["moving"] for s in survs)
        chase = not self.static_pending or gen >= PHASE_SWITCH
        comm = max(1.0, self.comm * 0.4 if (self.comm_drop and gen >= 60) else self.comm)
        gamma = math.log(2) / (comm * comm)
        self.assign = assign = self._assign_targets(act, survs, chase, comm, guard)
        crowd = [0] * len(survs)
        for k in assign.values(): crowd[k] += 1
        self._relay(unmarked, crowd)

        light = {}                                            # firefly brightness function
        for d in drones:
            if not d["alive"] or d["status"] != "active" or d["esc"] > 0: light[d["id"]] = -1e9; continue
            k = assign.get(d["id"])
            best = 100.0 / (1.0 + dist(d["x"], d["y"], survs[k]["x"], survs[k]["y"])) if k is not None else 0.0
            best -= ENERGY_WEIGHT * 0.15 * dist(d["x"], d["y"], *BASE)
            light[d["id"]] = best - 500.0 if self.in_zone(d["x"], d["y"]) else best

        for d in drones:
            if not d["alive"] or d["status"] == "standby" or d["parked"]: continue
            if d["status"] == "returning": self._fly_home(d); continue
            k, tx, ty = assign.get(d["id"]), 0.0, 0.0
            for o in act:                                     # attraction: brighter drones on the same target, in communication range (here, comm range)
                if o["id"] == d["id"] or k is None or assign.get(o["id"]) != k or light[o["id"]] <= light[d["id"]]: continue
                rr = dist(d["x"], d["y"], o["x"], o["y"])
                if 0 < rr <= comm:
                    beta = 0.5 * BETA0 * math.exp(-gamma * rr * rr)
                    tx += beta * (o["x"] - d["x"]); ty += beta * (o["y"] - d["y"])
            fm = math.hypot(tx, ty)
            if fm > FA_PULL_CAP: tx, ty = tx / fm * FA_PULL_CAP, ty / fm * FA_PULL_CAP
            for o in act:                                     # separation: drones stay independent
                rr = dist(d["x"], d["y"], o["x"], o["y"])
                if o["id"] != d["id"] and 0 < rr < SEP_DIST:
                    tx += (d["x"] - o["x"]) / rr * SEP_GAIN * (SEP_DIST - rr) / SEP_DIST
                    ty += (d["y"] - o["y"]) / rr * SEP_GAIN * (SEP_DIST - rr) / SEP_DIST
            if k is not None:                                 
                sv = survs[k]; sd = max(dist(d["x"], d["y"], sv["x"], sv["y"]), 0.01)
                pull = 1.05 if sv["moving"] else 1.35
                tx += (sv["x"] - d["x"]) / sd * pull; ty += (sv["y"] - d["y"]) / sd * pull
            else:                                            
                for j, sv in enumerate(survs):
                    rr = max(dist(d["x"], d["y"], sv["x"], sv["y"]), 0.01)
                    if crowd[j] > 0 and not sv["marked"] and rr < COVER_DIST:
                        tx += (d["x"] - sv["x"]) / rr * COVER_GAIN * (COVER_DIST - rr) / COVER_DIST
                        ty += (d["y"] - sv["y"]) / rr * COVER_GAIN * (COVER_DIST - rr) / COVER_DIST
                if dist(d["x"], d["y"], d["wx"], d["wy"]) < 1.0:
                    cands = [rand_pt(r) for _ in range(4)]
                    d["wx"], d["wy"] = max(cands, key=lambda c: min([dist(c[0], c[1], o["x"], o["y"]) for o in act if o["id"] != d["id"]] or [9]))
                wd = max(dist(d["x"], d["y"], d["wx"], d["wy"]), 0.01)
                tx += (d["wx"] - d["x"]) / wd * 0.75; ty += (d["wy"] - d["y"]) / wd * 0.75
            tx += ALPHA * (r.random() - 0.5); ty += ALPHA * (r.random() - 0.5)
            self._move(d, self._safe_drone_step(d, tx, ty))

        for s in survs:                                       # geo-tagging: only a standing survivor can be marked
            s["followers"] = sum(1 for d in act if dist(d["x"], d["y"], s["x"], s["y"]) < 1.5) if s["moving"] else 0
            if not s["marked"] and not s["moving"] and any(dist(d["x"], d["y"], s["x"], s["y"]) < FOUND_DIST for d in act):
                s["marked"], s["mark_pos"] = True, (round(s["x"], 2), round(s["y"], 2))
                s["coord_log"].append((gen, *s["mark_pos"]))
        self.gen += 1

    def _assign_targets(self, act, survs, chase, comm, quota_max):
        """One target per drone, from what it (or a drone within comm range) senses. Greedy nearest pairing in rounds:
        round 1 = one drone per survivor, later rounds add guards up to quota_max; surplus drones explore."""
        elig = [k for k, sv in enumerate(survs) if not sv["marked"] and (chase or not sv["moving"])]
        if not elig or not act: return {}
        known = {}
        for d in act:
            mates = [o for o in act if dist(d["x"], d["y"], o["x"], o["y"]) <= comm]
            for k in elig:
                sv = survs[k]
                if min(dist(o["x"], o["y"], sv["x"], sv["y"]) for o in mates) < SENSE_RANGE:
                    known[(d["id"], k)] = dist(d["x"], d["y"], sv["x"], sv["y"])
        assign, load = {}, {k: 0 for k in elig}
        for rnd in range(quota_max):
            for v, did, k in sorted((v, did, k) for (did, k), v in known.items() if did not in assign and load[k] == rnd):
                if did not in assign and load[k] == rnd: assign[did] = k; load[k] += 1
        return assign

    def marked_count(self): return sum(1 for s in self.survs if s["marked"])
    def fleet_stats(self):
        c = lambda st, alive=False: sum(1 for d in self.drones if d["status"] == st and (d["alive"] or not alive))
        return c("active", True), c("standby"), c("returning", True), c("parked")
    def phase_text(self):
        if self.all_parked: return "MISSION COMPLETE: Swarm recovered at base."
        if self.rtb: return "ALL TARGETS MARKED: Swarm returning to base (RTB Active)..."
        if self.static_pending and self.gen < PHASE_SWITCH: return "Phase 1: Searching for static survivors..."
        if any(s["moving"] for s in self.survs): return "Phase 2: Recording drift tracks & following survivors..."
        return "Drifting stabilized: locking final coordinates..."

# ================================== TKINTER GUI ==================================
def label(parent, text="", size=8, bold=False, fg=C_MUTED, bg=C_PANEL, mono=False):
    return tk.Label(parent, text=text, font=("Consolas" if mono else "Segoe UI", size, "bold" if bold else "normal"), fg=fg, bg=bg)
def button(parent, text, cmd, bg=C_BG, fg=C_TEXT, size=8, **kw):
    return tk.Button(parent, text=text, command=cmd, font=("Segoe UI", size, "bold"), fg=fg, bg=bg, bd=1, relief="flat", cursor="hand2", **kw)
class TacticalRadarApp:
    def __init__(self, root):
        self.root = root
        root.title("Tactical Firefly Swarm SAR - EO-CC-HFA"); root.geometry("1380x880"); root.minsize(1150, 750); root.configure(bg=C_BG)
        self.running, self.last_step_time, self.step_interval, self.last_telemetry_time = True, time.time(), 0.11, 0
        self.var_seed, self.var_comm = tk.IntVar(value=7), tk.DoubleVar(value=4.0)
        self.var_drones, self.var_surv, self.var_dyn = tk.IntVar(value=16), tk.IntVar(value=8), tk.IntVar(value=4)
        self.var_battery = tk.StringVar(value="Standard Pack (250) - 50% Relay")
        self.var_fixed, self.var_moving = tk.BooleanVar(value=True), tk.BooleanVar(value=True)
        self.var_new, self.var_drop, self.var_manual = tk.BooleanVar(), tk.BooleanVar(), tk.BooleanVar()
        self.sim, self.card_widgets = None, []; self._build_ui(); self.rebuild_sim(); root.after(16, self._loop)

    def _panel(self, parent, **pack):
        f = tk.Frame(parent, bg=C_PANEL, highlightbackground=C_BORDER, highlightthickness=1); f.pack(**pack); return f

    def _build_ui(self):
        box = tk.Frame(self.root, bg=C_BG); box.pack(fill="both", expand=True, padx=12, pady=12)
        hin = tk.Frame(self._panel(box, fill="x", pady=(0, 8)), bg=C_PANEL); hin.pack(fill="x", padx=14, pady=6)
        left = tk.Frame(hin, bg=C_PANEL); left.pack(side="left")
        label(left, "FIREFLY SWARM SAR - HYBRID ALGORITHM", 11, True, C_TEXT).pack(side="left", padx=(0, 10))
        self.relay_pill = label(left, "", 8, True, C_PRIMARY, C_BG, True)
        self.relay_pill.config(highlightthickness=1, padx=8, pady=2); self.relay_pill.pack(side="left")

        act = tk.Frame(hin, bg=C_PANEL); act.pack(side="right")
        self.btn_pause = button(act, "Pause", self.toggle_pause, C_PRIMARY, "#FFF", 9, padx=12, pady=3)
        self.btn_pause.pack(side="left", padx=3)
        for txt, cmd in [("Step", self.do_step), ("Restart", self.rebuild_sim), ("New Seed", self.do_new_seed)]:
            button(act, txt, cmd, size=9, padx=10, pady=3).pack(side="left", padx=3)

        hud = self._panel(box, fill="x", pady=(0, 8))
        hud_in = tk.Frame(hud, bg=C_PANEL); hud_in.pack(fill="x", padx=14, pady=6)
        self.lbl_phase = label(hud_in, "", 9, True, C_TEXT); self.lbl_phase.pack(side="left")
        self.lbl_metrics = label(hud_in, "", 9, True, C_PRIMARY, mono=True); self.lbl_metrics.pack(side="right")

        grid = tk.Frame(box, bg=C_BG); grid.pack(fill="both", expand=True)
        self.canvas = tk.Canvas(self._panel(grid, side="left", fill="both", expand=True, padx=(0, 8)),
                                bg=C_PANEL, bd=0, highlightthickness=0, cursor="crosshair")
        self.canvas.pack(fill="both", expand=True, padx=10, pady=10)
        self.canvas.bind("<Button-1>", self.on_canvas_click)
        side = tk.Frame(grid, bg=C_BG, width=390); side.pack(side="right", fill="y"); side.pack_propagate(False)

        c1 = self._mk_card(side, "FLEET & BATTERY SETUP")
        self._add_slider(c1, "Fleet Size", self.var_drones, 2, 32, 2)
        label(c1, "Battery Budget").pack(anchor="w")
        opt = tk.OptionMenu(c1, self.var_battery, "Standard Pack (250) - 50% Relay", "Critical Pack (120) - 50% Relay",
                            "Unlimited Range", command=lambda _: self.rebuild_sim())
        opt.config(bg=C_CARD, fg=C_TEXT, bd=1, relief="solid", font=("Segoe UI", 8)); opt.pack(fill="x", pady=2)
        self._add_slider(c1, "Total Survivors", self.var_surv, 1, 25, 1)
        self.dyn_scale = self._add_slider(c1, "Drifting Survivors", self.var_dyn, 0, 25, 1)
        self._add_slider(c1, "RF Comm Range", self.var_comm, 1.0, 10.0, 0.5, True)

        c2 = self._mk_card(side, "ENVIRONMENT & HAZARDS")
        hg = tk.Frame(c2, bg=C_PANEL); hg.pack(fill="x")
        for i, (txt, var) in enumerate([("Fixed No-Fly", self.var_fixed), ("Dynamic Hazard", self.var_moving),
                                       ("Zone (Gen 50)", self.var_new), ("Comm Drop (Gen 60)", self.var_drop),
                                       ("Manual Hold (Click / Toggle)", self.var_manual)]):
            tk.Checkbutton(hg, text=txt, variable=var, font=("Segoe UI", 8), fg=C_TEXT, bg=C_PANEL, activebackground=C_PANEL,
                           command=self.rebuild_sim).grid(row=i // 2, column=i % 2, columnspan=2 if i == 4 else 1, sticky="w", padx=2)

        c3 = self._mk_card(side, "TARGET DIRECTORY & LOGS", expand=True)
        bb = tk.Frame(c3, bg=C_PANEL); bb.pack(fill="x", pady=(0, 4))
        button(bb, "Resume", lambda: (self.sim.resume_all(), self._update_telemetry(True)), padx=8).pack(side="right", padx=(2, 0))
        button(bb, "Stop All", lambda: (self.sim.stop_all(), self._update_telemetry(True)), padx=8).pack(side="right")
        
        sb = ttk.Scrollbar(c3, orient="vertical"); sb.pack(side="right", fill="y")
        self.tgt_cv = tk.Canvas(c3, bg=C_PANEL, bd=0, highlightthickness=0, yscrollcommand=sb.set)
        self.tgt_cv.pack(side="left", fill="both", expand=True)
        sb.config(command=self.tgt_cv.yview)
        self.tgt_box = tk.Frame(self.tgt_cv, bg=C_PANEL)
        win = self.tgt_cv.create_window((0, 0), window=self.tgt_box, anchor="nw")
        self.tgt_box.bind("<Configure>", lambda e: self.tgt_cv.configure(scrollregion=self.tgt_cv.bbox("all")))
        self.tgt_cv.bind("<Configure>", lambda e: self.tgt_cv.itemconfigure(win, width=e.width))
        for ev in ("<MouseWheel>", "<Button-4>", "<Button-5>"):        
            self.root.bind_all(ev, lambda e: self.tgt_cv.yview_scroll(-1 if (e.num == 4 or e.delta > 0) else 1, "units"))

    def _mk_card(self, parent, title, expand=False):
        in_c = tk.Frame(self._panel(parent, fill="both" if expand else "x", expand=expand, pady=(0, 8)), bg=C_PANEL)
        in_c.pack(fill="both" if expand else "x", expand=expand, padx=10, pady=8)
        label(in_c, title, 8, True, C_PRIMARY).pack(anchor="w", pady=(0, 4))
        return in_c

    def _add_slider(self, parent, text, var, f, t, st, is_float=False):
        row = tk.Frame(parent, bg=C_PANEL); row.pack(fill="x", pady=2)
        h = tk.Frame(row, bg=C_PANEL); h.pack(fill="x")
        label(h, text).pack(side="left")
        val_l = label(h, "", 8, True, C_TEXT, mono=True); val_l.pack(side="right")
        fmt = lambda v: f"{v:.1f} km" if is_float else str(v)

        def chg(v):
            val = float(v) if is_float else int(float(v))
            var.set(val); val_l.config(text=fmt(val)); self.rebuild_sim()

        s = tk.Scale(row, from_=f, to=t, resolution=st, orient="horizontal", showvalue=0, bg=C_PANEL, fg=C_PRIMARY,
                     troughcolor=C_BG, highlightthickness=0, bd=0, command=chg)
        s.set(var.get()); val_l.config(text=fmt(var.get())); s.pack(fill="x")
        s.val_label = val_l
        return s

    def _style_pause(self):
        self.btn_pause.config(text="Pause" if self.running else "Play", bg=C_PRIMARY if self.running else C_BG,
                              fg="#FFF" if self.running else C_TEXT)

    def toggle_pause(self):
        self.running = not self.running; self._style_pause(); self.last_step_time = time.time()

    def do_step(self):
        self.running = False; self._style_pause(); self.sim.step(); self._update_telemetry(True)
    def do_new_seed(self): self.var_seed.set(random.randint(0, 1000000)); self.rebuild_sim()
    def _toggle_surv(self, sid): self.sim.toggle_stop(sid); self._update_telemetry(True)

    def rebuild_sim(self):
        n_surv = self.var_surv.get()
        n_dyn = min(self.var_dyn.get(), n_surv)
        self.dyn_scale.config(to=max(1, n_surv))                 
        if n_dyn != self.var_dyn.get():
            self.var_dyn.set(n_dyn); self.dyn_scale.set(n_dyn); self.dyn_scale.val_label.config(text=str(n_dyn))
        seed = self.var_seed.get()
        bs = self.var_battery.get()
        batt = 250 if "250" in bs else (120 if "120" in bs else None)
        self.sim = Simulation(n_surv=n_surv, n_dyn=n_dyn, n_drones=self.var_drones.get(), comm=self.var_comm.get(), battery=batt,
                              fixed_zone=self.var_fixed.get(), moving_zone=self.var_moving.get(), new_zone=self.var_new.get(),
                              comm_drop=self.var_drop.get(), manual_stop=self.var_manual.get(), seed=seed)
        col = C_PRIMARY if batt else C_GREEN
        self.relay_pill.config(text=f"RELAY ACTIVE ({batt} BATT)" if batt else "UNLIMITED RANGE", fg=col, highlightbackground=col)
        self.last_step_time = time.time(); self._init_target_cards(); self._update_telemetry(True)
    def _scr(self, x, y):
        w, h, pad = self.canvas.winfo_width(), self.canvas.winfo_height(), 36
        return pad + x / AREA * (w - 2 * pad), pad + (AREA - y) / AREA * (h - 2 * pad)
    def on_canvas_click(self, ev):                              
        x0, y0 = self._scr(0, 0); x1, y1 = self._scr(AREA, AREA)
        if x1 <= x0 or y0 <= y1: return
        ax, ay = (ev.x - x0) / (x1 - x0) * AREA, (y0 - ev.y) / (y0 - y1) * AREA
        cands = [s for s in self.sim.survs if s["dyn"] and not s["marked"] and dist(s["x"], s["y"], ax, ay) < 0.8]
        if cands: self._toggle_surv(min(cands, key=lambda s: dist(s["x"], s["y"], ax, ay))["id"])
    def _init_target_cards(self):
        for w in self.tgt_box.winfo_children(): w.destroy()
        self.card_widgets = []
        for s in self.sim.survs:
            card = tk.Frame(self.tgt_box, bg=C_CARD, highlightbackground=C_BORDER, highlightthickness=1)
            card.pack(fill="x", pady=2, padx=2)
            r1 = tk.Frame(card, bg=C_CARD); r1.pack(fill="x", padx=6, pady=(3, 1))
            label(r1, f"S{s['id']}", 8, True, C_TEXT, C_CARD).pack(side="left")
            st_lbl = label(r1, "", 7, bg=C_CARD); st_lbl.pack(side="right")
            r2 = tk.Frame(card, bg=C_CARD); r2.pack(fill="x", padx=6, pady=(0, 3))
            c_lbl = label(r2, "", 8, True, bg=C_CARD, mono=True); c_lbl.pack(side="left")
            btn = button(r2, "Stop", (lambda sid=s["id"]: self._toggle_surv(sid)) if s["dyn"] else None, "#FFF", size=7, padx=4, pady=0)
            btn.config(relief="solid")
            self.card_widgets.append({"card": card, "st": st_lbl, "c": c_lbl, "btn": btn, "pts": label(r2, "", 7, bg=C_CARD)})

    def _update_telemetry(self, force=False):
        now, sim = time.time(), self.sim
        if not force and now - self.last_telemetry_time < 0.12: return
        self.last_telemetry_time = now
        col = C_GREEN if sim.all_parked else (C_AMBER if sim.rtb else None)
        self.lbl_phase.config(text=sim.phase_text(), fg=col or C_TEXT)
        st = sim.fleet_stats()
        self.lbl_metrics.config(text=f"GEN {sim.gen} | ACTIVE {st[0]} | STANDBY {st[1]} | RETURNING {st[2]} | SWAPS {sim.swaps} | MARKED {sim.marked_count()}/{len(sim.survs)}")
        for s, cw in zip(sim.survs, self.card_widgets):
            xy = f"({s['x']:.1f}, {s['y']:.1f})"
            if s["marked"]: bd, st_t, st_c, c_t, c_c, b_t = C_GREEN, "✓ Confirmed Lock", C_GREEN, f"FINAL: ({s['mark_pos'][0]:.1f}, {s['mark_pos'][1]:.1f})", C_GREEN, None
            elif s["dyn"] and s["moving"]: bd, st_t, st_c, c_t, c_c, b_t = C_BORDER, f"Drifting ({s['followers']} drn)", C_AMBER, f"REC: {xy}", C_PRIMARY, "Stop"
            elif s["dyn"]: bd, st_t, st_c, c_t, c_c, b_t = C_BORDER, "Halted / Waiting", C_BLUE, f"HALT: {xy}", C_BLUE, "Resume"
            else: bd, st_t, st_c, c_t, c_c, b_t = C_BORDER, "Static Target", C_MUTED, f"STATIC: {xy}", C_TEXT, None
            cw["card"].config(highlightbackground=bd); cw["st"].config(text=st_t, fg=st_c); cw["c"].config(text=c_t, fg=c_c)
            if cw.get("bt") != b_t:
                cw["bt"] = b_t; cw["btn"].pack_forget(); cw["pts"].pack_forget()
                if b_t: cw["btn"].config(text=b_t); cw["btn"].pack(side="right", padx=(3, 0)); cw["pts"].pack(side="right")
            if b_t: cw["pts"].config(text=f"{len(s['coord_log'])} pts", fg=C_AMBER if s["moving"] else C_MUTED)

    def _badge(self, x, y, text, fg, bg, bd):
        cv, pw = self.canvas, len(text) * 5.8 + 10
        cv.create_rectangle(x - pw / 2, y - 7, x + pw / 2, y + 7, fill=bg, outline=bd)
        cv.create_text(x, y, text=text, fill=fg, font=("Consolas", 8, "bold"))

    def _draw_arena(self, p):
        cv, sim, scr = self.canvas, self.sim, self._scr
        cv.delete("all")
        if cv.winfo_width() < 120 or cv.winfo_height() < 120: return
        pos = lambda o: (lerp(o["prev_x"], o["x"], p), lerp(o["prev_y"], o["y"], p))
        for i in range(0, 11, 2):
            gx, gy0 = scr(i, 0); _, gy1 = scr(i, 10); gx0, gy = scr(0, i); gx1, _ = scr(10, i)
            cv.create_line(gx, gy0, gx, gy1, fill=C_GRID); cv.create_text(gx, gy0 + 12, text=str(i), fill=C_MUTED, font=("Consolas", 8))
            cv.create_line(gx0, gy, gx1, gy, fill=C_GRID); cv.create_text(gx0 - 12, gy, text=str(i), fill=C_MUTED, font=("Consolas", 8))
        scale = scr(AREA, 0)[0] - scr(0, 0)[0]
        for o in sim.active_zones():
            sx, sy = scr(*pos(o)); rad = o["r"] * scale / AREA
            cv.create_oval(sx - rad, sy - rad, sx + rad, sy + rad, fill="#FEE2E2", outline=C_RED, width=1.5, dash=(4, 3))
            self._badge(sx, sy, "HAZARD ZONE", "#B91C1C", "#FEE2E2", C_RED)
        bx, by = scr(*BASE); st = sim.fleet_stats()
        cv.create_oval(bx - 14, by - 14, bx + 14, by + 14, outline=C_PRIMARY, width=1.5)
        cv.create_rectangle(bx - 5, by - 5, bx + 5, by + 5, fill="#1E293B", outline="#1E293B")
        self._badge(bx + 64, by - 2, f"BASE [STB:{st[1]}|DCK:{st[3]}]", C_TEXT, "#FFF", C_BORDER)

        if not sim.rtb:                                           
            for s in sim.survs:
                if s["moving"] and not s["marked"]:
                    sx, sy = scr(*pos(s))
                    for d in sim.drones:
                        if d["alive"] and d["status"] == "active" and dist(d["x"], d["y"], s["x"], s["y"]) < 1.5:
                            cv.create_line(*scr(*pos(d)), sx, sy, fill=C_AMBER, dash=(3, 3))
        trails = [(s, "#FCD34D", 2) for s in sim.survs if s["dyn"] and s["moving"]] + \
                 [(d, "#FCD34D" if d["status"] == "returning" else "#93C5FD", 1.5) for d in sim.drones
                  if d["alive"] and d["status"] in ("active", "returning")]
        for o, col, w in trails:
            pts = [scr(*q) for q in o["trail"] + [pos(o)]]
            for a, b in zip(pts, pts[1:]): cv.create_line(*a, *b, fill=col, width=w)

        az = sim.active_zones()
        for s in sim.survs:
            cx, cy = project_outside_hazards(*pos(s), az, SURV_MARGIN)
            sx, sy = scr(cx, cy); tri = (sx, sy - 7, sx - 6, sy + 5, sx + 6, sy + 5)
            if s["marked"]:
                cv.create_oval(sx - 10, sy - 10, sx + 10, sy + 10, outline=C_GREEN, width=2)
                cv.create_rectangle(sx - 4, sy - 4, sx + 4, sy + 4, fill=C_GREEN, outline=C_GREEN)
                self._badge(sx + 50, sy - 2, f"✓ S{s['id']} FINAL:({s['mark_pos'][0]:.1f}, {s['mark_pos'][1]:.1f})", C_GREEN, "#DCFCE7", C_GREEN)
            elif s["dyn"]:
                col, bg, tag = (C_AMBER, "#FEF3C7", "REC") if s["moving"] else (C_BLUE, "#DBEAFE", "HALT")
                cv.create_polygon(*tri, fill=col, outline="")
                self._badge(sx + 50, sy - 2, f"S{s['id']} {tag}:({cx:.1f}, {cy:.1f})", col, bg, col)
            else:
                cv.create_rectangle(sx - 5, sy - 5, sx + 5, sy + 5, fill=C_MUTED, outline=C_BORDER)
                self._badge(sx + 38, sy - 2, f"S{s['id']} ({cx:.1f}, {cy:.1f})", C_TEXT, "#FFF", C_BORDER)
        for d in sim.drones:
            if d["status"] in ("standby", "parked") or not d["alive"]: continue
            dx, dy = scr(*project_outside_hazards(*pos(d), az, DRONE_MARGIN))
            cv.create_oval(dx - 5, dy - 5, dx + 5, dy + 5, fill=C_AMBER if d["status"] == "returning" else C_BLUE, outline="#FFF", width=1.5)
            cv.create_text(dx + 8, dy - 7, text=f"D{d['id']}", fill=C_TEXT, font=("Consolas", 7, "bold"))

    def _loop(self):
        now, live = time.time(), self.running and not self.sim.done
        if live and now - self.last_step_time >= self.step_interval:
            self.sim.step(); self.last_step_time = now; self._update_telemetry()
        self._draw_arena(min(1.0, (now - self.last_step_time) / self.step_interval) if live else 1.0)
        self.root.after(16, self._loop)
def main():
    root = tk.Tk()
    try:
        from ctypes import windll
        windll.shcore.SetProcessDpiAwareness(1)
    except Exception:
        pass
    TacticalRadarApp(root)
    root.mainloop()

if __name__ == "__main__":
    main()