<div align="center">

# EO-CC-HFA

### Energy-Optimized, Communication-Constrained Hybrid Firefly Algorithm for UAV Swarm Search and Rescue

**Department of Artificial Intelligence · Amrita Vishwa Vidyapeetham**
*Computational Thinking · Fall 2026 · Group 15*

**Team Members (Group 15):** Grandhi Charan · Dhananjeyan S · Ashwanth S · Adityanath Jha

**Supervisor:** Dr. Kalai Vani S.S, Assistant Professor

</div>

---

## Table of Contents

1. [About the Project](#1-about-the-project)
2. [The Firefly Algorithm](#2-the-firefly-algorithm)
3. [How the Firefly Algorithm Works in Our Project](#3-how-the-firefly-algorithm-works-in-our-project)
4. [Supporting Mechanisms](#4-supporting-mechanisms)
5. [Results](#5-results)
6. [Getting Started](#6-getting-started)
7. [Repository Structure](#7-repository-structure)
8. [Limitations and Future Work](#8-limitations-and-future-work)
9. [Team](#9-team)
10. [References](#10-references)
11. [License](#11-license)
12. [Citation](#12-citation)

---

## 1. About the Project

After a disaster, a swarm of drones can search a large area much faster than a single drone, but only if the drones agree on who searches where. This project builds a **decentralised swarm** that finds survivors without a central controller, under three realistic constraints:

| Constraint | How it is modelled |
|---|---|
| **Limited communication** | A drone only learns about survivors that it, or a drone inside its radio range, senses. The radio range can drop to 40 % during a mission. |
| **Limited energy** | Every unit of flight costs battery. Drones return to recharge and spare drones take over (battery relay). |
| **Dangerous environment** | Fixed, moving and late-appearing hazard zones that drones must never enter. |

Survivors are either **static** or **drifting**. Drifting survivors are tracked until they stop; a survivor is **marked** (geo-tagged) once a drone reaches it while it is standing still. When every survivor is marked, the swarm returns to base.

The project contains the program (`FA_final.py`), a full technical report, and an interactive website that explains and runs the algorithm in the browser.

---

## 2. The Firefly Algorithm

The Firefly Algorithm (FA) was proposed by Xin-She Yang (2008). It is a nature-inspired optimiser based on the behaviour of fireflies: a firefly is attracted to brighter fireflies, and the attraction gets weaker with distance.

Three rules define it:

1. Every firefly has a **brightness** `I` determined by the objective function.
2. A less bright firefly moves towards a brighter one.
3. Attractiveness decreases with distance.

For two fireflies `i` and `j` at distance $r_{ij}$ the attractiveness and the movement are

$$\beta(r) = \beta_0\, e^{-\gamma r^{2}}$$

$$p_i \leftarrow p_i + \beta_0\, e^{-\gamma r_{ij}^{2}}\,(p_j - p_i) + \alpha\left(\varepsilon - \tfrac{1}{2}\right)$$

where $\beta_0$ is the attractiveness at distance 0, $\gamma$ is the light-absorption coefficient, $\alpha$ scales a random term and $\varepsilon$ is uniform on $[0,1)$.

**Why FA suits a drone swarm.** Drones are naturally "fireflies": each has a position, can be given a score, and can be drawn towards better-placed neighbours using only local information. This makes FA a good fit for a decentralised, communication-limited swarm.

**Where the original FA is weak for search and rescue.** In the original algorithm all fireflies are attracted to the single brightest one. For a swarm this means (a) all drones crowd onto the same survivor, (b) the algorithm has no notion of radio range, battery or forbidden areas, and (c) the user has to re-tune $\gamma$ whenever the problem scale changes. EO-CC-HFA addresses each of these.

---

## 3. How the Firefly Algorithm Works in Our Project

In our simulator every UAV is a firefly. One call of `step()` is one generation. The Firefly part works in four steps.

### Step 1: Each drone gets its own target

A drone knows survivor *k* only if it, or a drone inside its radio range $R_c$, is within the sensing range $R_s = 4$:

$$k \in K_i \iff \min_{j \in \mathcal{N}_i}\lVert p_j - s_k\rVert < R_s$$

Drones are then paired with survivors by a greedy nearest-pair rule, in rounds, with at most $q$ guards per survivor:

$$q = \max\left(1,\ \min\left(3,\ \left\lfloor \frac{n_{\text{active}}}{n_{\text{unmarked}}} \right\rfloor\right)\right)$$

Drones left without a target explore. This stops the whole swarm from collapsing onto one survivor.

### Step 2: Brightness of a drone

Brightness plays the role of the objective function. A drone is bright when it is close to its target, cheap to bring home, and outside every hazard:

$$I_i = \frac{100}{1 + d_{i,\text{target}}} - w_e \cdot 0.15 \cdot d_{i,\text{base}} - 500 \cdot \mathbf{1}[\,p_i \in \text{hazard}\,]$$

with energy weight $w_e = 0.5$. Dead, inactive or trapped drones get $-10^{9}$ so that nobody follows them.

### Step 3: Attraction

Drone *i* is attracted to drone *j* only if **all** of the following hold:

- *j* is active and brighter than *i*,
- *j* is working on the **same target** as *i*,
- *j* is inside the radio range $R_c$.

The attractiveness keeps the Gaussian decay of the original FA, with a factor ½:

$$\beta_{ij} = \tfrac{1}{2}\,\beta_0\, e^{-\gamma\, r_{ij}^{2}}, \qquad \gamma = \frac{\ln 2}{R_c^{2}}$$

**The choice of $\gamma$ is the key adaptation.** Because $\gamma$ is derived from the radio range, the attraction at the edge of the radio range is always exactly half of its value at distance zero, whatever $R_c$ is. $\gamma$ no longer needs hand tuning. With $R_c = 4$: $\gamma = 0.0433$, and $\beta$ falls from 0.5 (at 0) to 0.25 (at the radio edge).

The pull on drone *i* is the sum over its attractors, capped so a large cluster can never outvote the drone's own sensing:

$$F_i = \sum_{j \in A_i} \beta_{ij}\,(p_j - p_i), \qquad \lVert F_i \rVert \le 1.0$$

### Step 4: The movement

The pull is combined with the other influences into one move, then limited to the maximum step $\Delta = 0.45$:

$$v_i = F_i + S_i + G_i + J_i \quad \text{(drone with a target)}$$

| Term | Meaning | Strength |
|---|---|---|
| $F_i$ | Firefly attraction (capped) | ≤ 1.0 |
| $S_i$ | Separation from drones closer than 0.8 | up to 0.9 |
| $G_i$ | Pursuit of the assigned target | 1.35 (static) or 1.05 (moving) |
| $J_i$ | Random jitter $\alpha(\varepsilon - \tfrac12)$, $\alpha = 0.25$ | ±0.125 per axis |

A drone without a target uses separation, a push away from survivors that are already covered, a pull of 0.75 towards an exploration waypoint, and jitter. The waypoint is the best of four random candidates, the one farthest from the nearest other drone.

### Pseudocode

```text
for each generation:
    move survivors; update hazards (grow, move, slide)
    choose phase (static first, drifting from generation 60) and radio range R_c
    gamma = ln(2) / R_c^2
    assign one target to every active drone              # radio-limited knowledge
    battery relay: recall low drones, recharge, relaunch spares
    for each drone: compute brightness I
    for each active drone i:
        F = sum over brighter drones j on the same target within R_c of
            0.5 * beta0 * exp(-gamma * r_ij^2) * (p_j - p_i)       # Firefly attraction
        F = min(1, 1/|F|) * F                                      # pull cap
        v = F + separation + pursuit (or exploration) + jitter
        move i by min(|v|, 0.45) with the hazard-safe step
    mark every standing survivor that an active drone is within 0.55 of
    if all survivors are marked: swarm returns to base
```

### Original FA compared with EO-CC-HFA

| Aspect | Original FA | EO-CC-HFA (this project) |
|---|---|---|
| Attracted to | Every brighter firefly | Brighter drones on the **same target** only |
| Range of attraction | Whole population | Within the **radio range** $R_c$ |
| Decay coefficient $\gamma$ | Chosen by hand | $\gamma = \ln 2 / R_c^2$ |
| Brightness | Objective value | Target proximity, energy cost, hazard penalty |
| Total pull | Unbounded | Capped at 1.0 |
| Obstacles | Not handled | Potential field, hard projection, stagnation escape |
| Energy | Not handled | Return rule, recharging, spare drones |
| Targets | One optimum | One target per drone, up to 3 guards each |

---

## 4. Supporting Mechanisms

The Firefly attraction is one part of a larger design. The following mechanisms make the swarm safe and long-lasting.

- **Hazard-safe motion.** A potential field pushes drones away from hazards (strength $3.6\left(\frac{r_d - d}{r_d - \rho}\right)^2$ inside the band $r_d = \rho + 0.28 + 1.35$) with a sideways swerve; a hard projection guarantees that no drone ends inside a hazard; a stagnation escape frees a drone that has made no progress for 20 generations.
- **Hazard dynamics.** Fixed, moving (bouncing, and sliding around the base and standing survivors so a mark is never lost) and late hazards that appear at generation 50 and grow over 12 generations.
- **Battery relay.** Flight costs 1.5 per unit. A drone returns when $E < 2\,d_{\text{base}} + 16 + 2\,|Z|$, recharges at 6 per generation, and spare drones are launched towards the least-covered survivor. With a battery limit, half of the fleet flies at a time.
- **Phases.** Static survivors are served first; drifting survivors are tracked from generation 60 at the latest and marked once they stop.
- **Determinism.** One seeded random generator drives everything, so the same settings and seed always give the same mission.

---

## 5. Results

All values were measured by running this code (100 seeds per point unless stated). Default scenario: 16 drones (8 flying, 8 in reserve), 8 survivors (4 drifting), radio range 4, battery 250, one fixed and one moving hazard.

| Experiment | Result |
|---|---|
| Default mission | **100 / 100** completed; last survivor marked at generation **89.7 ± 6.3**; fleet docked at 117.4 |
| Scaling with survivors (2 to 25) | Last mark rises only from 73.3 to 93.9; all missions completed |
| Battery relay, 400-generation mission (50 seeds) | With relay: **50 / 50** completed, no drone lost. Without relay: **0 / 50**, all 8 flying drones lost |
| Environment (static, fixed, fixed + moving, all + radio drop) | 100 / 100 in every case; last mark 87.6 to 89.7 |
| Robustness, 1,600 random configurations | **1,600 / 1,600** completed |

**Honest finding.** Ablation tests show that in these scenarios the Firefly attraction, the radio range and the energy weight have little measurable effect on mission time. The assignment rule, the pursuit force, the sensing range and the battery relay are what drive the results. The Firefly framework supplies the structure of the method (brightness, range-limited attraction, the $\gamma$ rule); a harder problem in which many drones have no target is needed to test the attraction more strongly. See the report for the full discussion.

---

## 6. Getting Started

### Requirements

- Python 3.8 or newer with **Tkinter** (included with most Python installers)
- No third-party packages

### Run the simulator

```bash
git clone https://github.com/<your-username>/<your-repository>.git
cd <your-repository>
python FA_final.py
```

The window shows the arena, tables of survivors and drones, and an event log. Controls: fleet size, number of survivors and drifters, radio range, battery size, and four environment switches (fixed hazard, moving hazard, late hazard, radio drop).

### Run without a window

Importing the file needs Tkinter to be importable. On a machine without it, stub it first:

```python
import sys, types
tk = types.ModuleType("tkinter"); tk.ttk = types.ModuleType("tkinter.ttk")
sys.modules["tkinter"] = tk; sys.modules["tkinter.ttk"] = tk.ttk

import FA_final as F

s = F.Simulation(n_drones=16, n_surv=8, n_dyn=4, seed=7)
while not s.done and s.gen < 2000:
    s.step()
print(s.gen, s.marked_count(), "of", len(s.survs), "marked")
```

`Simulation` accepts `n_surv`, `n_dyn`, `n_drones`, `comm`, `battery` (`None` for unlimited), `fixed_zone`, `moving_zone`, `new_zone`, `comm_drop`, `manual_stop` and `seed`.

### Interactive website

Open `EO-CC-HFA_Interactive_Guide.html` in a browser (no internet needed). It has a live simulator that reproduces the Python runs for the same settings and seed, interactive plots of the formulas, and the full source with an explanation of each part. To publish it, enable **GitHub Pages** for the repository and use the file as `index.html`.

---

## 7. Repository Structure

```text
.
├── FA_final.py                        # Simulation engine and Tkinter interface
├── EO-CC-HFA_Final_Report.docx        # Full technical report
├── EO-CC-HFA_Interactive_Guide.html   # Standalone interactive website
├── README.md
└── LICENSE
```

---

## 8. Limitations and Future Work

**Limitations:** two-dimensional arena with circular hazards; perfect sensing and radio inside their ranges; greedy, not optimal, assignment; hazard positions known to the drones; a simple energy model; simulation only (no hardware flight); time measured in generations, not seconds.

**Future work:** three-dimensional flight; noisy sensing and lossy radio; optimal assignment with the Hungarian method; a harder test for the attraction term; sensed hazards of other shapes; software-in-the-loop tests with a flight simulator.

---

## 9. Team

**Department of Artificial Intelligence, Amrita Vishwa Vidyapeetham**
Course: Computational Thinking, Fall 2026 · Group 15

| Name | Roll Number | Contribution |
|---|---|---|
| Grandhi Charan | CB.AI.U4AAR26020 | *[add contribution]* |
| Dhananjeyan S | CB.AI.U4AAR26018 | *[add contribution]* |
| Ashwanth S | CB.AI.U4AAR26009 | *[add contribution]* |
| Adityanath Jha | CB.AI.U4AAR26005 | *[add contribution]* |

**Supervisor:** Dr. Kalai Vani S.S, Assistant Professor, Department of Artificial Intelligence, Amrita Vishwa Vidyapeetham

---

## 10. References

1. M. H. Lafta, "Path Scheduling and Target Trajectory Optimization in UAVs Based on Dragonfly and Firefly Algorithm," *Advances in Engineering and Intelligence Systems*, vol. 1, no. 3, pp. 66–80, 2022. doi:10.22034/aeis.2022.356205.1037
2. G. Airlangga et al., "Adaptive Path Planning for Multi-UAV Systems in Dynamic 3D Environments: A Multi-Objective Framework," *Designs*, vol. 8, no. 6, art. 136, 2024. doi:10.3390/designs8060136
3. M. N. Ab Wahab et al., "Optimised Path Planning Using Enhanced Firefly Algorithm for a Mobile Robot," *PLOS ONE*, vol. 19, no. 8, e0308264, 2024. doi:10.1371/journal.pone.0308264
4. X. Fan, H. Li, Y. Chen and D. Dong, "UAV Swarm Search Path Planning Method Based on Probability of Containment (POC)," *Drones*, vol. 8, no. 4, art. 132, 2024. doi:10.3390/drones8040132
5. A. Ait Saadi et al., "UAV Path Planning Using Optimization Approaches: A Survey," *Archives of Computational Methods in Engineering*, 2022. doi:10.1007/s11831-022-09742-7
6. S. Das and P. Roy, "Path Planning of Unmanned Aerial Systems in a Static 3D Environment Using Firefly Algorithm F1-3 with B-Spline," *Transactions of the Institute of Measurement and Control*, vol. 48, no. 11, pp. 2653–2665, 2026. doi:10.1177/01423312251355291
7. S. Zeng et al., "Firefly Forest: A Swarm Iteration-Free Swarm Intelligence Clustering Algorithm," *Journal of King Saud University – Computer and Information Sciences*, vol. 36, no. 9, art. 102219, 2024. doi:10.1016/j.jksuci.2024.102219
8. M. G. Bindu and S. Malathi, "Explainable Feature Selection Using Improved Firefly Algorithm with Population Diversification and Stagnation-Aware Mechanisms," *Discover Computing*, vol. 29, art. 149, 2026. doi:10.1007/s10791-026-10041-z
9. X.-S. Yang, *Nature-Inspired Metaheuristic Algorithms*, Luniver Press, 2008.
10. Python Software Foundation, "tkinter — Python interface to Tcl/Tk," Python 3 Documentation. https://docs.python.org/3/library/tkinter.html

---

## 11. License

This project is released under the **MIT License**. Add a file named `LICENSE` with the text below.

```text
MIT License

Copyright (c) 2026 Group 15, Department of Artificial Intelligence,
Amrita Vishwa Vidyapeetham

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

---

## 12. Citation

```bibtex
@misc{eocchfa2026,
  title  = {EO-CC-HFA: Energy-Optimized, Communication-Constrained Hybrid Firefly Algorithm for UAV Swarm Search and Rescue},
  author = {Grandhi Charan and Dhananjeyan S and Ashwanth S and Adityanath Jha},
  year   = {2026},
  note   = {Group 15, Department of Artificial Intelligence, Amrita Vishwa Vidyapeetham. Supervisor: Dr. Kalai Vani S.S}
}
```
