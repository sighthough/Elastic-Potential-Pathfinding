# Elastic-Potential-Pathfinding
A continuous physics-based alternative to traditional grid-search algorithms.

*Co-authored by [sighthough](https://youtu.be/UtPiUGwu-0Q) and Gemini.*

👉 **[CLICK HERE TO RUN THE LIVE DEMO](https://sighthough.github.io/Elastic-Potential-Pathfinding/)**

feel free to rip anything you want from that demo


```markdown
# 🌀 Elastic-Potential Pathfinding (EPP)

> **A continuous, physics-driven alternative to traditional graph-search algorithms (A*, Dijkstra) for dynamic path generation, obstacle avoidance, and robotic trajectory planning.**

---

## 📌 Overview

**Elastic-Potential Pathfinding (EPP)** replaces discrete graph-search algorithms with a physical simulation model. Instead of traversing grid cells or waypoints, EPP connects the start and end coordinates with an elastic particle chain. The chain is governed by three simultaneously acting physical forces:

1. **Target Attraction ($\vec{F}_a$):** Pinning mechanics pull the terminal path particle toward the target destination.
2. **Obstacle Repulsion ($\vec{F}_r$):** Obstacles generate inverse-square distance force fields that push particles out of collision zones.
3. **Internal Elasticity ($\vec{F}_s$):** Hooke's-law spring connections between adjacent particles maintain path continuity and pull the chain taut around corners.

The resulting path isn't "searched"—it **settles naturally** into the lowest potential energy state.

---

## 📐 Mechanics & Physics Model

```text
                     ATTRACTIVE FORCE (Target)
                                  │
                                  ▼
                        [ Target Point (T) ]
                                  ▲
                                  │
                        ●─────────● (Particle N-1)
                       /
                      ● (Particle N-2)
                     /
    (Elastic Force) /    ▲ (Repulsive Force F_r)
                   ● ─── ┼ ─── [ Obstacle ]
                  /
                 ● (Particle 1)
                /
      [ Start Point (S) ]

```

### 1. Spring Cohesion (Hooke's Law)

Between consecutive particles $p_i$ and $p_{i+1}$, tension maintains path structural integrity:

$$\vec{F}_s = k \cdot (\vec{x}_{i+1} - \vec{x}_i) + k \cdot (\vec{x}_{i-1} - \vec{x}_i)$$

*Where $k$ is the spring stiffness constant.*

### 2. Obstacle Repulsion (Potential Field)

For any obstacle $O_j$ with collision radius $R_o$ and influence boundary $R_{\text{rep}}$, the repulsive force vector acting on particle $p_i$ is:

$$\vec{F}_r = \begin{cases} \frac{\alpha}{(d - R_o + \epsilon)^2} \cdot \hat{d}, & d < R_o + R_{\text{rep}} \\ 0, & d \ge R_o + R_{\text{rep}} \end{cases}$$

*Where $d = \Vert{}\vec{x}_i - \vec{x}_{O_j}\Vert{}$, $\hat{d}$ is the normalized directional unit vector away from the obstacle center, $\alpha$ is the repulsion magnitude, and $\epsilon$ prevents division-by-zero.*

### 3. Damping & Velocity Integration (Verlet / Euler)

To prevent infinite oscillations around corners, kinetic energy is damped:

$$\vec{v}_i^{(t+1)} = \mu \cdot \left( \vec{v}_i^{(t)} + \frac{\vec{F}_{\text{total}}}{m} \Delta t \right)$$

$$\vec{x}_i^{(t+1)} = \vec{x}_i^{(t)} + \vec{v}_i^{(t+1)} \Delta t$$

*Where $\mu \in (0, 1)$ is the damping coefficient (typically $0.80 - 0.88$).*

---

## 🎨 Visual System Architecture

### Field Geometry & Repulsion Bubbles

```text
+-------------------------------------------------------------+
|                Repulsion Field Radius (R_rep)               |
|            . . . . . . . . . . . . . . . . . . .            |
|          .                                       .          |
|        .         +-----------------------+         .        |
|       .          |                       |          .       |
|      .           |    Solid Obstacle     |           .      |
|      .           |    (Wall / Mesh)      |           .      |
|       .          |                       |          .       |
|        .         +-----------------------+         .        |
|          .                                       .          |
|            . . . . . . . . . . . . . . . . . . .            |
+-------------------------------------------------------------+
                    ▲                     ▲
                    │  Repulsion (F_r)    │
       ───────●─────┴─────────────────────┴─────●───────
              Particle Chain Bends Outside Perimeter

```

### Traditional A* vs. Elastic-Potential Pathfinding

```text
  TRADITIONAL GRID-SEARCH (A* / Dijkstra)      ELASTIC-POTENTIAL PATHFINDING (EPP)
  +---+---+---+---+---+---+---+               +-----------------------------------+
  | S | - | - |   |   |   |   |               | (S) ●                             |
  +---+---+---+---+---+---+---+               |      \                            |
  |   |   | X | X |   |   |   |               |       ●───●                       |
  +---+---+---+---+---+---+---+               |            \   [ Obstacle ]       |
  |   |   | X | X | - | - | T |               |             ●──────────┐          |
  +---+---+---+---+---+---+---+               |                        └──● (T)   |
  (Stepped, grid-bound 90°/45° angles)        (Continuous, smooth physical trajectory)

```

### Handling Local Minima Traps (U-Shaped Obstacles)

When a particle chain encounters a concave obstacle facing the target, equal opposing forces can cause a standstill:

```text
    LOCAL MINIMA DEADLOCK                    TANGENTIAL REDIRECTION
    +-------------------+                    +-------------------+
    |     Particle      |                    |     Particle      |
    |        ●          |                    |        ●────►     |
    |     │  │  │       |                    |     │  │  \       |
    |  ==>│  │  │<==    |                    |  ==>│  │   \==>   |
    |     ▼  ▼  ▼       |                    |     ▼  ▼    \     |
    |   [  Target  ]    |                    |   [  Target  \ ]  |
    +-------------------+                    +-------------------+
    Net Force F = 0 (Stuck)                  Tangential Slide Vector Applied

```

---

## 🚀 Quickstart Base Implementations

### Option 1: Standalone JavaScript Engine Module (`ElasticPathfinder.js`)

```javascript
/**
 * Elastic-Potential Pathfinding Engine
 * Zero external dependencies. Browser & Node.js compatible.
 */
class ElasticPathfinder {
    constructor(config = {}) {
        this.numParticles = config.numParticles || 30;
        this.stiffness = config.stiffness || 0.15;
        this.damping = config.damping || 0.82;
        this.repulsionRadius = config.repulsionRadius || 100;
        this.repulsionStrength = config.repulsionStrength || 4000;
        
        this.particles = [];
        this.start = { x: 0, y: 0 };
        this.target = { x: 0, y: 0 };
        this.obstacles = [];
    }

    init(start, target, obstacles = []) {
        this.start = { ...start };
        this.target = { ...target };
        this.obstacles = obstacles;
        this.particles = [];

        for (let i = 0; i < this.numParticles; i++) {
            const t = i / (this.numParticles - 1);
            this.particles.push({
                x: this.start.x + (this.target.x - this.start.x) * t,
                y: this.start.y + (this.target.y - this.start.y) * t,
                vx: 0,
                vy: 0
            });
        }
    }

    step() {
        if (this.particles.length === 0) return;

        // Pin boundary endpoints
        this.particles[0].x = this.start.x;
        this.particles[0].y = this.start.y;
        this.particles[0].vx = 0;
        this.particles[0].vy = 0;

        const lastIdx = this.numParticles - 1;
        this.particles[lastIdx].x = this.target.x;
        this.particles[lastIdx].y = this.target.y;
        this.particles[lastIdx].vx = 0;
        this.particles[lastIdx].vy = 0;

        // Update interior nodes
        for (let i = 1; i < lastIdx; i++) {
            const p = this.particles[i];
            let fx = 0;
            let fy = 0;

            // 1. Spring forces (Hooke's Law)
            const prev = this.particles[i - 1];
            const next = this.particles[i + 1];
            fx += (prev.x - p.x) * this.stiffness + (next.x - p.x) * this.stiffness;
            fy += (prev.y - p.y) * this.stiffness + (next.y - p.y) * this.stiffness;

            // 2. Obstacle Repulsion Forces
            for (const obs of this.obstacles) {
                const dx = p.x - obs.x;
                const dy = p.y - obs.y;
                const dist = Math.hypot(dx, dy);
                const effectiveRadius = (obs.radius || 0) + this.repulsionRadius;

                if (dist < effectiveRadius && dist > 0.001) {
                    const force = this.repulsionStrength / Math.pow(dist + 10, 2);
                    
                    // Repulsive vector
                    let rfx = (dx / dist) * force;
                    let rfy = (dy / dist) * force;

                    // Tangential force component to escape local minima traps
                    const tx = -rfy * 0.25;
                    const ty = rfx * 0.25;

                    fx += rfx + tx;
                    fy += rfy + ty;
                }
            }

            // Integration & Damping
            p.vx = (p.vx + fx) * this.damping;
            p.vy = (p.vy + fy) * this.damping;
            p.x += p.vx;
            p.y += p.vy;
        }
    }

    getPath() {
        return this.particles.map(p => ({ x: p.x, y: p.y }));
    }
}

// Export for Node / ES Modules
if (typeof module !== 'undefined' && module.exports) {
    module.exports = ElasticPathfinder;
}

```

---

### Option 2: Standalone Python Module (`elastic_pathfinder.py`)

```python
import math
from typing import List, Dict, Tuple

class ElasticPathfinder:
    def __init__(self, num_particles: int = 30, stiffness: float = 0.15, 
                 damping: float = 0.82, repulsion_radius: float = 100.0, 
                 repulsion_strength: float = 4000.0):
        self.num_particles = num_particles
        self.stiffness = stiffness
        self.damping = damping
        self.repulsion_radius = repulsion_radius
        self.repulsion_strength = repulsion_strength
        
        self.start = (0.0, 0.0)
        self.target = (0.0, 0.0)
        self.particles = []
        self.obstacles = []  # List of dicts: [{'x': float, 'y': float, 'radius': float}]

    def init(self, start: Tuple[float, float], target: Tuple[float, float], obstacles: List[Dict] = None):
        self.start = start
        self.target = target
        self.obstacles = obstacles or []
        self.particles = []

        sx, sy = start
        tx, ty = target
        
        for i in range(self.num_particles):
            t = i / (self.num_particles - 1) if self.num_particles > 1 else 0
            px = sx + (tx - sx) * t
            py = sy + (ty - sy) * t
            self.particles.append({'x': px, 'y': py, 'vx': 0.0, 'vy': 0.0})

    def step(self):
        if not self.particles:
            return

        # Pin endpoints
        self.particles[0]['x'], self.particles[0]['y'] = self.start
        self.particles[0]['vx'] = self.particles[0]['vy'] = 0.0

        last_idx = self.num_particles - 1
        self.particles[last_idx]['x'], self.particles[last_idx]['y'] = self.target
        self.particles[last_idx]['vx'] = self.particles[last_idx]['vy'] = 0.0

        # Update interior particles
        for i in range(1, last_idx):
            p = self.particles[i]
            fx, fy = 0.0, 0.0

            # Spring Tension (Hooke's Law)
            prev = self.particles[i - 1]
            nxt = self.particles[i + 1]
            fx += (prev['x'] - p['x']) * self.stiffness + (nxt['x'] - p['x']) * self.stiffness
            fy += (prev['y'] - p['y']) * self.stiffness + (nxt['y'] - p['y']) * self.stiffness

            # Obstacle Repulsion
            for obs in self.obstacles:
                dx = p['x'] - obs['x']
                dy = p['y'] - obs['y']
                dist = math.hypot(dx, dy)
                effective_radius = obs.get('radius', 0.0) + self.repulsion_radius

                if 0.001 < dist < effective_radius:
                    force = self.repulsion_strength / ((dist + 10.0) ** 2)
                    rfx = (dx / dist) * force
                    rfy = (dy / dist) * force

                    # Tangential sliding component
                    tx = -rfy * 0.25
                    ty = rfx * 0.25

                    fx += rfx + tx
                    fy += rfy + ty

            # Apply integration & damping
            p['vx'] = (p['vx'] + fx) * self.damping
            p['vy'] = (p['vy'] + fy) * self.damping
            p['x'] += p['vx']
            p['y'] += p['vy']

    def get_path(self) -> List[Tuple[float, float]]:
        return [(p['x'], p['y']) for p in self.particles]


if __name__ == "__main__":
    solver = ElasticPathfinder(num_particles=20)
    solver.init(start=(0, 0), target=(500, 500), obstacles=[{'x': 250, 'y': 250, 'radius': 50}])
    
    for iteration in range(100):
        solver.step()
        
    path = solver.get_path()
    print(f"Path settled after 100 iterations. Midpoint: {path[len(path)//2]}")

```

---

## ⚙️ Configuration Tuning Parameters

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `numParticles` | `int` | `30` | Number of node links in the rubber-band chain. Higher values improve spatial smoothness at slight computational cost. |
| `stiffness` ($k$) | `float` | `0.15` | Spring tension coefficient. Higher values snap paths tighter around obstacles; lower values allow laxer curves. |
| `damping` ($\mu$) | `float` | `0.82` | Kinetic energy dissipation factor per frame ($0.0 - 1.0$). Prevents infinite string jitter/oscillation. |
| `repulsionRadius` | `float` | `100.0` | Outer distance boundary where an obstacle begins pushing particles away. |
| `repulsionStrength` | `float` | `4000.0` | Force scalar magnitude for obstacle rejection. Ramps up quadratically as distance decreases. |

---

## 💡 Practical Use Cases

* **🎮 Game Development:** Smooth enemy movement around moving obstacles without recalculating complex navmeshes.
* **🤖 Robotics & UAVs:** Real-time trajectory adjustment for mobile rovers and drones encountering unseen obstacles.
* **🎥 Dynamic Camera Rails:** Automated virtual camera movement in 3D environments that gracefully slides past scenery without clipping through geometry.
* **🎨 Generative Art & FX:** Interactive particle strings, fluid cable simulation, and ribbon mechanics.

---

## 📄 License

MIT License. Free to use, modify, and distribute for personal, academic, or commercial projects.

```

```
