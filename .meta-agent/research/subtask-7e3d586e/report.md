# State-of-the-Art Navigation and Obstacle Avoidance for Humanoid Robots (2023-2025)

## Question

What are the state-of-the-art navigation and obstacle avoidance approaches for humanoid robots in 2023-2025? Compare Nav2-based methods vs learning-based methods. Focus on: (1) 3D costmap/voxel approaches for legged robots, (2) MPPI vs other local planners, (3) LiDAR-inertial SLAM alternatives to FastLIO2, (4) How humanoid-specific challenges (height, Z-axis instability, limited compute) are addressed.

---

## Key Findings

### 1. The Paradigm Shift: From Classical Stacks to End-to-End RL

The humanoid navigation field is undergoing a paradigm shift from classical perception-planning-control pipelines (Nav2 + SLAM + elevation maps + MPPI) to **end-to-end reinforcement learning policies that directly map raw sensor data to motor commands**. The key driver is that humanoid robots face 3D obstacles (overhangs, stairs, uneven terrain) that 2.5D navigation representations fundamentally cannot capture.

**Three dominant approaches in 2024-2025:**
1. **Classical/Hybrid (Nav2-based):** SLAM → elevation map → Nav2 global planner → MPPI local controller. Still practical for wheeled/quadruped platforms but struggles with full-body humanoid collision avoidance.
2. **RL with Height Maps (2.5D perception):** Uses depth camera or LiDAR-derived elevation maps as policy input. Works for terrain traversal but blind to non-ground-level obstacles.
3. **End-to-End RL with Raw LiDAR (3D perception):** Directly processes 3D point clouds. The SOTA for humanoid navigation — captures overhangs, aerial obstacles, and full-body collision risks.

### 2. Humanoid-Specific Challenges and Solutions

| Challenge | Classical Approach | Learning-Based Approach |
|-----------|-------------------|----------------------|
| **Height** (full-body collision) | Voxel layer in Nav2 costmap, but 2D squash loses 3D info | Raw LiDAR point cloud → PD-RiskNet/P3O-CBF (direct 3D awareness) |
| **Z-axis instability** | ZMP/MPC-based gait control | Reward penalties on Z-velocity (weight: -3×10⁻⁴), joint torque limits |
| **Limited compute** | FAST-LIO2 + Nav2 (heavy stack) | Compact LiDAR embedding (64-dim vector), Super-LIO (3.7-4.2x faster) |
| **Dynamic obstacles** | Dynamic obstacle layers (reactive, slow) | CBF-based safety constraints + comfort rewards (proactive avoidance) |
| **Social awareness** | Not addressed | Proxemic comfort rewards (social distance = 1.2m), tangential avoidance |

---

## Per-Source Details

### Source 1: End-to-End Humanoid Robot Safe and Comfortable Locomotion Policy

- **Paper:** Wang et al., "End-to-End Humanoid Robot Safe and Comfortable Locomotion Policy," arXiv:2508.07611, 2025. HKUST(GZ) & IIT.
- **Category:** Learning-based (end-to-end RL with constrained MDP)
- **Method name:** P3O-CBF (Penalized Proximal Policy Optimization with Control Barrier Function costs)

**Key details:**
- **Sensor:** Livox Mid-360 LiDAR (raw 3D point clouds) + proprioception (joint positions, velocities, accelerations, base velocity, gravity vector, base height)
- **Network:** LiDAR features (64-dim) processed by GRU → concatenated with proprioception history (10 timesteps) → MLP actor. Critic receives privileged info (true obstacle distances in 8 directions, contact forces, joint limits).
- **Safety framework:** CMDP formulation with Linear Discrete-Time CBF (LDCBF). The CBF barrier function h_D(s_k) = η_k^T(p(s_k) - o_k) - D_min defines signed distance to obstacles. CBF violation → cost function for P3O constraint optimization. Hard safety margin d_safe = 0.8 m.
- **Comfort rewards:** Proxemic comfort (social distance 1.2m), safe approach velocity (penalizes velocity component normal to obstacles), tangential avoidance (encourages perpendicular-to-obstacle velocity vectors), motion smoothness.
- **Training:** NVIDIA Isaac Sim + Genesis (sim2sim), domain randomization, structured curriculum.

**Results (30 trials per scenario):**

| Scenario | PPO-RewardShaping | P3O | P3O-CBF (Ours) |
|----------|-------------------|-----|----------------|
| Suspended Obstacle | 20% | 90% | 83% |
| Narrow Passage | 0% | 33% | 60% |
| Cluttered Static Course | 93% | 100% | 100% |
| Dynamic Agents | 56% | 70% | 86% |

- **Safety:** P3O-CBF reduced unsafe-space time by 53% vs PPO, uncomfortable-space time by 35% vs PPO.
- **Real-world deployment:** Unitree G1 humanoid, onboard computation, cluttered lab + dynamic human approach scenarios. Zero-shot sim-to-real.
- **ROS2 Nav2 integration:** None (standalone end-to-end policy)
- **Limitations:** Narrow passage success only 60%; suspended obstacle success slightly lower than P3O (83% vs 90%) suggesting CBF cost may sometimes over-constrain in confined spaces.

**Excerpt — reward weights (Table I):**
- Velocity tracking: weight 2.0, exp(-α_v ‖v_k - v_k^cmd‖²)
- Z velocity penalty: -3×10⁻⁴
- Joint torques: -1×10⁻⁶
- Action smoothing: -5×10⁻³
- Proxemic comfort: weight 1.5
- Safe approach velocity: -1.0
- Tangential avoidance: 1.0

---

### Source 2: VAPOR — Legged Robot Navigation in Unstructured Outdoor Environments

- **Paper:** Weerakoon et al., arXiv:2309.07832, 2023. University of Maryland.
- **Category:** Learning-based (offline RL)
- **Method:** VAPOR — Vegetation-Aware Planning using Offline RL with CQL-SAC

**Key details:**
- **Platform:** Boston Dynamics Spot (quadruped, applicable to legged/humanoid)
- **Sensor inputs:** Three robot-centric 2D cost maps from 3D LiDAR:
  1. **Intensity cost map** (C_i): LiDAR reflectance → solidity/pliability of vegetation (grass vs trees)
  2. **Height cost map** (C_h): Maximum object height per grid cell
  3. **Goal cost map** (C_g): Distance/direction to goal
  Plus proprioceptive signals (joint positions, forces, battery current) for stability monitoring.
- **RL algorithm:** Conservative Q-Learning (CQL) with SAC backbone. Actor-critic with spatial and channel attention layers.
- **Offline training:** ~4 hours of teleoperation data collected in real outdoor vegetation. No simulation needed — eliminates sim-to-real gap.
- **Context-aware planner:** Switches between holonomic velocity space (minimize entrapment risk in vegetation) and non-holonomic velocity space (navigate narrow passages between solid obstacles).

**Results:**
- Up to 40% improvement in success rate vs end-to-end CQL-SAC and IQL baselines
- 2.9% decrease in average current consumption (energy efficiency)
- 11.2% decrease in normalized trajectory length

**Key advantage for legged robots:** Proprioceptive feedback detects vegetation entanglement before it becomes critical — the robot can back out holonomically instead of rotating (which worsens entanglement).
- **ROS2 Nav2 integration:** Works alongside existing navigation stacks (context-aware planner generates velocities evaluated by the RL critic)
- **Limitations:** Requires data collection in target environment (offline RL limitation); primarily tested on Spot quadruped, not humanoid

---

### Source 3: Omni-Perception — Omnidirectional Collision Avoidance

- **Paper:** Wang et al., arXiv:2505.19214, 2025. HKUST(GZ) & Beijing Innovation Center of Humanoid Robotics.
- **Category:** Learning-based (end-to-end RL with raw LiDAR)
- **Method:** Omni-Perception with PD-RiskNet (Proximal-Distal Risk-Aware Hierarchical Network)

**Key details:**
- **Platform:** Unitree G1 humanoid, Livox Mid-360 LiDAR
- **Core innovation:** PD-RiskNet partitions raw point cloud into proximal (near-field, dense → FPS sampling → GRU with privileged height supervision) and distal (far-field, sparse → average downsampling → GRU) subsets. Proximal GRU outputs 187 features, distal GRU outputs 64 features.
- **Actor network:** MLP [1024, 512, 256, 128] → 12 joint position targets
- **Reward function:** Novel components:
  - Velocity tracking with avoidance: 360° divided into 36 sectors, each sector generates avoidance velocity if obstacle within threshold (1m). exp(-d_j × α_avoid) magnitude.
  - Distance maximization: rewards maximizing capped LiDAR ray distances
- **Training:** PPO, 4096 parallel environments, Isaac Gym
- **LiDAR simulation toolkit:** Cross-platform (Isaac Gym, Genesis, MuJoCo), supports non-repetitive scanning LiDARs (Livox Mid-360), self-occlusion modeling, 10% point masking + 10% noise for domain randomization.

**Real-world results (30 trials each):**

| Scenario | Omni-Perception | Native Unitree System |
|----------|----------------|-----------------------|
| Static obstacles | 100% | 100% |
| Aerial obstacles | 70% | 0% |
| Small obstacles | 83% | 100% |
| Moving humans | 90% | 0% |

- **Key advantage:** The native Unitree system completely fails on aerial obstacles and moving humans (0% success), while Omni-Perception achieves 70% and 90% respectively.
- **Limitations:** Dense grass degrades LiDAR geometric features; very small/thin objects can be lost in distal point cloud averaging.
- **ROS2 Nav2 integration:** None (standalone)

---

### Source 4: MPPI with Repulsive Potential Augmentation (RPA-MPPI)

- **Paper:** Fuke et al., "Towards Local Minima-free Robotic Navigation: MPPI via Repulsive Potential Augmentation," arXiv:2410.11379, 2025. Keio University.
- **Category:** Classical (model-predictive control enhancement)
- **Method:** RPA-MPPI — MPPI with Artificial Potential Field repulsive term

**Key details:**
- **Problem addressed:** MPPI (and MPC in general) suffers from local minima entrapment in finite-horizon optimization. When a robot faces a large obstacle between itself and the goal, the robot gets stuck behind the obstacle.
- **Solution:** Integrate a repulsive potential field term into the MPPI cost function. The key insight is that this provides "soft global guidance" without requiring explicit global path planning.
- **Cost formulation:** Conventional cost: ϕ(p) = ‖p_goal - p‖² + w_obst × 1_obst(p). RPA adds: repulsive potential term that pushes the cost landscape away from obstacle configurations that would cause local minima.
- **Theoretical guarantee:** The paper proves that the repulsive potential-augmented cost function is free of local minima for rectangular obstacles (extensible to 3D via axial symmetry).
- **Computational cost:** No additional path search needed. The modification is minimal — just an added term in the cost function. Maintains MPPI's GPU-parallelizable rollout structure.

**Advantages over alternatives:**
- Vs. reactive methods (Nav2 recovery behaviors): Better global optimality, not just myopic escape
- Vs. global planner + MPPI: No computational overhead for path search; no path tracking issues

- **ROS2 Nav2 integration:** Directly applicable to Nav2's MPPI controller (nav2_mppi_controller). The cost function modification is straightforward to implement as a critic extension.
- **Limitations:** Only validated in 2D simulations; dynamic local minima detection and non-convex obstacle handling remain open problems. Not yet tested on humanoid platforms.

**MPPI fundamentals (from Nav2 docs):**
- MPPI generates K randomly perturbed control sequences from a Gaussian distribution, forward-simulates each, and computes a weighted average using softmax-weighted costs.
- GPU-parallelizable (rollouts are independent)
- Handles non-convex costs and nonlinear dynamics without gradient computation
- Nav2's MPPI is the "successor to TEB and pure path tracking MPC controllers"

---

### Source 5: Super-LIO — Efficient LiDAR-Inertial Odometry

- **Paper:** Wang et al., "Super-LIO: A Robust and Efficient LIO System with Compact Mapping Strategy," arXiv:2509.05723, 2025. ECUST & Shanghai AI Lab.
- **Category:** SLAM/localization
- **Method:** Super-LIO with OctVox map + HKNN search

**Key details:**
- **Core innovation — OctVox map:** Each voxel subdivided into 2×2×2 = 8 subvoxels, each storing one incrementally averaged point. Achieves strict density control (max 8 points per voxel) and global noise suppression through incremental averaging. O(1) voxel access via Robin Hood hashing.
- **HKNN (Heuristic-Guided KNN):** Exploits subvoxel octant symmetry to precompute near-to-far traversal order with early termination. Better neighbor quality than Faster-LIO's 18-neighbor search without significantly more computation.
- **State formulation:** IESKF (Iterated Error-State Kalman Filter) with 18-dimensional state: [R, p, v, b_a, b_g, g] ∈ SO(3) × R¹⁵

**Performance comparison (per-frame processing time, ms):**

| Method | X86 (AMD 5800H, 5× speed) | ARM (Cortex-A78AE / Orin NX, 1× speed) |
|--------|---------------------------|----------------------------------------|
| **Super-LIO** | ~4.5 ms | ~17 ms |
| FAST-LIO2 | ~19 ms | ~80 ms |
| Faster-LIO | ~13 ms | ~40 ms |
| iG-LIO | ~7 ms | ~21 ms |

- **Speedup:** 3.7× faster than FAST-LIO2 on X86, 4.2× faster on ARM. Lower CPU utilization.
- **Accuracy:** Average RMSE 0.74 m vs FAST-LIO2's 0.81 m across 18 public sequences — competitive or better.
- **Tested on:** M2DGR, NCLT, MCD, NTU datasets + 10 self-collected sequences (parks, forests, factories, garages, offices). Total 30+ sequences, 400+ minutes.
- **Supported LiDARs:** Livox Mid-360, Velodyne HDL-32E, Livox MID-70, Ouster-16 — plug-and-play.
- **ROS2 Nav2 integration:** Open-source, designed as drop-in LIO module (https://github.com/Liansheng-Wang/Super-LIO.git)
- **Critical for humanoids:** The ARM (Orin NX) performance is especially relevant — humanoid onboard computers typically use ARM/embedded processors with limited compute budget. 17 ms/frame on ARM enables real-time operation with significant headroom for downstream navigation tasks.

**FAST-LIO2 baseline (for comparison):**
- Tightly-coupled LiDAR-IMU with iterated EKF
- Incremental KD-tree (iKD-Tree) for nearest-neighbor queries
- Removed explicit feature extraction (uses all raw points)
- Widely adopted as the standard LIO for legged/humanoid robots
- Limitation: O(log N) amortized update cost, pointer-heavy traversals reduce cache efficiency

**Other LIO alternatives mentioned in literature:**
- **Faster-LIO:** Replaces iKD-tree with sparse hash-voxel map (iVox), up to 2000 Hz, but sensitive to voxel parameters
- **Point-LIO:** Point-wise state updates, enhanced robustness in high-dynamic scenarios, but higher computation
- **LIO-SAM:** Pose graph optimization backend via GTSAM, global consistency through loop closure, but iterative solvers scale poorly
- **KISS-ICP:** Hash-voxel grid with capped points per voxel, predictable memory, but limited density control

---

### Source 6: GPU-Based Elevation Mapping for Legged Robots

- **Paper/Software:** Miki et al., "Elevation Mapping for Locomotion and Navigation using GPU," IROS 2022. ETH Zurich (leggedrobotics).
- **Software:** elevation_mapping_cupy (https://github.com/leggedrobotics/elevation_mapping_cupy)
- **Category:** Classical perception (2.5D terrain representation)

**Key details:**
- **Representation:** 2.5D grid map storing height values per cell, robot-centric frame. GPU-accelerated using CuPy for real-time processing.
- **Multi-Modal Elevation Map (MEM) framework:** Integrates geometry, semantics, and RGB information into layered map channels.
- **Use case:** Input to locomotion controllers (foothold planning) and navigation planners (traversability assessment).
- **Limitation (critical for humanoids):** 2.5D representation is fundamentally blind to overhanging obstacles, non-planar geometry, and aerial clutter. As noted by Wang et al. (2025): "The reduction of 3D sensory information to a 2D elevation map makes the robot blind to any non-ground-level obstacles, such as overhanging clutter or the upper bodies of other agents. This limitation poses a substantial collision risk for a full-body humanoid robot."
- **ROS2 integration:** Full ROS2 node, widely used in legged robot research (ANYmal, Spot, etc.)

---

### Source 7: Nav2 Voxel Layer and Costmap System

- **Source:** Nav2 1.0.0 documentation (docs.nav2.org)
- **Category:** Classical navigation

**Key details:**
- **Voxel Layer:** Implements 3D raycasting for depth/3D sensors. Maintains a 3D environmental model internally but "squashes down to 2D for planning and control." The 2D squash loses height information — obstacles above or below the robot's footprint plane are collapsed.
- **Parameters:** max_obstacle_height, min_obstacle_height, voxel_size, unknown_threshold, mark_threshold.
- **Key limitation for humanoids:** The 2D squash means a humanoid robot cannot distinguish between a low table (walk over) and a wall (must go around). The robot's tall body means its upper torso/arms can collide with obstacles that a 2D costmap marks as free.
- **Global costmap:** Long-range route planning using static + voxel layers.
- **Local costmap:** Near-field obstacle avoidance with rolling window.
- **MPPI Controller (nav2_mppi_controller):** Official Nav2 local trajectory planner. Successor to TEB and DWB controllers. Uses importance sampling with Gaussian-perturbed control sequences. GPU-parallelizable rollouts. Presented at ROSCon 2023 by Steve Macenski.

---

### Source 8: Berkeley Real-World Humanoid Locomotion with RL

- **Paper:** Radosavovic et al., "Real-world humanoid locomotion with reinforcement learning," Science Robotics, 2024. DOI: 10.1126/scirobotics.adi9579. UC Berkeley.
- **Category:** Learning-based (RL locomotion)

**Key details:**
- **Platform:** Digit humanoid (Agility Robotics)
- **Architecture:** Transformer model ("humanoid transformer") that predicts next action based on history of proprioceptive observations and actions.
- **Training:** Pre-trained on flat-ground trajectories with sequence modeling → fine-tuned on uneven terrain using RL.
- **Key findings:** Transformer architecture outperforms other NN architectures; model benefits from larger context; joint teacher imitation + RL training is beneficial.
- **Emergent behavior:** In-context adaptation — gait changes based on terrain. Robot commanded over flat → downhill → flat terrain showed adaptive locomotion.
- **Real-world results:** Traversed 4+ miles of hiking trails in Berkeley, climbed steep streets in San Francisco. Zero-shot sim-to-real deployment.
- **Sensor:** Proprioception only (blind locomotion — no exteroceptive sensors). This is a limitation for obstacle avoidance/navigation in cluttered environments.
- **ROS2 Nav2 integration:** Not applicable (locomotion controller, not navigation system)

---

### Source 9: Comprehensive Humanoid Robotics Roadmap (PMC12783740)

- **Paper:** Uthai et al., "Opportunities, challenges and roadmap for humanoid robots in construction," Scientific Reports, 2025. University of Florida & NVIDIA.
- **Category:** Survey/roadmap

**Key comparison table (adapted):**

| Category | Method | Core Innovation | Limitations |
|----------|--------|-----------------|-------------|
| **Locomotion** | Classical bipedal (ZMP) | COM within support polygon | Limited adaptability to rough terrain |
| | Deep RL locomotion | Sim-trained walking policies | Hardware-specific tuning; computationally intensive |
| **Perception** | Multi-modal sensing | LiDAR + RGB-D + IMU fusion | Susceptible to environmental factors (lighting, dust) |
| | Scene parsing | Deep learning for object recognition | Performance degrades in cluttered/variable conditions |

**Humanoid-specific challenges identified:**
1. **3D complexity:** Construction sites have overhead cranes, trenches underfoot, partial floors at intermediate levels — "2.5D" approaches fail.
2. **Dynamic environments:** Site layouts change daily; frequent re-mapping required.
3. **Multi-modal sensor fusion:** Need LiDAR + stereo + IMU + radar fusion for robust perception under dust/glare/occlusion.
4. **Compute constraints:** Onboard computers for humanoids (e.g., Orin NX class) have ~30-90 min battery endurance with ~1.5-3.0 kWh packs.
5. **Safety protocols:** Real-time collision avoidance, fallback procedures, emergency stops — especially critical for tall bipedal robots.

**Platform comparison (Table 2, adapted):**

| Robot | Payload (kg) | Weight (kg) | DoF | Status |
|-------|-------------|------------|-----|--------|
| Unitree G1 | 3 | ~35 | 29 | Commercial ($21-66K) |
| Unitree H1 | 30 | ~47 | 19 | Commercial ($94-150K) |
| Boston Dynamics Atlas | 11 | 86-150 | 50 | R&D only |
| Tesla Optimus | 20 | 47-72 | ~40 | Prototype |
| Agility Digit | 18 | 45-63.5 | 16 | Limited production |

---

### Source 10: Humanoid Robot Navigation in Shared Care Spaces

- **Paper:** ACM 3802842.3802897, 2025.
- **Category:** Hybrid (classical SLAM + navigation)

**Key details:**
- **Platform:** Unitree G1 humanoid
- **Approach:** Advanced SLAM with multi-modal sensors (LiDAR, depth cameras)
- **Application:** Healthcare/care environments
- **Key takeaway:** Demonstrates that classical Nav2+SLAM stacks CAN work on humanoid platforms, but the paper is a "proof-of-concept" deployment — implying the approach is not yet production-ready.

---

## Synthesis & Recommendation

### Architecture Decision Matrix

| Requirement | Recommended Approach | Rationale |
|-------------|----------------------|----------|
| **3D obstacle avoidance** (overhangs, aerial) | End-to-end RL with raw LiDAR (Omni-Perception/P3O-CBF) | 2.5D elevation maps fundamentally cannot detect non-ground obstacles; raw point cloud gives full 3D awareness |
| **Dynamic environments** (moving humans) | P3O-CBF with comfort rewards | 86% success vs 0% for native systems; CBF provides formal safety guarantees; comfort rewards produce socially acceptable behavior |
| **Outdoor vegetation/terrain** | VAPOR-style offline RL with intensity+height cost maps | LiDAR intensity differentiates solid vs pliable obstacles; proprioception detects entanglement |
| **Compute-constrained platforms** | Super-LIO (SLAM) + lightweight RL policy | 17ms/frame on ARM Orin NX; 64-dim LiDAR embedding fits in small networks |
| **Nav2 compatibility required** | Nav2 + MPPI + voxel layer + RPA-MPPI enhancement | Maintains ROS2 ecosystem compatibility; RPA-MPPI solves local minima without global replanning |
| **Unknown/challenging terrain locomotion** | Transformer-based RL (Berkeley approach) | Proven on 4+ miles of hiking trails; emergent terrain adaptation |

### Key Tradeoffs

1. **Nav2 + Classical Stack vs End-to-End RL:**
   - Nav2 advantages: Modular, debuggable, ROS2 ecosystem, swappable components, well-documented.
   - Nav2 disadvantages: 2D costmap fundamentally inadequate for humanoids; MPPI local minima; requires separate SLAM, elevation mapping, and planning nodes.
   - RL advantages: Direct 3D perception, tight perception-action coupling, handles non-convex obstacle geometries, can learn social behaviors.
   - RL disadvantages: Black-box safety guarantees (though CBF-based approaches help), sim-to-real gap, requires GPU training, harder to debug.

2. **Depth Camera vs LiDAR for Humanoid Perception:**
   - Depth cameras: Cheaper, lighter, provide RGB+depth. But sensitive to lighting, limited FoV, noise. Elevation maps from depth are blind to overhangs.
   - LiDAR: Lighting-invariant, 360° coverage, direct 3D measurement. But higher cost, heavier, and harder to simulate for RL training. The Omni-Perception LiDAR simulation toolkit addresses this gap.
   - **Recommendation for humanoids:** LiDAR (e.g., Livox Mid-360) is strongly preferred for humanoid navigation due to 3D awareness and lighting invariance.

3. **FAST-LIO2 vs Super-LIO vs Others:**
   - FAST-LIO2: Most mature, most adopted, but ~80ms/frame on ARM.
   - Super-LIO: 4.2× faster on ARM with competitive accuracy. Best for compute-constrained humanoids.
   - Point-LIO: Best robustness for aggressive motion, but higher compute.
   - LIO-SAM: Best for long-term mapping with loop closure, but poor scalability.

### Practical Recommendations for a Humanoid Navigation System (2025)

**Tier 1 — Research/SOTA system:**
- SLAM: Super-LIO (Livox Mid-360 LiDAR + IMU)
- Navigation: End-to-end P3O-CBF policy (LiDAR → motor commands)
- Safety: CBF-based cost constraints in CMDP formulation
- Training: Isaac Sim/Genesis with custom LiDAR simulation toolkit
- Target platform: Unitree G1 or H1

**Tier 2 — Production/ROS2-compatible system:**
- SLAM: FAST-LIO2 or Super-LIO
- Navigation: Nav2 with Smac Hybrid-A* planner (global) + MPPI with RPA enhancement (local)
- Costmap: Voxel layer with point cloud input (not just laser scan)
- Perception: elevation_mapping_cupy for terrain + voxel layer for obstacles
- Locomotion: Separate RL locomotion policy (Berkeley-style transformer) for terrain adaptation

**Tier 3 — Outdoor/all-terrain system:**
- SLAM: FAST-LIO2 or Super-LIO
- Navigation: VAPOR-style offline RL with intensity + height cost maps
- Perception: 3D LiDAR with intensity-based traversability classification
- Context-aware planner that switches holonomic/non-holonomic based on vegetation detection

### Open Problems

1. **Narrow passage navigation** for full-body humanoids remains challenging (P3O-CBF only 60% success rate)
2. **Multi-modal fusion** of LiDAR + RGB-D + tactile sensing for humanoid navigation is largely unexplored
3. **Continual/online learning** for navigation policies deployed in changing environments
4. **Formal safety verification** of end-to-end RL policies beyond CBF-based cost constraints
5. **Semantic understanding** integrated with navigation (e.g., recognizing traversable vs non-traversable vegetation)
6. **Whole-body collision avoidance** — current approaches treat the robot as a point/footprint, not accounting for arms, torso, and head collision risks in full 3D

---

*Note: Some quantitative results are based on limited trial counts (30 trials) from single papers. For production deployment, these should be validated with larger sample sizes and diverse environments.*