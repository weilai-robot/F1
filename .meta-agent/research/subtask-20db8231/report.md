# State-of-the-Art Humanoid Robot Navigation (2024-2026)

A practical, deployment-oriented survey covering: SLAM drift from gait vibration, 3D-aware obstacle avoidance, alternatives to Nav2+MPPI, end-to-end RL navigation from LiDAR, and how commercial humanoids (Unitree, Tesla, Agility) navigate.

---

## 1. Key Findings (by sub-question)

### Q1 — SLAM drift from gait vibration
**Core problem:** Bipedal walking injects high-frequency vibration into the IMU and causes motion blur in cameras. Pure LiDAR-IMU odometry (FastLIO2) also drifts badly during dynamic locomotion, and fails outright in featureless areas (tunnels, flatlands) where the point cloud degenerates.

**Best available solutions (all add leg kinematics):**
- **Leg odometry is more robust than double-integrated IMU** under vibration, because leg forward-kinematics needs only *single* integration of joint velocities, so errors accumulate more slowly (Okawara 2025, Cerberus 2023).
- **Cerberus (open-source VILO):** fuses stereo cameras + IMU + joint encoders + foot-contact sensors in one factor graph with *online kinematic calibration* and *contact-outlier rejection*. Result: **<1% position drift after 450 m of travel** (reported, [partially verified — from secondary source/abstract]).
- **Okawara et al. 2025 (Tightly-Coupled LiDAR-IMU-Leg Odometry, RAL):** adds a **neural leg-kinematics model** trained *online* on a unified factor graph, incorporating **foot reaction-force (tactile)** data to implicitly model foot-ground dynamics (slippage, deformable terrain). Validated on Unitree Go2 on a sandy beach and mixed-terrain campus; outperforms SOTA in featureless + deformable conditions. Adapts to weight-load changes (e.g., carrying a payload).
- **elevation_mapping_cupy (ETH Zurich, open source):** explicit **"Height Drift Compensation"** module that corrects the state-estimation drift that would otherwise create vertical artifacts in the elevation map — directly addressing the manifestation of SLAM drift in downstream navigation.
- **Fallon et al. (Humanoids 2014, foundational):** "Drift-free humanoid state estimation" — probabilistic fusion of kinematic + inertial + LiDAR via a Gaussian Particle Filter that applies per-scan LiDAR corrections while walking. Still cited as the canonical multi-modal drift-free approach.

> **Practical takeaway:** Do NOT run FastLIO2 alone on a biped. Add a leg-odometry factor (Cerberus-style) and, for deformable/outdoor terrain, a learned/tactile leg model (Okawara). Expect an order-of-magnitude reduction in drift.

### Q2 — 3D-aware obstacle avoidance for tall bipeds
**Core problem:** A 2D costmap (Nav2 default) sees only a horizontal slice; a tall humanoid collides with tables, low ceilings, and its own upper body against shelves. Depth-camera-derived 2D elevation maps are blind to overhanging obstacles and fail in bad lighting.

**Best available solutions:**
- **GPU elevation mapping (elevation_mapping_cupy):** robot-centric 2.5D grid where each cell stores height + variance + traversability. Features specifically targeting 3D awareness: **Visibility Cleanup & Artifact Removal** (raycasting removes virtual artifacts, correctly interprets overhanging obstacles so they are not mistaken for walls), **learning-based traversability filter**, plane segmentation, and a Multi-Modal layer (RGB/semantics fused in). Runs on GPU for real-time updates. *This is the practical upgrade path from a flat 2D costmap.*
- **End-to-end 3D LiDAR policies (full-body awareness):** P3O-CBF and Omni-Perception consume the *raw 3D point cloud*, so they see the full vertical extent of the robot and can avoid suspended/overhanging obstacles that a 2.5D height map collapses. P3O-CBF explicitly tests a "Suspended Obstacle" scenario (walk under a low platform) where 2D elevation maps fail — achieves 83-90% success vs 20% for reward-shaping baseline.
- **Footstep planners with planar regions:** decompose the 3D scene into planar patches and place footsteps (Maier et al. IROS 2013, DeSouza) — best for rough/staircase terrain where foothold choice matters.

> **Practical takeaway:** Two tiers — (a) moderate cost: elevation_mapping_cupy (2.5D, handles overhangs via raycast cleanup, GPU, open source); (b) full 3D: end-to-end LiDAR policy that needs no intermediate map.

### Q3 — Alternatives to Nav2 + MPPI
MPPI is a *wheeled-base* predictive controller; it plans base velocity on a 2D plane and does not reason about footsteps or full-body 3D collision. Alternatives:

| Approach | What it is | Best for |
|---|---|---|
| **Footstep planners (SBPL / D*-Lite / A* on planar regions)** | Search discrete footstep actions over a 3D map; replan incrementally. Used by Agility Digit. | Discrete-terrain, stairs, clutter; where foothold matters |
| **MPPI on elevation maps (uneven-terrain MPPI)** | Adapts MPPI to consume a 2.5D elevation map as the cost input (arXiv:2209.07252) | Smooth-terrain wheeled/legged velocity control |
| **Constrained RL locomotion (P3O-CBF)** | End-to-end policy with CBF-derived safety cost in a CMDP | Reactive, low-latency, no map maintenance |
| **Reach-avoid value networks (Agile-but-Safe, ABS)** | Agile policy + recovery policy switched by a learned reach-avoid value function | High-speed collision-free navigation with a safety net |
| **Navigation World Models (CVPR 2025)** | Video-generation model that predicts future observations given actions; plan against the imagined future | Learning-based planning without explicit maps |
| **Foundation models for navigation (NAVER Labs)** | Large end-to-end agents trained on heterogeneous 3D scenes | Generalization across unseen environments |

> **Practical takeaway:** If you keep a modular stack, swap MPPI for a footstep planner (Digit's choice) or elevation-map-MPPI. If you want to escape the map→plan→track pipeline entirely, go end-to-end RL (P3O-CBF/ABS).

### Q4 — End-to-end RL navigation directly from LiDAR point clouds
**Yes — this is now deployable on real hardware.** Two leading 2025 papers (same research group, HKUST-GZ / IIT):

1. **P3O-CBF (arXiv:2508.07611) — read in full.** End-to-end policy maps **raw spatio-temporal LiDAR point clouds → motor commands** on Unitree G1 with a Livox Mid-360. LiDAR is encoded to a 64-dim vector, fused with proprioception (10-step history: joints, velocities, base twist, command), processed by a **GRU + MLP**. Safety is handled by translating **Control Barrier Function (CBF)** principles into CMDP costs, solved with **Penalized Proximal Policy Optimization (P3O)**. Adds HRI-grounded *comfort* rewards (proxemics 1.2 m social distance, tangential avoidance, approach-velocity penalty). **Deployed on physical G1** in cluttered lab + reactive human-approach tests. Sim2real via Isaac Sim + Genesis, domain randomization + curriculum.
2. **Omni-Perception (arXiv:2505.19214).** End-to-end policy with **PD-RiskNet (Proximal-Distal Risk-Aware Hierarchical Network)** for omnidirectional collision avoidance from raw LiDAR. Ships a high-fidelity LiDAR simulation toolkit (realistic noise, fast raycasting; compatible with Isaac Gym / Genesis / MuJoCo) to enable sim2real. Validated in real-world + extensive simulation.

> **Practical takeaway:** End-to-end LiDAR→torque RL is real and beats reward-shaping baselines (P3O-CBF: 53% less time in unsafe zone than PPO; 86% success vs 56% PPO on dynamic agents). Main cost: training infrastructure + a good LiDAR simulator.

### Q5 — How commercial humanoids navigate

| Robot | Navigation approach | Sensors | Deployment status |
|---|---|---|---|
| **Unitree G1 / H1 / H1-2** | ROS-based autonomy stack; demonstrated running open-source stack (FastLIO-style SLAM + occupancy grid + navigation). Community (sentdex) shows G1 doing SLAM + occupancy graph + nav with a single Mid-360 LiDAR. | Livox Mid-360 LiDAR (360°, solid-state), depth cameras, IMU | Widely deployed in research; open SDK; $16K (G1)–$90-150K (H1) |
| **Tesla Optimus** | **End-to-end neural networks** reused/adapted from the FSD stack. Uses the same **FSD inference computer** in the chest; a learned **world representation** + trajectory planning in that space; "48-neural-network FSD architecture" adapted to bipedal. Training leverages Tesla's 8.2B-mile vehicle data infrastructure and a learned "world simulator" (neural sim, not traditional physics sim). Vision-first (cameras), FSD-derived. | Cameras (vision-first), IMU, joint encoders, foot force/torque (Gen 2 feet) | Hundreds of units in internal Tesla-factory testing (as of 2026); consumer target ~2027; $20-30K aspirational |
| **Agility Digit (V4)** | **Redesigned custom navigation stack** with a **footstep path planner** (Navigation Team Lead Daniel Piedrahita). Torsos packed with sensors + computers. Optimized for warehouse logistics (unload AMRs, load conveyor totes). | Multi-sensor torso (cameras + depth + LiDAR-class sensing, [exact spec not fully public]) | **Commercially deployed** — GXO multi-year agreement; operates in real warehouses |
| **Figure 01/02** | Integrates LiDAR + 3D cameras for spatial awareness; AI-driven (VLA model Helix). | LiDAR + RGB-D cameras | Pilot deployments (BMW Spartanburg) |

> **Practical takeaway:** Unitree = accessible open LiDAR+ROS stack (best starting point for builders). Agility Digit = production footstep-planner stack (proven in warehouses). Tesla = betting on full end-to-end learned vision (no modular SLAM/footstep stack) — highest risk/highest ceiling.

---

## 2. Per-Source Details (with extraction-spec fields)

### Source A — P3O-CBF: End-to-End Humanoid Safe & Comfortable Locomotion Policy (arXiv:2508.07611, HKUST-GZ / IIT, 2025)
*Read in full.*
- **(1) Architecture:** Actor = 10-step proprioception/command history ⊕ 64-dim LiDAR embedding → GRU (temporal) → MLP → motor commands. Critic sees actor inputs + privileged info (true obstacle distance/velocity in 8 directions, contact forces, joint-limit/safe-distance flags). Trained with **P3O** (Penalized Proximal Policy Optimization) in a **CMDP**; safety as *costs* (not reward shaping). CBF principle: LDCBF barrier h_D = signed distance − D_min; violation → instantaneous cost C_D = max(0, −G_D).
- **(2) Sensors:** Livox Mid-360 LiDAR (raw point cloud), proprioception (joint pos/vel/acc, base linear+angular velocity, gravity vector, base height, previous action, user velocity command). Robot: Unitree G1.
- **(3) Deployment:** **Real — Unitree G1, onboard compute, real-time.** Two scenarios: cluttered lab traversal + reactive avoidance of a sudden human approach from behind.
- **(4) Compute:** All perception + inference onboard the robot in real time (exact SoC not specified). Network is small (64-dim LiDAR embedding + GRU + MLP) — inference-light by design.
- **(5) Advantages over Nav2+FastLIO2+MPPI:** (a) lighting-invariant 3D perception (vs depth-camera 2D elevation map); (b) sees overhanging/full-body obstacles (2D maps cannot); (c) principled safety via CBF-CMDP, not brittle reward shaping (53% less time in unsafe zone than PPO; 86% vs 56% success on dynamic agents; 60% vs 0% in narrow passage); (d) socially-aware "comfort" motion; (e) no map-maintenance/planning latency.
- **(6) Open source:** Project page https://github.com/aCodeDog/SafeHumanoidsPolicy (code availability to be confirmed).

### Source B — Tightly-Coupled LiDAR-IMU-Leg Odometry with Neural Leg Kinematics + Foot Tactile Info (Okawara et al., IEEE RAL, arXiv:2506.09548, 2025)
*Read in full.*
- **(1) Architecture:** Unified factor graph jointly solving odometry + *online training* of a **neural leg-kinematics model**. Network input = time-series (window=3) of [IMU accel(3), IMU gyro(3), joint angles(12), joint torques(12), foot force(4)] = 102-dim → common-feature layer → offline/static + online/adaptive motion-prediction layers → 6-DOF body twist + per-foot contact state. Split offline (terrain/weight-invariant basics) vs online (adaptive) layers to balance compute/accuracy. Explicit online uncertainty (covariance) estimation of the leg constraint.
- **(2) Sensors:** LiDAR, IMU, joint encoders, joint torques, 1-D foot force pads (Unitree Foot Pad). Robot: Unitree Go2 (quadruped; method generalizes to bipeds).
- **(3) Deployment:** **Real** — sandy beach (deformable + featureless) and campus (asphalt/gravel/grass + featureless areas); 3 kg payload removed mid-run to test adaptivity. Outperforms SOTA.
- **(4) Compute:** Online factor-graph optimization with small online MLP (168 params for the online part) — runs in real time alongside odometry.
- **(5) Advantages over FastLIO2 (pure LiDAR-IMU):** robust in featureless environments (point-cloud degeneracy) and on deformable terrain (foot slippage/sinkage) where FastLIO2 and conventional leg-odometry both fail; adapts to payload weight changes online.
- **(6) Open source:** Project page https://takuokawara.github.io/RAL2025_project_page/ (code release expected; [confirm]).

### Source C — Cerberus: Low-Drift VILO for Agile Locomotion (Yang et al., arXiv:2209.07654, ICRA 2023)
*Abstract + secondary verified.*
- **(1) Architecture:** Visual-Inertial-Leg Odometry — stereo-VIO + leg kinematics in one estimator; **online kinematic-parameter calibration** (handles link/foot-rubber deformation, machining errors) + **contact-outlier rejection** (rejects unreliable foot contacts during slip). Improved version Cerberus2.0 on GitHub.
- **(2) Sensors:** stereo cameras, IMU, joint encoders, foot-contact sensors.
- **(3) Deployment:** Real legged robots, various terrains.
- **(4) Compute:** Real-time (factor-graph/EKF-style).
- **(5) Advantages:** **<1% position drift after 450 m** — directly addresses gait-vibration drift that wrecks FastLIO2/camera-only VIO on bipeds/quadrupeds.
- **(6) Open source:** YES — https://github.com/ShuoYangRobotics/Cerberus2.0 .

### Source D — elevation_mapping_cupy (ETH Legged Robotics, arXiv:2204.12876 / 2309.16818, open source)
*Read GitHub README in full.*
- **(1) Architecture:** GPU (CuPy) robot-centric 2.5D elevation grid. Pipeline: multi-modal images + point clouds → associate to cells → fuse (per-layer algorithms) → post-process plugins (traversability, line detection). **MEM (Multi-Modal Elevation Map)** framework fuses geometry + semantics + RGB.
- **(2) Sensors:** any point-cloud source (LiDAR or depth camera) + optional images/semantics.
- **(3) Deployment:** Widely used in legged-robot navigation/locomotion (ANYmal lineage); production-grade.
- **(4) Compute:** **GPU required** (CuPy). Real-time on modern mobile GPUs.
- **(5) Advantages over 2D Nav2 costmap:** (a) **Height Drift Compensation** — removes vertical artifacts from state-estimation drift; (b) **Visibility Cleanup / raycasting** — correctly interprets overhanging obstacles (not misread as walls); (c) learning-based traversability; (d) multi-modal layers; (e) GPU speed.
- **(6) Open source:** YES — https://github.com/leggedrobotics/elevation_mapping_cupy (983★, active).

### Source E — Omni-Perception (arXiv:2505.19214, same group as P3O-CBF, 2025)
*Abstract read.*
- **(1) Architecture:** End-to-end policy; **PD-RiskNet (Proximal-Distal Risk-Aware Hierarchical Network)** processes raw spatio-temporal LiDAR for omnidirectional risk. Ships a **LiDAR simulation toolkit** (realistic noise + fast raycasting; Isaac Gym/Genesis/MuJoCo compatible) for scalable sim2real.
- **(2) Sensors:** raw LiDAR.
- **(3) Deployment:** Real-world + extensive sim validation.
- **(4) Compute:** Onboard (end-to-end inference).
- **(5) Advantages:** omnidirectional (360°) 3D avoidance; no intermediate map; robust to non-planar obstacles; the LiDAR sim toolkit lowers the barrier to training such policies.
- **(6) Open source:** Project page expected (follow P3O-CBF repo group).

### Source F — Agile-but-Safe / ABS (He et al., RSS 2024, arXiv:2401.17583)
*Project page + abstract read.*
- **(1) Architecture:** Three learned components + a switch: **agile policy** (fast maneuvers), **recovery policy** (prevents falls), **reach-avoid value network** (decides when to switch). Plus an exteroception-representation network. All trained in sim. Enables >1.0 m/s collision-free navigation (prior safe controllers were <1.0 m/s).
- **(2) Sensors:** exteroception (depth/LiDAR) + proprioception.
- **(3) Deployment:** Real quadrupeds; demonstrated in cluttered static+dynamic environments.
- **(4) Compute:** Onboard neural inference.
- **(5) Advantages over MPPI:** learned agility + formal-style safety net (reach-avoid) → faster yet safe; no manual trajectory sampling.
- **(6) Open source:** YES — https://github.com/LeCAR-Lab/ABS .

### Source G — Footstep planners (SBPL / D*-Lite / A* on planar regions; Garimort ICRA 2011; Maier IROS 2013)
*Secondary.*
- **(1) Architecture:** Discrete/continuous footstep search over a 3D/planar-region map; D*-Lite enables incremental replanning as the map changes; continuous footstep locations + efficient collision checking.
- **(2) Sensors:** 3D map source (LiDAR/RGB-D → octomap/planar regions).
- **(5) Advantages over MPPI:** reasons about *footsteps*, not base velocity — correct abstraction for bipedal robots on stairs/clutter.
- **(6) Open source:** ROS `footstep_planner` (SBPL-based); Agility Digit uses a custom redesigned version.

### Source H — Tesla Optimus (commercial, secondary + Tesla.com/AI)
*Secondary.*
- **(1) Architecture:** End-to-end learned; reuses the **FSD computer** (in the chest) and a **learned world representation** + trajectory planning in that space; "48-neural-network FSD architecture" adapted to bipedal control; a learned **"world simulator"** (neural, not physics) for training.
- **(2) Sensors:** cameras (vision-first), IMU, joint encoders, Gen-2 feet with force/torque sensing.
- **(3) Deployment:** hundreds of units internal factory testing (2026); consumer ~2027.
- **(5) Advantages:** leverages billions of miles of vehicle data + shared inference hardware; no manual SLAM/footstep-engineering; smooth heel-toe gait reported.
- **(6) Open source:** NO (proprietary).

### Source I — Unitree G1/H1 (commercial)
*Secondary.*
- **(1) Architecture:** ROS-based autonomy stack; community-demonstrated pipeline: Mid-360 LiDAR → SLAM (FastLIO-style) → occupancy grid → navigation. Open SDK + ROS 2 + Python.
- **(2) Sensors:** Livox Mid-360 LiDAR (solid-state, 360°), depth cameras, IMU.
- **(3) Deployment:** broad research + commercial; $16K (G1) to $90-150K (H1/H1-2).
- **(5) Advantages:** most accessible open platform for builders; single compact LiDAR suffices for indoor nav.
- **(6) Open source:** SDK + community stacks open; autonomy code partly open.

### Source J — Agility Digit (commercial)
*Secondary.*
- **(1) Architecture:** **Redesigned custom navigation stack** centered on a **footstep path planner** (Navigation Team Lead Daniel Piedrahita); torso with dense sensors/computers; focuses on real-world warehouse logistics.
- **(2) Sensors:** multi-sensor torso (cameras + depth + range sensing).
- **(3) Deployment:** **Commercially deployed — GXO multi-year agreement; real warehouses** (the most production-proven humanoid nav).
- **(6) Open source:** NO (proprietary).

---

## 3. Synthesis & Recommendation

**For a builder deploying a humanoid navigation stack today, a pragmatic layered architecture emerges:**

1. **Odometry/localization (kill gait-vibration drift):** Start with **Cerberus** (open-source VILO) or, if you have LiDAR, a **LiDAR-IMU-leg** tightly-coupled estimator (Okawara 2025 style with neural+tactile leg model for outdoor/deformable terrain). Do NOT run FastLIO2 alone on a biped.
2. **3D-aware mapping:** Use **elevation_mapping_cupy** (GPU 2.5D + height-drift compensation + overhang handling + traversability). This is the direct, open-source replacement for a flat Nav2 2D costmap and is the lowest-risk upgrade.
3. **Planning/control:** Keep modular → **footstep planner** (Digit's proven choice) or elevation-map-MPPI. Go end-to-end → **P3O-CBF / Omni-Perception** (raw-LiDAR RL, sim2real-validated on G1) if you want no map and low latency. Add **Agile-but-Safe** for a learned safety net.
4. **Sensor baseline:** a single **360° solid-state LiDAR (e.g., Livox Mid-360)** + depth cameras + IMU is sufficient for indoor humanoid navigation, as Unitree's deployed G1/H1 demonstrates.

**Commercial divergence:** Unitree (open LiDAR+ROS) and Agility (custom footstep planner, production-proven) represent the *modular/engineered* school; Tesla (end-to-end FSD-derived neural vision) represents the *fully-learned* bet. The modular school is deployable now; the learned school is the higher-ceiling frontier.

**Gaps / unverified:** exact drift numbers for Cerberus on a *biped* (the <1%/450m figure is reported from secondary sources, likely quadruped); precise Agility Digit sensor bill-of-materials (not public); Tesla Optimus specifics are largely marketing/secondary (no peer-reviewed paper). P3O-CBF/Omni-Perception end-to-end policies are research-stage (single lab, limited scenario count) — treat as proof-of-concept, not production-ready.