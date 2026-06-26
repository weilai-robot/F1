## Sources Consulted

### Primary (read in full)

1. **P3O-CBF: End-to-End Humanoid Robot Safe and Comfortable Locomotion Policy** — Wang et al., 2025, arXiv:2508.07611 (HKUST-GZ / IIT). https://arxiv.org/html/2508.07611v1 — *End-to-end raw-LiDAR→motor policy with CBF-CMDP safety; deployed on Unitree G1.* Key excerpts: "directly maps raw, spatio-temporal LiDAR point clouds to motor commands"; "LiDAR embedding ... is a 64-dimensional vector"; "GRU ... concatenated with the flattened proprioceptive"; success rates PPO 20%/P3O 90%/Ours 83% (suspended obstacle), 56%/70%/86% (dynamic agents); "all computations ... performed in real-time on the robot's onboard computer"; "Unitree G1 ... equipped with a Livox Mid-360 LiDAR".

2. **Tightly-Coupled LiDAR-IMU-Leg Odometry with Online Learned Leg Kinematics Incorporating Foot Tactile Information** — Okawara et al., IEEE RAL 2025, arXiv:2506.09548. https://arxiv.org/html/2506.09548v2 — *Neural leg-kinematics model with foot reaction-force, online-trained on a unified factor graph.* Key excerpts: "leg kinematics-based motion prediction requires a single integration, which accumulates errors more slowly compared to double integration"; input "IMU data ... joint angles ... joint torques ... and foot force sensor values"; "outperforms state-of-the-art works" on sandy beach + mixed-terrain campus; Unitree Go2.

3. **elevation_mapping_cupy** — Miki et al. (ETH), IROS 2022, arXiv:2204.12876; MEM arXiv:2309.16818. https://github.com/leggedrobotics/elevation_mapping_cupy — *GPU 2.5D elevation map with height-drift compensation, overhang/visibility cleanup, traversability.* Key excerpts (README): "**Height Drift Compensation**: Tackles state estimation drifts"; "**Visibility Cleanup and Artifact Removal**: Raycasting ... correctly interpret overhanging obstacles"; "**Learning-based Traversability Filter**"; "**GPU-Enhanced Efficiency**".

### Primary (abstract / key data)

4. **Omni-Perception: Omnidirectional Collision Avoidance for Legged Locomotion in Dynamic Environments** — Wang et al., 2025, arXiv:2505.19214. https://arxiv.org/abs/2505.19214 — *PD-RiskNet processes raw LiDAR for omnidirectional avoidance; ships LiDAR sim toolkit.* Excerpt: "direct integration of LiDAR sensing into end-to-end learning for legged locomotion remains underexplored ... PD-RiskNet ... high-fidelity LiDAR simulation toolkit with realistic noise modeling and fast raycasting, compatible with Isaac Gym, Genesis, and MuJoCo."

5. **Cerberus: Low-Drift Visual-Inertial-Leg Odometry For Agile Locomotion** — Yang et al., ICRA 2023, arXiv:2209.07654. https://github.com/ShuoYangRobotics/Cerberus2.0 — *Open-source VILO; online kinematic calibration + contact-outlier rejection.* Excerpt (secondary): "position estimation drift below 1% after 450m of travel." [drift figure from secondary source — partially verified]

6. **Agile-but-Safe (ABS): Learning Collision-Free High-Speed Legged Locomotion** — He et al., RSS 2024, arXiv:2401.17583. https://github.com/LeCAR-Lab/ABS — *Agile + recovery policy + reach-avoid value network; >1.0 m/s safe nav.* Excerpt: "an agile policy to execute agile motor skills ... and a recovery policy to prevent failures, collaboratively achieving high-speed and collision-free navigation."

7. **MPPI on elevation maps** — arXiv:2209.07252 (2022). https://arxiv.org/pdf/2209.07252 — *Adapts MPPI to uneven terrain via 2.5D elevation map as cost input.* Excerpt: "we adapt MPPI control method to uneven terrains using 2.5D elevation maps."

8. **Navigation World Models** — Bar et al., CVPR 2025. https://openaccess.thecvf.com/content/CVPR2025/papers/Bar_Navigation_World_Models_CVPR_2025_paper.pdf — *Controllable video-generation model predicts future observations from actions.* Excerpt: "a Navigation World Model (NWM), a controllable video generation model that predicts future visual observations based on past observations and navigation actions."

9. **Drift-free humanoid state estimation fusing kinematic, inertial and LiDAR** — Fallon et al., Humanoids 2014. https://www.pure.ed.ac.uk/ws/files/18903340/14_fallon_humanoids.pdf — *GPF fusing 3 modalities; foundational.* Excerpt: "incorporation of exteroceptive sensing to achieve reliable drift-free alignment to a prior map while walking using a Gaussian Particle Filter."

10. **Integrated Perception, Mapping, and Footstep Planning for Humanoid Navigation Among 3D Obstacles** — Maier et al., IROS 2013. http://vigir.missouri.edu/~gdesouza/Research/Conference_CDs/IEEE_IROS_2013/media/files/0457.pdf — *3D map → planar projection + circular collision footprint.*

11. **Humanoid Navigation with Dynamic Footstep Plans (D*-Lite)** — Garimort et al., ICRA 2011. http://www.arminhornung.de/Research/pub/garimort11icra.pdf — *Incremental D*-Lite footstep replanning.*

12. **Footstep Planning Over Rough Terrain (planar regions)** — arXiv:1907.08673. https://ar5iv.labs.arxiv.org/html/1907.08673 — *A* footstep planner on planar-region decomposition.*

### Commercial / secondary

13. **Unitree G1/H1** — https://robotsguide.com/robots/unitree-g1 ; sentdex dev series https://www.youtube.com/watch?v=sJYlJlIEBpg ; open-stack demo https://www.instagram.com/reel/DVigM3wDt12 — *Mid-360 LiDAR + ROS stack; G1 from $16K.*

14. **Tesla Optimus** — https://www.tesla.com/AI ; https://optimusk.blog/blog/ai-training-for-tesla-optimus ; https://builtin.com/robotics/tesla-robot — *FSD-derived end-to-end neural nets; shared FSD computer; learned world representation; 8.2B-mile data leverage.* [details largely marketing/secondary]

15. **Agility Digit** — https://www.agilityrobotics.com/content/digits-next-steps ; footstep-planner tech talk https://www.youtube.com/watch?v=VeutCk1xYzI ; https://robotsguide.com/robots/digit — *Redesigned footstep-planner nav stack; deployed in GXO warehouses.*

16. **Figure 01** — https://www.linkedin.com/posts/hokole_yesseveral-commercially-available-humanoid-activity-7392229959148175360-AGNe — *LiDAR + 3D cameras for spatial awareness.*

17. **FAST-LIO2 drift on legged robots (community report)** — https://robotics.stackexchange.com/questions/117993/drift-while-3d-mapping-with-fast-lio-lidar-imu — *Confirms severe drift when robot moves; motivation for leg-odometry fusion.*

18. **Survey: visual perception of humanoid robots** — https://www.sciencedirect.com/science/article/pii/S266737972400055X — *"Humanoid robots experience significant vibrations during walking, and most robotic visual sensors lack built-in stabilization."* Motivates Q1.

19. **Effects of biped humanoid walking gaits on sparse visual SLAM** — Shiguematsu et al., Humanoids 2018. https://www.martimbrandao.com/papers/Shiguematsu2018-humanoids.pdf — *Faster walking reduces drift but increases vibration — fundamental trade-off.*
