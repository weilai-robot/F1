## Sources Consulted

1. **Wang et al., "End-to-End Humanoid Robot Safe and Comfortable Locomotion Policy,"** arXiv:2508.07611, 2025.
   - URL: https://arxiv.org/html/2508.07611v1
   - Takeaway: P3O-CBF achieves 86% success on dynamic agents, 53% reduction in unsafe-space time vs PPO, deployed on Unitree G1.
   - Key excerpt: "An end-to-end locomotion policy that directly maps raw, spatio-temporal LiDAR point clouds to motor commands... Our key contribution is a novel methodology that translates the principles of Control Barrier Functions (CBFs) into costs within the CMDP."
   - Key excerpt (reward weights): "Z velocity: -3×10⁻⁴, Proxemic Comfort: weight 1.5, d_social = 1.2m"
   - Key excerpt (results): "Dynamic Agents: PPO 56%, P3O 70%, P3O-CBF 86%"

2. **Weerakoon et al., "VAPOR: Legged Robot Navigation in Unstructured Outdoor Environments using Offline RL,"** arXiv:2309.07832, 2023.
   - URL: https://arxiv.org/html/2309.07832
   - Takeaway: Offline RL with intensity+height cost maps from LiDAR achieves 40% improvement in success rate on Spot robot in vegetation.
   - Key excerpt: "Our policy uses height and intensity-based cost maps derived from 3D LiDAR point clouds... improves success rates by up to 40%, decreases the average current consumption by up to 2.9%."

3. **Fuke et al., "Towards Local Minima-free Robotic Navigation: MPPI via Repulsive Potential Augmentation,"** arXiv:2410.11379, 2025.
   - URL: https://arxiv.org/html/2410.11379v2
   - Takeaway: RPA-MPPI eliminates local minima without global path replanning; minimal cost function modification.
   - Key excerpt: "The key idea is repulsive potential augmentation, integrating high-level directional information into the MPPI framework as a single repulsive term through an artificial potential field."

4. **Wang et al., "Super-LIO: A Robust and Efficient LiDAR-Inertial Odometry System,"** arXiv:2509.05723, 2025.
   - URL: https://arxiv.org/html/2509.05723v1
   - Takeaway: OctVox map achieves 3.7× (X86) and 4.2× (ARM) speedup over FAST-LIO2 with competitive accuracy.
   - Key excerpt: "Super-LIO processes each frame approximately 73% faster than SOTA, while consuming less CPU resources... ARM Orin NX: ~17ms/frame vs FAST-LIO2 ~80ms/frame."
   - Code: https://github.com/Liansheng-Wang/Super-LIO.git

5. **Wang et al., "Omni-Perception: Omnidirectional Collision Avoidance for Legged Locomotion,"** arXiv:2505.19214, 2025.
   - URL: https://arxiv.org/html/2505.19214
   - Takeaway: PD-RiskNet processes raw LiDAR for 360° collision avoidance; 70% success on aerial obstacles vs 0% for native system.
   - Key excerpt: "PD-RiskNet partitions raw point cloud into proximal (FPS sampling → GRU, 187 features) and distal (average downsampling → GRU, 64 features) subsets."
   - Key excerpt (results): "Aerial obstacles: Omni-Perception 70% vs Unitree System 0%; Moving humans: 90% vs 0%"

6. **Miki et al., "Elevation Mapping for Locomotion and Navigation using GPU,"** IROS 2022. ETH Zurich.
   - URL: https://github.com/leggedrobotics/elevation_mapping_cupy
   - Takeaway: GPU-accelerated 2.5D elevation mapping for legged robots; Multi-Modal Elevation Map framework.
   - Key excerpt: "Multi-Modal Elevation Map (MEM) Framework: Allows seamless integration of diverse data like geometry, semantics, and RGB information."

7. **Nav2 Documentation — Voxel Layer Parameters & MPPI Controller,** Nav2 1.0.0.
   - URL: https://docs.nav2.org/configuration/packages/costmap-plugins/voxel.html
   - URL: https://docs.nav2.org/configuration/packages/configuring-mppic.html
   - Takeaway: Voxel layer maintains 3D model but squashes to 2D for planning; MPPI is Nav2's official local planner.
   - Key excerpt: "This costmap layer implements a plugin that uses 3D raycasting... It contains a 3D environmental model within it that manages the planning space and squashes down to 2D for planning and control."

8. **Radosavovic et al., "Real-world humanoid locomotion with reinforcement learning,"** Science Robotics, 2024.
   - DOI: 10.1126/scirobotics.adi9579
   - URL: https://arxiv.org/abs/2303.03381
   - Takeaway: Transformer-based RL achieves blind humanoid locomotion over 4+ miles of hiking trails.
   - Key excerpt (from search): "A fully learning-based approach for real-world humanoid locomotion... transformer model to predict the next action based on the history of proprioceptive observations."

9. **Uthai et al., "Opportunities, challenges and roadmap for humanoid robots in construction,"** Scientific Reports, 2025. PMC12783740.
   - URL: https://pmc.ncbi.nlm.nih.gov/articles/PMC12783740
   - Takeaway: Comprehensive survey of humanoid robot methods, platforms, and challenges.
   - Key excerpt (Table 1): "Deep RL locomotion: Simulation-trained walking policies with real-world tuning → Requires hardware-specific tuning; computationally intensive"

10. **LiDAR Odometry Benchmark,** ISPRS Archives, 2025.
    - URL: https://isprs-archives.copernicus.org/articles/XLVIII-1-W6-2025/25/2025/
    - Takeaway: Comparison of DLO, DLIO, FASTER-LIO, FAST-LIO2, Point-LIO for low-cost platforms.

11. **Berkeley Humanoid — Learning-Based Control Platform,** 2025.
    - URL: https://berkeley-humanoid.com
    - Takeaway: Low-cost mid-scale humanoid platform designed for narrow sim-to-real gap.
    - Key excerpt: "Our research platform enables state-of-the-art robust outdoor experiments over various terrains with a simple reinforcement learning controller using light domain randomization."

12. **Unitree G1/H1 ROS2 SDK and Specs,** 2025.
    - URL: https://www.unitree.com/cn/opensource
    - Takeaway: Full ROS2 support for Unitree humanoid platforms; H1 achieves 3.3 m/s bipedal running speed.