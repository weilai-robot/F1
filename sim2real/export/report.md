# SPI identified parameters — X1 pelvis + motor model

Data: 28 clips (round_exc kp40/kd3 + walk_diag)

## Result

| quantity | nominal | identified (raw) | exported (clamped) |
|---|---|---|---|
| mass [kg] | 4.3042 | 6.9734 | 6.9734 |
| com [m] | [0.00252285, -0.00063439, 0.03023409] | [0.059912, 0.19268, -0.197828] | [0.059912, 0.149366, -0.119766] |
| I diag [kg m^2] | [0.0268, 0.0108, 0.0218] | [1.541846, 1.204947, 1.974891] | [0.998802, 0.934137, 0.981672] |
| motor kappa | (see config nominal) | {'hip_pitch': 62.043065284568925, 'hip_rolleyaw': 22.433510687064167, 'knee': 150.08107539479983, 'ankle': 22.076239800066432} | same |
| kappa_s | 1.0 | 0.5359 | same |

Multi-step prediction cost: nominal **3039218.1** -> best **118018.5** (25.8x lower).

## Notes

* clamp: com[1] clamped to +-0.15 m
* clamp: com[2] clamped to +-0.15 m
* clamp: inertia eig [0.9146, 1.5397, 2.2674] -> [0.9146, 1.0, 1.0] (clamped to [0.005,1.0])
* weak observability without mocap: com_y/z and inertia absorb model error; kappas are all in-box and well identified.
* mass_landscape (mass-only scan, others nominal): best 3.71 kg cost 2.15M — mass is correlated with inertia/motor gains in the joint optimum, treat single-parameter scans as diagnostic only.

## Optimization history (tail)

  - 200159.5394880295
  - 611044.9704494574
  - 149436.4757698276
  - 139774.1566350929
  - 435117.351035039
