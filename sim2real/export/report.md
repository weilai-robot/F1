# SPI identified parameters — X1 pelvis + motor model

Data: None clips (round_exc kp40/kd3 + walk_diag)

## Result

| quantity | nominal | identified (raw) | exported (clamped) |
|---|---|---|---|
| mass [kg] | 4.3042 | 3.7831 | 3.7831 |
| com [m] | [0.00252285, -0.00063439, 0.03023409] | [0.001213, -0.008196, -0.016179] | [0.001213, -0.008196, -0.016179] |
| I diag [kg m^2] | [0.0268, 0.0108, 0.0218] | [0.0415, 0.133621, 0.112001] | [0.0415, 0.133621, 0.112001] |
| motor kappa | (see config nominal) | {'hip_pitch': 71.09175899234774, 'hip_rolleyaw': 30.43982673952897, 'knee': 89.44574326343829, 'ankle': 13.141874141303514} | same |
| kappa_s | 1.0 | 0.4340 | same |

Multi-step prediction cost: nominal **724826.6** -> best **167958.7** (4.3x lower).

## Notes

* no clamping applied (--no-clamp or in-box values)
* weak observability without mocap: com_y/z and inertia absorb model error; kappas are all in-box and well identified.
* mass_landscape (mass-only scan, others nominal): best 3.71 kg cost 2.15M — mass is correlated with inertia/motor gains in the joint optimum, treat single-parameter scans as diagnostic only.

## Optimization history (tail)


