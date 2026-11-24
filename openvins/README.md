# OpenVINS visual-inertial probe

Supplementary monocular visual-inertial experiment reported in Appendix F of the
paper. It is **not** one of the nine systems in the main comparison.

## Setup

- OpenVINS commit `6948812` (ROS1, `run_subscribe_msckf`), stock code.
- Camera: `rgb_highres_center`, 4112×2504, downsampled 0.25× to 1028×626
  (fx = fy = 1816.99 px, cx = 514.01, cy = 371.72), pinhole/radtan.
- IMU: `novatel_oem7_corrimu`. Each 10 Hz row is a **0.01 s increment**, not a
  0.1 s rate: summing the yaw increments over 3.1 gives 0.015 rad, which matches
  the 0.014 rad heading change; treating them as 0.1 s rates gives 0.0015 rad.
  The player divides by 0.01 s and holds the rate over the 0.1 s interval at
  100 Hz. Accelerations are gravity-compensated, so +9.81 m/s² is re-added along
  the body z axis.
- Camera-IMU rotation: official `labels.json` camera pose composed with the
  `corrimu` frame (x lateral, y longitudinal, z vertical). OSDaR23 publishes no
  IMU extrinsic, so the translation is set to zero.
- Initialization: the three sequences turn by only 0.1°/0.0°/0.6° in 10 s and
  have no stationary start, so the stock static initializer (needs a stationary
  start) and dynamic initializer (needs ≥10° rotation) both fail. We set
  `init_max_disparity = 1000` and enable ZUPT so the static path is taken at the
  first frame; the reported trajectory starts after the 2 s initialization
  window (75/100 frames on every sequence).
- Runs on CPU; no GPU is used.

## Results

SE(3) mean, camera-center; Sim(3) is a shape diagnostic because the mileage
ratios on 3.1/5.1 are outside [0.9, 1.1].

| Sequence | Matched frames | ATE SE(3) (cm) | SE(3) RMSE (cm) | Sim(3) (cm), s* | Mileage ratio |
|---|---:|---:|---:|---|---:|
| 1.2 | 75/100 | 137.18 | 164.56 | 25.44, 0.769 | 1.288 |
| 3.1 | 75/100 | 1471.42 | 1678.89 | 1036.16, 5.630 | 0.175 |
| 5.1 | 75/100 | 1623.60 | 1870.73 | 267.17, 6.733 | 0.147 |

## Files

- `estimator_config.yaml` — OpenVINS estimator configuration (mono, 10 Hz IMU
  upsampled to 100 Hz, CLAHE, online camera extrinsics).
- `kalibr_imu_chain.yaml`, `kalibr_imucam_chain.yaml` — IMU noise and
  camera-IMU chain.
- `osdar_vio_player.py` — publishes `/imu0` and `/cam0/image_raw` from a
  sequence directory.
- `run_vio.sh` — starts roscore, OpenVINS, the pose recorder, and the player.
  Set `ROS_ENV` and `OPENVINS_WS` if your environments are not in the defaults.
- `../results/openvins/` — TUM trajectories, per-sequence metric JSON, and
  `summary.csv`.
