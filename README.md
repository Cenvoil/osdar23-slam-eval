# OSDaR23 SLAM evaluation protocol

Code and derived tables for:

> **Evaluating SLAM on OSDaR23: A Reproducible Protocol and Empirical Observations**
> Hai-Chuan Gan, Dan Tao — Beijing Jiaotong University

This repository is the paper-specific evaluation package. It contains the ATE/RPE
scripts, the protocol documents, and the CSV tables used in the paper. It does not
contain OSDaR23 raw data or third-party system binaries.

## Quick start

```bash
pip install -r requirements.txt
python eval_ate.py --gt reference.tum --est estimate.tum --out metrics.json
python rpe_eval.py --gt reference.tum --est estimate.tum
```

`eval_ate.py` reports SE(3)/Sim(3), mean/RMSE, and w2c-t/camera-center. The paper's
main numbers use Horn SE(3), mean, and w2c-t.

## Layout

- `eval_ate.py` — ATE with explicit alignment/aggregation/representation.
- `rpe_eval.py` — translation RPE at Δ=1 and Δ=10, with one constant SE(3) frame alignment.
- `protocol_ATE_Sim3.md`, `protocol_RPE.md` — protocol definitions.
- `method_definition.md` — the system/evaluation conventions.
- `results/` — CSV versions of Tables 2–5 and the failure-mode table.
- `make_failure_figure.py` — failure-evidence figure script.

## Data

OSDaR23 is available from its providers at <https://data.fid-move.de> under their
terms. The reference trajectory is built from `novatel_oem7_inspva` plus the official
camera extrinsic (see `protocol_ATE_Sim3.md`). Raw data and third-party trajectories
are not redistributed here.

## License

MIT for the scripts in this repository. See `LICENSE`.
