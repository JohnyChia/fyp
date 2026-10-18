# V4.2 corrected six-model comparison and deployment decision

Status: **COMPARISON COMPLETE; RUN04 DEPLOYMENT SWITCH BLOCKED**

No model was retrained, no checkpoint or dataset was modified, thresholds were
selected only on the common Synthetic Val cohort, and no sealed-test outcome was
accessed. The V4.2 Hybrid lineage is bound to correction-manifest SHA-256
`b634760affc11ce33f4dcbda5904aa4c482480824a3650b2d70b9add45a062bd`
and Git commit `d673a31b09543fb4e7aca4bb10bb22b6bfc76bd1`.

## Decision

- **Primary scientific selection:** Run01. This preserves the original frozen
  V4.1 Synthetic-control selection scope; it does not claim that Run01 is the
  strongest corrected engineering model.
- **Engineering highest-scoring model:** Run04 (V4.2 Hybrid YOLO26n-P2),
  provisional. It leads Synthetic/Gazebo Dev at **0.5613**, is within **0.0045**
  of the AI-assisted Real-reference leader, has **0.9667** perturbation
  retention, and passed CPU output/repeat/coordinate checks.
- **AI-assisted Real-reference leader:** Run02 at **0.2454**, versus Run04 at
  **0.2409**. This 58-image reference is not independent expert ground truth,
  so it is a secondary diagnostic and cannot override the frozen
  Synthetic/Gazebo-first policy.
- **Current deployable model:** Run01 on the previously tested workstation.
- **Switching deployment to Run04:** **NO-GO / BLOCKED**, until the exact Run04
  SHA passes actual CUDA ROS2/Gazebo end-to-end execution and GPU resource
  measurement. The active ROS2 profile remains Run01.

## Common-cohort results

| Run | Training condition | Threshold | Synthetic/Gazebo Dev mAP50-95 | AI Real HIGH/MOD mAP50-95 | ECE | Perturbation retention | CPU p90 ms | RSS MB | Eligibility |
|---|---|---:|---:|---:|---:|---:|---:|---:|---|
| Run01 | V4.1 Synthetic | 0.40 | 0.5193 | 0.0091 | 0.0807 | 0.5000 | 77.75 | 601.53 | PASS, historical tested-workstation receipt |
| Run02 | V4.2 Hybrid | 0.40 | 0.5455 | **0.2454** | 0.0832 | 0.9143 | 90.77 | 622.26 | BLOCKED: new-SHA E2E |
| Run03 | V4.1 Synthetic | 0.45 | 0.5133 | 0.0236 | 0.0738 | 0.4429 | 121.62 | 634.08 | PASS, historical tested-workstation receipt |
| Run04 | V4.2 Hybrid | 0.45 | **0.5613** | 0.2409 | 0.0832 | **0.9667** | 121.10 | 666.19 | BLOCKED: new-SHA E2E |
| Run05 | V4.1 Synthetic | 0.45 | 0.4956 | 0.0619 | 0.0669 | 0.7882 | 239.67 | 1026.41 | PASS, historical tested-workstation receipt |
| Run06 | V4.2 Hybrid | 0.50 | 0.5003 | 0.2403 | 0.0655 | 1.0000 | 217.67 | 1031.79 | **FAIL** |

## Hard gates and integrity

Run02 and Run04 produced no non-finite outputs, no invalid coordinates and no
repeat mismatch in the new CPU audit. Run06 is explicitly excluded: it produced
four coordinate violations, and its published EMA checkpoint is not proven to
be the global best over E1-E200. No silent promotion is allowed.

Run02 completed 200 epochs and its global-best checkpoint is identifiable, but
its effective `nbs=64` and `patience=200` differ from the canonical recipe.
Run04 completed 200 epochs and its global-best checkpoint is identifiable, but
its effective `patience=0` differs from the canonical recipe. Neither published
YOLO artifact set machine-binds its starting initializer SHA-256. These facts do
not invalidate the observed inference scores, but they prevent claiming a clean
single-variable causal comparison of V4.1 Synthetic versus V4.2 Hybrid.

The ROS2 loader requires `device_policy: cuda_required`. CUDA/NVML was
unavailable during this audit; therefore GPU latency, VRAM, and actual new-weight
ROS2/Gazebo E2E are **BLOCKED**, not PASS. The historical status JSON reports a
tested-workstation pass for the older exact checkpoints, but its referenced raw
Gazebo summary is no longer present at the recorded relative path; this evidence
gap is declared rather than hidden.

## Reliability, domain shift and uncertainty

Run04's weakest Synthetic Dev class is `leaf_blight` (AP50-95 **0.5348**).
Its small-object AP is **0.1557**, higher than Run02 (**0.1321**), while small
objects remain the main weakness. On the AI-assisted reference, Run04 recall is
lowest for `leaf_rhizoctonia` (0.1905) and `leaf_blight` (0.2353).

The paired 2,000-replicate physical-tree bootstrap on Synthetic Dev macro-F1
gives Run04 minus Run02 = **+0.00975**, 95% CI **[+0.00402, +0.01600]**.
This quantifies only Synthetic Dev sampling uncertainty; it excludes training
seed, real-domain and annotation uncertainty and is not a COCO-mAP interval.

All six models show substantial domain shift. Run04's AI-reference-to-Synthetic
ratio is 0.429, so neither its Gazebo score nor the AI-assisted reference alone
supports a claim of production real-world accuracy.

## Verification

- ROS2 package tests: **61 passed**.
- Frozen comparison/tooling tests: **16 passed** in the pinned Phase2A
  environment (`Python 3.10.12`, `pycocotools 2.0.10`, `numpy 2.2.6`).
- `git diff --check`: **PASS**.
- The workspace already contained unrelated modified/untracked ROS2 and receipt
  files; none was reset, deleted, committed or pushed by this comparison.

## Required next action

When CUDA is available, run the exact Run04 checkpoint
`04fa8dc9fa7df9bb238ab42469dee857caedb1c36d2d02bdf252bb9e831bc6f5`
through the existing ROS2 start-inspection, six-tree return, image-processing,
database/log receipt flow. Verify manifest binding, class order, preprocessing,
GPU latency/VRAM and zero invalid geometry. If every frozen hard gate passes,
Run04 becomes the recommended deployment model; otherwise keep Run01 and record
the failed gate explicitly.
