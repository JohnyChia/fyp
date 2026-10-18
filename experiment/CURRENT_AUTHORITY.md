# Current FYP authority and production release

Status date: 2026-10-18 (Asia/Singapore)  
Status: **SIX-MODEL STUDY COMPLETE; RUN04 V4.2 ACTIVE PRODUCTION**

This file is the additive current-status authority. It supersedes status
statements in older indexes and dated readiness reports without modifying or
invalidating those historical receipts.

## Current study scope

The original formal eight-run factorial claim was withdrawn before final
comparison. Run01–Run06 form the completed deployment-oriented six-model study.
Run07 and Run08 (D-FINE Nano synthetic/hybrid) were cancelled before training
and have no trained weights or performance claims.

The completed rows are:

| Run | Architecture | Training condition | Role |
|---|---|---|---|
| Run01 | YOLO26n | V4.1 synthetic-only | Frozen primary scientific selection; rollback model |
| Run02 | YOLO26n | V4.2 corrected Hybrid | Engineering comparison |
| Run03 | YOLO26n-P2 | V4.1 synthetic-only | Primary scientific selection set |
| Run04 | YOLO26n-P2 | V4.2 corrected Hybrid | Engineering leader; active production |
| Run05 | RF-DETR Nano | V4.1 synthetic-only | Primary scientific selection set |
| Run06 | RF-DETR Nano | V4.2 corrected Hybrid | Excluded from deployment by hard-gate failure |

## V4.2 correction authority

The corrected Hybrid source-family authority is:

`final_cross_domain_study/durian_v4_2_8run_ready/manifests/SOURCE_FAMILY_CORRECTION_V4_2.json`

SHA-256:
`b634760affc11ce33f4dcbda5904aa4c482480824a3650b2d70b9add45a062bd`.

Historical V4.1 and stale V4.1-labelled metadata remain lineage evidence and
must not be substituted for the correction manifest. The sealed-test boundary
has not been used for training, tuning, comparison, selection or promotion.

## Selection and deployment

The frozen primary scientific selection remains **Run01**. Under the common
engineering comparison and deployment hard gates, **Run04** is the highest
eligible engineering model and has been explicitly promoted to ROS2
production. These are different claims and must not be conflated.

Current production identity:

- backend: `fyp_run04_v42`
- checkpoint:
  `/home/johny/durian_training/models_8run/model4_yolo26n_p2_hybrid/best.pt`
- checkpoint SHA-256:
  `04fa8dc9fa7df9bb238ab42469dee857caedb1c36d2d02bdf252bb9e831bc6f5`
- threshold: `0.45`
- device policy: `cuda_required`
- preprocessing: `framework_native_rect_original_pixel_inverse_map`
- adapter: `frozen-common-evaluator-v1`
- active configuration SHA-256:
  `f2d159baeaa135c536b552e2919106fafd455f4f91ee7807ce21d4c27dd9e9c4`
- production release manifest SHA-256:
  `f862721dce84db68668bdc49dc46b57130019144421b93c971ff896e3721ae5a`

Run04 passed exact-checkpoint CUDA, six-tree ROS2/Gazebo inspection, return to
start, captured-image processing, DB/log binding, class/preprocessing,
latency/VRAM and rollback gates. Phase 3 promotion additionally passed 61/61
ROS2 regression tests, active-config CUDA smoke, final-release smoke and a
controlled Run01 CUDA rollback-load drill.

## Rollback authority

Run01 remains the verified rollback target:

- checkpoint SHA-256:
  `b7e221be1716737941f9ac0bfeab832e6fce67f405a1d0f133287ce84e2b4ef0`
- configuration SHA-256:
  `28f157300d187ad63e57a1658fca3cfd020c02647bc402b5db3c3cb376152005`
- rollback snapshot: 8/8 files hash-verified
- rollback CUDA load drill: PASS

Recovery instructions and immutable snapshot evidence are under
`final_cross_domain_study/fyp_training_and_real_gt_handoff/post_training_evidence/v42_corrected_comparison_20261018/run04_production_promotion_preflight_20261018/rollback_run01/`.

## Current authoritative evidence chain

1. `final_cross_domain_study/durian_v4_2_8run_ready/FINAL_VALIDATION_REPORT.json`
2. `final_cross_domain_study/durian_v4_2_8run_ready/manifests/SOURCE_FAMILY_CORRECTION_V4_2.json`
3. `final_cross_domain_study/fyp_training_and_real_gt_handoff/FYP_PROPOSAL_DEPLOYMENT_DECISION_POLICY.json`
4. `final_cross_domain_study/fyp_training_and_real_gt_handoff/post_training_evidence/v42_corrected_comparison_20261018/FINAL_MODEL_REGISTRY_V42.json`
5. `final_cross_domain_study/fyp_training_and_real_gt_handoff/post_training_evidence/v42_corrected_comparison_20261018/FINAL_GATE_MATRIX_POST_RUN04_E2E.csv`
6. `final_cross_domain_study/fyp_training_and_real_gt_handoff/post_training_evidence/v42_corrected_comparison_20261018/run04_ros2_gazebo_e2e_20261018_attempt02/RUN04_FINAL_E2E_GATE_RECEIPT.json`
7. `final_cross_domain_study/fyp_training_and_real_gt_handoff/post_training_evidence/v42_corrected_comparison_20261018/run04_production_promotion_20261018/RUN04_V42_PRODUCTION_RELEASE_MANIFEST.json`
8. `final_cross_domain_study/fyp_training_and_real_gt_handoff/post_training_evidence/v42_corrected_comparison_20261018/run04_production_promotion_20261018/RUN04_PRODUCTION_PROMOTION_RECEIPT.json`

## Superseded status documents

`EXPERIMENT_INDEX.md`, `SIX_MODEL_SCOPE_AMENDMENT.json`,
`FINAL_DOUBLE_CHECK_REPORT.md`, `COMPREHENSIVE_DIAGNOSIS_20261010.md`, the
pre-E2E `V42_SIX_MODEL_FINAL_COMPARISON.md`, and
`FINAL_DECISION_RECEIPT.json` remain historical evidence. Their earlier
pending, V4.1-Hybrid or Run04-blocked status statements are superseded by the
current V4.2 registry, final E2E receipt and production promotion receipt.
They must not be overwritten or deleted.

## Scientific and operational limitations

- The AI-assisted 58-image Real reference is supporting FYP evidence, not
  independent expert ground truth.
- V4.1 synthetic-only and V4.2 corrected-Hybrid runs are useful engineering
  comparisons but are not a clean same-dataset causal estimate of Hybrid
  training effect.
- The sealed test remains unopened; no sealed-test performance claim exists.
- Run04 is verified on the tested RTX 3050 Ti workstation. Other production
  hardware remains unverified.
- The active checkpoint is an absolute external path under
  `/home/johny/durian_training`; moving it breaks deployment until the config
  and release manifest are deliberately revised and revalidated.
- The working tree contains preserved modified/untracked ROS2 and experiment
  material. Final GitHub publication was separately approved through the
  controlled 127-file release gate; Git history is the publication authority.
- Workspace cleanup is not authorized by this document. A verified off-device
  recovery copy is required before destructive cleanup.
