# FYP final release index

Status: **FINAL RELEASE — CONTROLLED MAIN PUBLICATION APPROVED**

This directory is a lightweight index for the final FYP source release. It is
not a backup and contains no dataset, checkpoint, runtime database, captured
image, processed image, log, virtual environment, build product, or sealed-test
sample.

## Release identity

- GitHub target: `JohnyChia/fyp`, branch `main`
- verified remote base: `ad0dbbe54ab6b323627e734d4b8a178e6e827336`
- model authority: `JohnyChia/durian_training`, branch
  `model-synthetic-and-hybrid`, commit
  `d673a31b09543fb4e7aca4bb10bb22b6bfc76bd1`
- active backend: `fyp_run04_v42`
- active configuration SHA-256:
  `f2d159baeaa135c536b552e2919106fafd455f4f91ee7807ce21d4c27dd9e9c4`
- Run04 checkpoint SHA-256:
  `04fa8dc9fa7df9bb238ab42469dee857caedb1c36d2d02bdf252bb9e831bc6f5`

The source workspace's local `main` was not based on `fyp/main` and was
incorrectly configured to track the `durian_training` remote. A normal push
from that branch was therefore forbidden. The approved release procedure
creates the release commit directly on the verified `fyp/main` parent without
checking out a second worktree, copying the workspace, merging unrelated
history, or force pushing. `PRE_PUSH_AUDIT.md` preserves the approval-gate state
before publication; the published commit identity is recorded by Git history.

## Authoritative evidence selected for publication

The final release intentionally publishes only small, essential evidence at
its original paths:

1. `experiment/CURRENT_AUTHORITY.md`
2. V4.2 final validation and source-family correction manifests
3. the frozen FYP deployment decision policy and common evaluator
4. the V4.2 six-model registry, common comparison, uncertainty/domain-shift
   summaries, post-E2E gate matrix and decision receipt
5. the exact Run04 CUDA/ROS2/Gazebo E2E gate receipt and PASS report
6. the Run04 production release manifest, promotion receipt/report and final
   production gate matrix
7. the Run01 rollback snapshot manifest, recovery instructions and CUDA load
   receipt

Historical reports that show an earlier BLOCKED state remain evidence, but the
current authority and production-promotion receipt supersede their status. No
historical receipt is overwritten.

## Model and recovery contract

`MODEL_ARTIFACT_REFERENCES.json` records all six checkpoint identities without
copying model bytes. Restore the model repository at the recorded commit, run
Git LFS materialization there, and verify every selected checkpoint SHA-256.
The active ROS2 YAML must hash to the recorded production value before launch.

Run01 remains the rollback target. Follow the published recovery instructions,
verify the 8/8 snapshot manifest, verify checkpoint SHA-256
`b7e221be1716737941f9ac0bfeab832e6fce67f405a1d0f133287ce84e2b4ef0`,
and run a CUDA load check. Rollback metadata is published; the checkpoint bytes
remain in the model repository and are not duplicated here.

## Scope and limitations

- Run01/03/05 are retained V4.1 synthetic-only models; Run02/04/06 use the
  corrected V4.2 Hybrid dataset.
- Run07/08 were cancelled before training; no weights or metrics exist.
- Run04 is the engineering/deployment selection. Run01 remains the frozen
  primary scientific selection.
- The real reference is small and AI-assisted, not expert ground truth.
- The sealed test remains unopened and is excluded from this release.
- Deployment certification is hardware-specific to the tested RTX 3050 Ti
  workstation.

## Deliberate exclusions

The release excludes datasets, sealed-test material, all checkpoint bytes,
raw captures, processed images, SQLite databases, logs, caches, environments,
build/install trees, temporary files, local backup bundles, and the stale V4.1
`fyp_run02.json`, `fyp_run04.json`, and `fyp_run06.json` manifests.

See `RELEASE_MANIFEST.json` for the machine-readable authority and exclusion
contract. `SHA256SUMS` binds every selected source/evidence file except itself.
The controlled commit/push was separately and explicitly approved.
