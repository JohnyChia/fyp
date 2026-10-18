# Run01 rollback recovery instructions

These instructions are prepared evidence only. Do not execute them unless a
separately approved Run04 promotion fails or an approved rollback drill is in
progress.

1. Stop the active inspection launch and confirm no writer is using the active
   deployment YAML or inspection database.
2. Hash the rollback snapshot and require every entry in
   `RUN01_ROLLBACK_SNAPSHOT_MANIFEST.json` to pass.
3. Verify the canonical Run01 checkpoint still exists at
   `/home/johny/durian_ws/experiment/analysis/dev_validation_audit/phase2b/artifacts/run01/best.pt`
   with SHA-256
   `b7e221be1716737941f9ac0bfeab832e6fce67f405a1d0f133287ce84e2b4ef0`.
   If it is missing or differs, restore it from `checkpoints/run01_best.pt`
   using a temporary file, verify the SHA, and only then atomically replace the
   failed canonical copy.
4. Restore the active deployment YAML from
   `src/durian_inspection_pkg/config/leaf_detector_deployment.yaml` in this
   snapshot using a temporary file and atomic rename. Require SHA-256
   `28f157300d187ad63e57a1658fca3cfd020c02647bc402b5db3c3cb376152005`.
5. Do not overwrite the runtime/launch source files unless their hashes changed
   as part of the separately approved promotion. They are included to identify
   the exact tested working deployment, not to erase unrelated future work.
6. Parse the restored YAML and require `leaf_detector.backend: fyp_run01`.
7. Load and validate the Run01 manifest, exact checkpoint SHA, class table,
   threshold 0.40, preprocessing and `cuda_required` policy.
8. Run configuration regression and a bounded CUDA/backend smoke test. If the
   failure happened during a full mission, use a new isolated DB/log/capture
   path for any rollback verification; never overwrite production evidence.
9. Generate a create-only rollback receipt containing before/after hashes,
   reason, operator approval and test results.

Never use `git reset --hard`, `git checkout --`, `git clean`, force-push or a
broad recursive copy for this rollback.
