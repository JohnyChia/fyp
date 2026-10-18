# Run04 V4.2 final ROS2/Gazebo E2E result

Status: **PASS — ELIGIBLE; EXPLICIT PROMOTION APPROVAL REQUIRED**

Run04 checkpoint `04fa8dc9fa7df9bb238ab42469dee857caedb1c36d2d02bdf252bb9e831bc6f5` passed every frozen engineering deployment
gate without changing the active Run01 configuration or retuning threshold
0.45. No sealed-test data was accessed.

## Verified result

- GPU: `NVIDIA GeForce RTX 3050 Ti Laptop GPU`; 48 samples; p90 **37.79 ms**;
  peak allocated/reserved VRAM **85.74/104.00 MB**.
- GPU output validity: 0 non-finite values, 0 coordinate violations.
- Mission: start accepted, all six trees inspected with 4/4 valid viewpoints,
  return home succeeded (0.09 m, 0.16 rad), deferred processing completed.
- Database: 6/6 tree rows completed, 1369/1369 observations processed, exact
  backend `fyp_run04_v42`, exact checkpoint SHA, zero processing errors.
- Receipts/logs: checkpoint, class mapping, 640x640 preprocessing, threshold,
  pipeline fingerprint and output paths are bound and hash-verified.

Two shutdown-only log messages record `processing_node` and `gzserver` requiring
SIGTERM after the bounded SIGINT grace period. They occurred after terminal
COMPLETED and receipt/DB finalization; no process remains and no evidence was
lost. They are retained as a non-gating cleanup warning.

## Decision

Run04 is now the **recommended deployment model** under the frozen engineering
policy. Run01 remains active because automatic promotion was forbidden. Run02
fallback is not required and Run06 was not run.

Promotion, if approved, must be a separate controlled change that adds a
production Run04 V4.2 manifest/profile, runs configuration regression tests,
and preserves rollback to Run01.
