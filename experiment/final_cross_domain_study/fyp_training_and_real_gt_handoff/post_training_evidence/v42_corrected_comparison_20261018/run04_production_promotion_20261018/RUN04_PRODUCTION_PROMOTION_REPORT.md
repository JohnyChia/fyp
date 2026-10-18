# Run04 V4.2 production promotion result

Status: **PASS — RUN04 IS THE ACTIVE PRODUCTION MODEL**

Run04 YOLO26n-P2 V4.2 Hybrid was promoted under the explicitly approved,
SHA-bound transaction. The final active backend is `fyp_run04_v42`; the exact
checkpoint SHA-256 is
`04fa8dc9fa7df9bb238ab42469dee857caedb1c36d2d02bdf252bb9e831bc6f5`.

The approved preflight YAML (`80368b54...bc5ed`) was first installed
byte-for-byte. After all tests passed, only its manifest path was finalized to
the immutable production release manifest. The resulting final active config
SHA-256 is
`f2d159baeaa135c536b552e2919106fafd455f4f91ee7807ce21d4c27dd9e9c4`.

## Gate result

| Gate | Result |
|---|---|
| Critical pre-promotion SHA checks | PASS |
| Run04 checkpoint identity | PASS |
| Threshold 0.45 / no retuning | PASS |
| `cuda_required` | PASS |
| Six-class mapping | PASS |
| Frozen preprocessing/adapter | PASS |
| 48-sample CUDA resource audit | PASS |
| Active-config backend inference | PASS |
| Final-release-manifest inference | PASS |
| Non-finite/coordinate validity | PASS, 0/0 |
| ROS2 configuration regression | PASS, 61/61 twice |
| DB/log/receipt SHA bindings | PASS |
| Run01 rollback snapshot | PASS, 8/8 |
| Run01 rollback CUDA load drill | PASS, 2/2 |
| Restore Run04 after rollback drill | PASS |
| Sealed-test boundary | PASS |

GPU audit: RTX 3050 Ti, 48 samples, p90 48.6293 ms, peak allocated/reserved
VRAM 85.74/104.00 MB. The final production backend smoke completed three
inference requests with consistent output and exact class/preprocessing
bindings.

## Fail-closed event retained

The first new backend-smoke attempt successfully started the worker and ran
inference, then the test harness referenced a nonexistent `LeafDetection.xyxy`
property. The model hard gate was not reached. Run01 was immediately restored
to exact SHA `28f157...52005`. The harness was corrected to use the dataclass
fields `x1/y1/x2/y2`, all identity checks were repeated, and attempt02 passed.
The failure receipt is preserved; it was not hidden or overwritten.

## Rollback

Run01 remains fully recoverable:

- config SHA: `28f157300...52005`
- checkpoint SHA: `b7e221be1...b4ef0`
- snapshot: 8/8 files verified
- controlled CUDA load: 2/2 inference requests passed

The rollback drill ended by restoring Run04 as active and verifying the final
Run04 configuration and manifest again.

## Remaining risks

1. Run04's checkpoint is still referenced by an absolute path under
   `/home/johny/durian_training`; moving that repository will break runtime.
2. The working tree contains intentionally preserved pre-existing modified and
   untracked ROS2/model-runner files and has not been committed.
3. Phase 3 did not repeat the full 22-minute Gazebo mission. It relies on the
   earlier exact-checkpoint six-tree E2E PASS plus new production CUDA/backend,
   DB/log and rollback tests.
4. Deployment on hardware other than the tested RTX 3050 Ti remains unverified.

No training, threshold tuning, sealed-test access, dataset/checkpoint mutation,
cleanup, deletion, commit or push occurred.
