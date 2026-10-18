# FYP GitHub main pre-push audit

Audit date: 2026-10-18 (Asia/Singapore)  
State: **PREPARED — COMMIT/PUSH APPROVAL REQUIRED**

## Decision

**CONDITIONAL GO** for one controlled, non-force release commit whose parent is
the verified `fyp/main` commit and whose tree changes are restricted exactly to
`PUBLISH_FILESET.txt`.

**NO-GO** for `git push` from the current local `main`. The local branch is
configured to track `origin/main`, but `origin` is the `durian_training`
repository. Local `main` and `fyp/main` have no merge base. A normal push,
merge of unrelated histories, or force push would be unsafe.

No commit or push has been performed. Explicit user approval is still required.

## Verified repositories

| Role | Remote/branch | Verified head |
|---|---|---|
| FYP release target | `fyp` → `JohnyChia/fyp`, `main` | `ad0dbbe54ab6b323627e734d4b8a178e6e827336` |
| Model authority | `JohnyChia/durian_training`, `model-synthetic-and-hybrid` | `d673a31b09543fb4e7aca4bb10bb22b6bfc76bd1` |

The FYP head was refreshed into `.git/FETCH_HEAD` at 2026-10-18 17:13 +08:00.
The model branch identity matches the frozen six-model registry.

## Production and rollback identity

| Gate | Result | Evidence |
|---|---|---|
| active backend | PASS | `fyp_run04_v42` |
| active config | PASS | SHA-256 `f2d159baeaa135c536b552e2919106fafd455f4f91ee7807ce21d4c27dd9e9c4` |
| Run04 checkpoint | PASS | SHA-256 `04fa8dc9fa7df9bb238ab42469dee857caedb1c36d2d02bdf252bb9e831bc6f5` |
| threshold/device | PASS | `0.45`, `cuda_required` |
| production release manifest | PASS | SHA-256 `f862721dce84db68668bdc49dc46b57130019144421b93c971ff896e3721ae5a` |
| promotion receipt | PASS | SHA-256 `1e9b4c668bbe0f4dd688fbed137cb04d29b6e1192d757a3a6047365904800a19` |
| Run01 rollback snapshot | PASS | 8/8 files; manifest SHA-256 `735ce1e6e12bbba0be9c4cf1230e427a2c423e14c0711e792a7396c608ef486b` |
| sealed-test boundary | PASS | no sealed sample selected or opened |

All six selected model checkpoint hashes match the frozen registry and the
materialized Git LFS objects in the model repository. Run07/08 remain correctly
recorded as cancelled before training.

## Proposed upload

- exact file count: 127
- new files relative to `fyp/main`: 106
- modified files relative to `fyp/main`: 21
- deletions: 0
- uncompressed selected bytes: 1,349,540 bytes (about 1.29 MiB)
- new checkpoint/model bytes: 0
- new Git LFS bytes: 0
- dataset bytes: 0
- DB/log/captured/processed image bytes: 0

The upload is source, configuration, small manifests, CSV/JSON receipts, one
small SVG result visualization, and documentation only. Git transport
compression will normally make the network transfer smaller than the raw blob
total, but no compressed size is claimed before Git creates the objects.

Expected additional local storage for the release commit is bounded by the
selected raw blob total plus small Git object/commit overhead (under 2 MB before
any repository housekeeping). No archive, clone, worktree, checkpoint copy,
dataset copy, or local backup is created.

## Source and evidence review

- README now documents architecture, six-model methodology, V4.2 correction,
  separate scientific/engineering decisions, active Run04, rollback, and
  limitations.
- `PROJECT_INDEX.md` no longer claims training or selection is pending.
- the sealed-test-safe validator now verifies the exact active config,
  checkpoint, threshold, device policy and model-repository commit.
- stale V4.1 `fyp_run02.json`, `fyp_run04.json`, and `fyp_run06.json` are
  explicitly excluded. The active profile uses only the V4.2 production
  release manifest.
- historical pre-E2E reports remain unchanged and are labelled superseded by
  `experiment/CURRENT_AUTHORITY.md`; no receipt is rewritten.
- ignored backup bundles, raw evidence, databases, logs, datasets, caches,
  build products and local inspection helper scripts are excluded.

## Validation results

| Check | Result |
|---|---|
| sealed-test-safe development validator | PASS; 0 failures |
| ROS2/Python package tests | PASS; 61/61 |
| Flutter tests | PASS; 6/6 |
| frozen post-training/evaluator tests | PASS; 16/16 in pinned Phase2A runtime |
| JSON syntax for release metadata | PASS |
| `git diff --check` | PASS |
| selected-file secret scan | PASS; no token/private-key/password pattern |
| PII review | PASS; only dummy ROS maintainer address and Git SSH syntax |
| forbidden binary extension scan | PASS |
| current-session CUDA recheck | BLOCKED_ENVIRONMENT; `nvidia-smi` unavailable |

The current-session CUDA blocker does not replace the immutable prior Run04
CUDA/Gazebo E2E and production-promotion PASS receipts. It prevents claiming a
new CUDA execution in this pre-push audit.

## Controlled commit method after approval

1. Re-fetch `fyp/main` and require it still equals the approved base. Stop if
   the remote moved.
2. Use a temporary Git index under `/tmp`, initialized from that exact remote
   tree. Do not switch branches, alter the normal index, stash, clone, create a
   worktree, or touch unrelated dirty/untracked files.
3. Add only the 127 paths in `PUBLISH_FILESET.txt`, preserving executable bits.
4. Verify the candidate tree contains no file outside the approved list, no
   selected path is missing, and all production/evidence hashes still match.
5. Create one commit with parent `ad0dbbe54ab6b323627e734d4b8a178e6e827336`.
6. Push that commit to `fyp/main` without force. Stop on any non-fast-forward.
7. Fetch again and verify the remote main tree/commit and key hashes.

Final authorization gate: **WAITING FOR EXPLICIT APPROVAL TO COMMIT AND PUSH**.
