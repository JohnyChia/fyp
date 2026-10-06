# Durian FYP project index

1. Production ROS 2 system: `src/`, `run.sh`, `config/`, runtime databases and image folders.
2. Frozen legacy models: `models/leaf_candidates/` (four hash-locked backends).
3. Legacy datasets: `experiment/datasets/` (frozen; sealed test remains closed).
4. Dataset V4 engineering: `experiment/dataset_v4_spec/` and `experiment/dataset_v4_tools/`.
5. Real data: external `/mnt/c/fyp basic models/fyp_datasets/durian_leaf`, READ_ONLY; candidate labels live under the final study.
6. Hybrid study and final eight-run package: local-only under `experiment/final_cross_domain_study/`.
7. Model remediation evidence: `experiment/analysis/` and `experiment/colab_preparation/`.
8. Final cross-domain study: `experiment/final_cross_domain_study/`.
9. Development state: `DEVELOPMENT_READY_FOR_FINAL_MODEL`; CUDA-native smoke is environment-blocked on the current machine.
10. Exact next step: complete formal runs externally, select the final model from formal results, then supply its validated deployment YAML.
