# Leaf candidate staging area

This directory is reserved for the four validation-selected trained artifacts:

- `yolo26n_best.pt`
- `yolo26n_p2_best.pt`
- `rfdetr_n_best.pth`
- `dfine_n_best.pth`

The public initialization weights (`yolo26n.pt`, `rf-detr-nano.pth`, and
`dfine_n_coco.pth`) are not deployment substitutes. The four controlled
validation-selected files are staged and their manifests bind their computed
SHA-256 and size.

Manifest `status: AVAILABLE` means only that the exact artifact is present and
its identity validates. `deployment_status` remains `BLOCKER` until class
semantics, the locked framework runtime, and the required smoke test all pass.
This distinction prevents a staged checkpoint from being mistaken for a
deployment-ready backend.
