import importlib.util
import json
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent


def module(name: str):
    spec = importlib.util.spec_from_file_location(name, HERE / f"{name}.py")
    value = importlib.util.module_from_spec(spec); spec.loader.exec_module(value); return value


exporter = module("export_canonical_predictions")


class PostTrainingToolingTests(unittest.TestCase):
    def test_all_four_model_families_are_resolved(self):
        cases = {"YOLO26n": "yolo26n", "YOLO26n-P2": "yolo26n_p2",
                 "RF-DETR Nano": "rfdetr_n", "D-FINE Nano": "dfine_n"}
        for architecture, expected in cases.items():
            self.assertEqual(exporter.family({"architecture": architecture}), expected)

    def test_canonical_rows_filter_floor_sort_and_cap(self):
        boxes = [[1, 1, 2, 2]] * 302; scores = [0.0001] + [0.5] * 300 + [0.9]; labels = [0] * 302
        rows = exporter.canonical_rows(1, boxes, scores, labels, 10, 10)
        self.assertEqual(len(rows), 300); self.assertEqual(rows[0]["confidence"], 0.9)

    def test_canonical_rows_reject_invalid_geometry(self):
        with self.assertRaisesRegex(RuntimeError, "out-of-bounds"):
            exporter.canonical_rows(1, [[-1, 1, 2, 2]], [0.9], [0], 10, 10)

    def test_artifact_intake_covers_exactly_six_runs(self):
        path = HERE / "post_training_evidence/POST_TRAINING_ARTIFACT_INTAKE.json"
        doc = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual([x["run_id"] for x in doc["runs"]], [f"Run{i:02d}" for i in range(1, 7)])
        self.assertTrue(all(x["operator_note"] == "READY_IMMUTABLE_HISTORICAL" for x in doc["runs"][:4]))


if __name__ == "__main__": unittest.main()
