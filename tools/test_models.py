import tempfile
import unittest
from pathlib import Path

from drone_control.models import ModelStore


class FakeRuntime:
    def __init__(self) -> None:
        self.command = None

    def set_batched_vla_command(self, command):
        self.command = command


class ModelStoreTest(unittest.TestCase):
    def test_download_uses_existing_local_checkpoint(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            checkpoint = root / "runs" / "transformer_vla.pt"
            checkpoint.parent.mkdir()
            checkpoint.write_bytes(b"local checkpoint")
            runtime = FakeRuntime()
            store = ModelStore(root, runtime, selection_path=root / "config" / "active_model.local.json")

            result = store.download("transformer-vla")

            self.assertEqual(result["source"], "local")
            self.assertEqual(Path(result["path"]), checkpoint)
            self.assertEqual(result["sizeBytes"], len(b"local checkpoint"))
            listed = {model["id"]: model for model in store.list()["models"]}
            self.assertTrue(listed["transformer-vla"]["downloaded"])

            store.select("transformer-vla")
            self.assertIsNotNone(runtime.command)
            self.assertIn(str(checkpoint), runtime.command)


if __name__ == "__main__":
    unittest.main()
