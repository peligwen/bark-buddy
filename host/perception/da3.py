"""
Depth Anything 3 backend.

One forward pass over all keyframes returns depth and camera poses in a
shared frame. Torch and the model library are imported inside `infer` so the
rest of the host runs without them.
"""

from dataclasses import dataclass
from pathlib import Path

import numpy as np

DEFAULT_MODEL = "depth-anything/DA3-LARGE-1.1"


@dataclass(frozen=True)
class Prediction:
    depth: np.ndarray           # (N, H, W) model units, shared across frames
    w2c: np.ndarray             # (N, 3, 4) world-to-camera, OpenCV convention
    K: np.ndarray               # (N, 3, 3) intrinsics at the (H, W) of `depth`
    conf: np.ndarray | None     # (N, H, W) per-pixel confidence, if the model gives one
    images: np.ndarray          # (N, H, W, 3) uint8 RGB the model actually saw


def infer(image_paths: list[Path], model_id: str = DEFAULT_MODEL,
          device: str = "cuda", process_res: int = 504) -> Prediction:
    import torch
    from depth_anything_3.api import DepthAnything3

    model = DepthAnything3.from_pretrained(model_id).to(device=torch.device(device))
    out = model.inference([str(p) for p in image_paths], process_res=process_res)
    conf = getattr(out, "conf", None)
    return Prediction(
        depth=np.asarray(out.depth, dtype=np.float32),
        w2c=np.asarray(out.extrinsics, dtype=np.float64)[:, :3, :4],
        K=np.asarray(out.intrinsics, dtype=np.float64),
        conf=None if conf is None else np.asarray(conf, dtype=np.float32),
        images=np.asarray(out.processed_images, dtype=np.uint8),
    )
