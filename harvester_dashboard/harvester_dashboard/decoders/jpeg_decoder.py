"""JPEG RGB decoder (PIL-based; no cv2 exists on these hosts)."""

from __future__ import annotations

import io

import numpy as np
from PIL import Image as PilImage


class JpegDecoder:
    """Decode canonical JPEG payloads into RGB uint8 arrays."""

    def decode(self, header, payload: bytes) -> np.ndarray:
        width = int(header['width'])
        height = int(header['height'])
        image = PilImage.open(io.BytesIO(payload))
        if image.mode != 'RGB':
            image = image.convert('RGB')
        array = np.asarray(image, dtype=np.uint8)
        if array.shape != (height, width, 3):
            raise ValueError(
                'decoded JPEG is {} but the header declares {}x{}'.format(
                    array.shape[:2], width, height))
        return array
