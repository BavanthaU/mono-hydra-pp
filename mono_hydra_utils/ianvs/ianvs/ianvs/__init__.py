from ianvs._ianvs_bindings import parse_encoding
import sensor_msgs.msg
from typing import Optional
import numpy as np


def get_image(msg: sensor_msgs.msg.Image) -> Optional[np.ndarray]:
    """Return appropriate image from message if possible."""
    info = parse_encoding(msg.encoding)
    if info is None:
        return None

    shape = (msg.height, msg.width, info.channels)
    return np.squeeze(np.frombuffer(msg.data, dtype=info.dtype_str).reshape(shape))
