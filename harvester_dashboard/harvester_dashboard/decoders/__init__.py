"""Codec-driven payload decoders mirroring the gateway's encoders.

Every decoder implements ``decode(header, payload) -> object`` and is
selected by the packet header ``codec`` value, never by an assumed camera
or sensor identity.
"""

from __future__ import annotations

from .errors import UnsupportedCodecError
from .h264_decoder import H264Decoder
from .h265_decoder import H265Decoder
from .depth_decoder import DepthDecoder
from .jpeg_decoder import JpegDecoder
from .lidar_decoder import LidarDecoder


_DECODERS = {
    'jpeg': JpegDecoder,
    'h264': H264Decoder,
    'h265': H265Decoder,
    'depth_uint16_le': DepthDecoder,
    'lidar_xyz_f32': LidarDecoder,
}


def decoder_for_codec(codec: str):
    """Return a fresh decoder instance for a header codec string."""
    decoder_class = _DECODERS.get(codec)
    if decoder_class is None:
        raise UnsupportedCodecError(
            'no decoder registered for codec {!r}'.format(codec))
    return decoder_class()


def decode_rgb(header, payload):
    """Decode an RGB payload using the header-declared codec."""
    return decoder_for_codec(header['codec']).decode(header, payload)


def decode_depth(header, payload):
    """Depth channels must always declare ``depth_uint16_le``."""
    codec = header['codec']
    if codec != 'depth_uint16_le':
        raise UnsupportedCodecError(
            'depth channel declared unexpected codec {!r}'.format(codec))
    return DepthDecoder().decode(header, payload)


def decode_lidar(header, payload):
    """LiDAR channels must always declare ``lidar_xyz_f32``."""
    codec = header['codec']
    if codec != 'lidar_xyz_f32':
        raise UnsupportedCodecError(
            'lidar channel declared unexpected codec {!r}'.format(codec))
    return LidarDecoder().decode(header, payload)


__all__ = [
    'UnsupportedCodecError',
    'decoder_for_codec',
    'decode_rgb',
    'decode_depth',
    'decode_lidar',
    'JpegDecoder',
    'H264Decoder',
    'H265Decoder',
    'DepthDecoder',
    'LidarDecoder',
]
