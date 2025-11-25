"""
Point Cloud Streaming over WebRTC using Google Draco Compression.

This module provides functionality to stream 3D point cloud data over WebRTC data channels
with efficient compression using Google's Draco encoder.

Example usage:
    from avp_stream import VisionProStreamer
    from avp_stream.pointcloud_streamer import PointCloudEncoder, PointCloudDecoder
    
    # Create encoder
    encoder = PointCloudEncoder(quantization_bits=14, compression_level=1)
    
    # Encode a point cloud
    points = np.random.rand(1000, 3).astype(np.float32)
    encoded_data = encoder.encode(points)
    
    # Decode the point cloud
    decoder = PointCloudDecoder()
    decoded_points = decoder.decode(encoded_data)
"""

import numpy as np
from typing import Optional, Tuple, Dict, Any
import struct
import time


# Check for DracoPy availability
try:
    import DracoPy
    DRACO_AVAILABLE = True
except ImportError:
    DRACO_AVAILABLE = False


class PointCloudEncoder:
    """
    Encodes 3D point cloud data using Google Draco compression.
    
    Draco is a library for compressing 3D geometric data, providing significant
    compression ratios while maintaining reasonable precision.
    
    Parameters
    ----------
    quantization_bits : int, default 14
        Number of bits for quantization. Higher values = more precision but larger files.
        Common values: 10 (low quality), 14 (balanced), 16 (high quality)
    compression_level : int, default 1
        Compression level (1-10). Higher = better compression but slower encoding.
        For real-time streaming, use 1-3.
    include_header : bool, default True
        If True, prepend a header with metadata (timestamp, point count, etc.)
        
    Attributes
    ----------
    total_encoded_bytes : int
        Total bytes encoded since instantiation
    total_original_bytes : int  
        Total original bytes before encoding
    frame_count : int
        Number of frames encoded
        
    Examples
    --------
    Basic encoding::
    
        encoder = PointCloudEncoder()
        points = np.random.rand(1000, 3).astype(np.float32)
        encoded = encoder.encode(points)
        
    With colors::
    
        encoder = PointCloudEncoder()
        points = np.random.rand(1000, 3).astype(np.float32)
        colors = np.random.randint(0, 255, (1000, 3), dtype=np.uint8)
        encoded = encoder.encode(points, colors=colors)
    """
    
    # Header format: magic (4 bytes) + version (1 byte) + flags (1 byte) + 
    #                timestamp (8 bytes) + point_count (4 bytes) + draco_size (4 bytes)
    HEADER_FORMAT = '<4sBBQII'
    HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
    MAGIC = b'AVPC'  # AVP Point Cloud
    VERSION = 1
    
    # Flags
    FLAG_HAS_COLORS = 0x01
    FLAG_HAS_NORMALS = 0x02
    FLAG_HAS_INTENSITY = 0x04
    
    def __init__(
        self, 
        quantization_bits: int = 14, 
        compression_level: int = 1,
        include_header: bool = True
    ):
        if not DRACO_AVAILABLE:
            raise ImportError(
                "DracoPy is required for point cloud streaming. "
                "Install it with: pip install DracoPy"
            )
            
        if not 1 <= quantization_bits <= 30:
            raise ValueError("quantization_bits must be between 1 and 30")
        if not 1 <= compression_level <= 10:
            raise ValueError("compression_level must be between 1 and 10")
            
        self.quantization_bits = quantization_bits
        self.compression_level = compression_level
        self.include_header = include_header
        
        # Statistics
        self.total_encoded_bytes = 0
        self.total_original_bytes = 0
        self.frame_count = 0
        
    def encode(
        self, 
        points: np.ndarray,
        colors: Optional[np.ndarray] = None,
        timestamp_ns: Optional[int] = None
    ) -> bytes:
        """
        Encode a point cloud to Draco-compressed bytes.
        
        Parameters
        ----------
        points : np.ndarray
            Point cloud positions as (N, 3) float32 array
        colors : np.ndarray, optional
            Point colors as (N, 3) uint8 RGB array
        timestamp_ns : int, optional
            Timestamp in nanoseconds. If None, uses current time.
            
        Returns
        -------
        bytes
            Draco-compressed point cloud data with optional header
            
        Raises
        ------
        ValueError
            If points array has wrong shape or dtype
        """
        # Validate input
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError(f"Expected (N, 3) array, got {points.shape}")
        
        # Ensure float32
        if points.dtype != np.float32:
            points = points.astype(np.float32)
            
        # Encode with Draco
        draco_bytes = DracoPy.encode_point_cloud_to_buffer(
            points,
            quantization_bits=self.quantization_bits,
            compression_level=self.compression_level
        )
        
        # Update statistics
        self.total_original_bytes += points.nbytes
        self.frame_count += 1
        
        if not self.include_header:
            self.total_encoded_bytes += len(draco_bytes)
            return bytes(draco_bytes)
        
        # Build header
        if timestamp_ns is None:
            timestamp_ns = int(time.time_ns())
            
        flags = 0
        if colors is not None:
            flags |= self.FLAG_HAS_COLORS
            
        header = struct.pack(
            self.HEADER_FORMAT,
            self.MAGIC,
            self.VERSION,
            flags,
            timestamp_ns,
            points.shape[0],
            len(draco_bytes)
        )
        
        result = header + bytes(draco_bytes)
        
        # Handle colors (append as raw bytes after Draco data if present)
        if colors is not None:
            if colors.shape[0] != points.shape[0]:
                raise ValueError("Colors array must have same number of points")
            if colors.dtype != np.uint8:
                colors = colors.astype(np.uint8)
            result += colors.tobytes()
            
        self.total_encoded_bytes += len(result)
        return result
    
    def get_compression_ratio(self) -> float:
        """
        Get the average compression ratio.
        
        Returns
        -------
        float
            Ratio of original bytes to encoded bytes
        """
        if self.total_encoded_bytes == 0:
            return 0.0
        return self.total_original_bytes / self.total_encoded_bytes
    
    def reset_stats(self):
        """Reset encoding statistics."""
        self.total_encoded_bytes = 0
        self.total_original_bytes = 0
        self.frame_count = 0


class PointCloudDecoder:
    """
    Decodes Draco-compressed point cloud data.
    
    Examples
    --------
    ::
    
        decoder = PointCloudDecoder()
        result = decoder.decode(encoded_data)
        points = result['points']
        timestamp = result['timestamp_ns']
    """
    
    def __init__(self):
        if not DRACO_AVAILABLE:
            raise ImportError(
                "DracoPy is required for point cloud streaming. "
                "Install it with: pip install DracoPy"
            )
    
    def decode(self, data: bytes) -> Dict[str, Any]:
        """
        Decode Draco-compressed point cloud data.
        
        Parameters
        ----------
        data : bytes
            Encoded point cloud data (with or without header)
            
        Returns
        -------
        dict
            Dictionary containing:
            - 'points': np.ndarray of shape (N, 3) float32
            - 'colors': np.ndarray of shape (N, 3) uint8 or None
            - 'timestamp_ns': int or None
            - 'point_count': int
            
        Raises
        ------
        ValueError
            If data cannot be decoded
        """
        result = {
            'points': None,
            'colors': None,
            'timestamp_ns': None,
            'point_count': 0
        }
        
        # Check for header
        if len(data) >= PointCloudEncoder.HEADER_SIZE:
            magic = data[:4]
            if magic == PointCloudEncoder.MAGIC:
                return self._decode_with_header(data)
        
        # No header - decode raw Draco data
        decoded = DracoPy.decode(data)
        points = np.array(decoded.points, dtype=np.float32)
        result['points'] = points
        result['point_count'] = len(points)
        return result
    
    def _decode_with_header(self, data: bytes) -> Dict[str, Any]:
        """Decode data with AVPC header."""
        header = struct.unpack(
            PointCloudEncoder.HEADER_FORMAT,
            data[:PointCloudEncoder.HEADER_SIZE]
        )
        
        _, version, flags, timestamp_ns, point_count, draco_size = header
        
        # Extract Draco data
        draco_start = PointCloudEncoder.HEADER_SIZE
        draco_end = draco_start + draco_size
        draco_data = data[draco_start:draco_end]
        
        # Decode Draco
        decoded = DracoPy.decode(draco_data)
        points = np.array(decoded.points, dtype=np.float32)
        
        result = {
            'points': points,
            'colors': None,
            'timestamp_ns': timestamp_ns,
            'point_count': len(points)
        }
        
        # Check for colors
        if flags & PointCloudEncoder.FLAG_HAS_COLORS:
            colors_start = draco_end
            colors_size = point_count * 3
            if len(data) >= colors_start + colors_size:
                colors_data = data[colors_start:colors_start + colors_size]
                result['colors'] = np.frombuffer(colors_data, dtype=np.uint8).reshape(-1, 3)
                
        return result


class PointCloudStreamMixin:
    """
    Mixin class that adds point cloud streaming capabilities to VisionProStreamer.
    
    This mixin provides methods for sending and receiving point clouds over
    WebRTC data channels using Draco compression.
    
    Note: This mixin is designed to be used with VisionProStreamer. The actual
    integration requires VisionOS-side changes to handle the point cloud data channel.
    """
    
    def __init__(self):
        self._pc_encoder: Optional[PointCloudEncoder] = None
        self._pc_decoder: Optional[PointCloudDecoder] = None
        self._pc_data_channel = None
        self._pc_callback = None
        self._pc_send_queue = []
        
    def init_pointcloud_streaming(
        self,
        quantization_bits: int = 14,
        compression_level: int = 1
    ):
        """
        Initialize point cloud streaming components.
        
        Parameters
        ----------
        quantization_bits : int, default 14
            Draco quantization bits (higher = more precision)
        compression_level : int, default 1  
            Draco compression level (higher = better compression, slower)
        """
        self._pc_encoder = PointCloudEncoder(
            quantization_bits=quantization_bits,
            compression_level=compression_level
        )
        self._pc_decoder = PointCloudDecoder()
        print(f"✓ Point cloud streaming initialized (qbits={quantization_bits}, clevel={compression_level})")
        
    def register_pointcloud_callback(self, callback):
        """
        Register a callback for receiving point clouds.
        
        Parameters
        ----------
        callback : callable
            Function that takes a dict with 'points', 'colors', 'timestamp_ns' keys
        """
        self._pc_callback = callback
        print(f"✓ Point cloud callback registered")
        
    def send_pointcloud(
        self,
        points: np.ndarray,
        colors: Optional[np.ndarray] = None,
        timestamp_ns: Optional[int] = None
    ) -> bool:
        """
        Send a point cloud over the WebRTC data channel.
        
        Parameters
        ----------
        points : np.ndarray
            Point positions as (N, 3) array
        colors : np.ndarray, optional
            Point colors as (N, 3) uint8 array
        timestamp_ns : int, optional
            Timestamp in nanoseconds
            
        Returns
        -------
        bool
            True if sent successfully, False otherwise
        """
        if self._pc_encoder is None:
            print("⚠️ Point cloud streaming not initialized. Call init_pointcloud_streaming() first.")
            return False
            
        try:
            encoded = self._pc_encoder.encode(points, colors, timestamp_ns)
            
            # If data channel is available, send directly
            if self._pc_data_channel is not None:
                # This would be used with WebRTC data channel
                # The actual sending depends on the WebRTC implementation
                self._pc_send_queue.append(encoded)
                return True
            else:
                # Queue for later sending
                self._pc_send_queue.append(encoded)
                return True
                
        except Exception as e:
            print(f"⚠️ Failed to encode point cloud: {e}")
            return False
            
    def get_pointcloud_stats(self) -> Dict[str, Any]:
        """
        Get point cloud streaming statistics.
        
        Returns
        -------
        dict
            Statistics including compression ratio, frame count, etc.
        """
        if self._pc_encoder is None:
            return {'initialized': False}
            
        return {
            'initialized': True,
            'compression_ratio': self._pc_encoder.get_compression_ratio(),
            'frame_count': self._pc_encoder.frame_count,
            'total_original_bytes': self._pc_encoder.total_original_bytes,
            'total_encoded_bytes': self._pc_encoder.total_encoded_bytes,
            'quantization_bits': self._pc_encoder.quantization_bits,
            'compression_level': self._pc_encoder.compression_level
        }


def create_sample_pointcloud(
    num_points: int = 1000,
    bounds: Tuple[float, float, float, float, float, float] = (-1, 1, -1, 1, -1, 1),
    include_colors: bool = False
) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """
    Create a sample point cloud for testing.
    
    Parameters
    ----------
    num_points : int
        Number of points to generate
    bounds : tuple
        (min_x, max_x, min_y, max_y, min_z, max_z) bounds for points
    include_colors : bool
        If True, also generate random colors
        
    Returns
    -------
    tuple
        (points, colors) where colors is None if include_colors is False
    """
    min_x, max_x, min_y, max_y, min_z, max_z = bounds
    
    points = np.random.rand(num_points, 3).astype(np.float32)
    points[:, 0] = points[:, 0] * (max_x - min_x) + min_x
    points[:, 1] = points[:, 1] * (max_y - min_y) + min_y
    points[:, 2] = points[:, 2] * (max_z - min_z) + min_z
    
    colors = None
    if include_colors:
        colors = np.random.randint(0, 255, (num_points, 3), dtype=np.uint8)
        
    return points, colors


# Convenience function to check if Draco is available
def is_draco_available() -> bool:
    """Check if DracoPy is installed and available."""
    return DRACO_AVAILABLE
