
import unittest
from unittest.mock import MagicMock
import numpy as np
import sys
import os

# Add parent directory to path to import avp_stream
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from avp_stream.streamer import VisionProStreamer

class TestPointCloudStreaming(unittest.TestCase):
    def test_update_pc(self):
        streamer = VisionProStreamer(ip="192.168.86.21")
        streamer.start_pointcloud_streaming()
        
        # Mock channel
        mock_channel = MagicMock()
        mock_channel.readyState = "open"
        streamer._point_cloud_channel = mock_channel
        
        # Test data
        points = np.random.rand(100, 3).astype(np.float32)
        colors = np.random.randint(0, 255, (100, 3), dtype=np.uint8)
        transform = np.eye(4, dtype=np.float32)
        
        # Call update_pc
        streamer.update_pc(points, colors, transform)
        
        # Verify calls
        # Should have sent transform (header 0x01) and points (header 0x02)
        self.assertEqual(mock_channel.send.call_count, 2)
        
        # Check transform packet
        args, _ = mock_channel.send.call_args_list[0]
        packet = args[0]
        self.assertEqual(packet[0], 0x01)
        self.assertEqual(len(packet), 1 + 16 * 4) # Header + 16 floats
        
        # Check points packet
        args, _ = mock_channel.send.call_args_list[1]
        packet = args[0]
        self.assertEqual(packet[0], 0x02)
        # Length depends on compression, but should be > 1
        self.assertGreater(len(packet), 10)

if __name__ == '__main__':
    unittest.main()
