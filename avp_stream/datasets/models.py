"""
Data models for the avp_stream.datasets module.
"""

from dataclasses import dataclass, field
from datetime import datetime
from typing import Optional


@dataclass
class PublicRecording:
    """
    Represents a public recording shared via CloudKit.
    
    Attributes:
        id: Unique CloudKit record ID
        recording_id: Original recording identifier
        title: Display title of the recording
        description: Optional description
        cloud_url: URL to the recording (Dropbox or Google Drive shared link)
        thumbnail_url: Optional URL to a thumbnail image
        provider: Cloud storage provider ("dropbox", "googleDrive", or "iCloudDrive")
        recording_type: Type of recording (egorecord, teleoperation-video, teleoperation-mujoco, teleoperation-isaac)
        created_at: Timestamp when the recording was made public
        
        # Metadata fields for filtering
        duration: Recording duration in seconds
        frame_count: Number of frames recorded
        has_video: Whether recording contains video
        has_left_hand: Whether left hand tracking is present
        has_right_hand: Whether right hand tracking is present
        has_simulation_data: Whether simulation data is present
        has_usdz: Whether USDZ scene file is present
        video_source: Video source type ("network" or "uvc")
        average_fps: Average frames per second
    """
    id: str
    recording_id: str
    title: str
    cloud_url: str
    provider: str
    created_at: datetime
    description: Optional[str] = None
    thumbnail_url: Optional[str] = None
    recording_type: Optional[str] = None
    # Metadata fields
    duration: Optional[float] = None
    frame_count: Optional[int] = None
    has_video: Optional[bool] = None
    has_left_hand: Optional[bool] = None
    has_right_hand: Optional[bool] = None
    has_simulation_data: Optional[bool] = None
    has_usdz: Optional[bool] = None
    video_source: Optional[str] = None
    average_fps: Optional[float] = None
    
    @classmethod
    def from_cloudkit_record(cls, record: dict) -> "PublicRecording":
        """
        Create a PublicRecording from a CloudKit record dictionary.
        
        Args:
            record: Dictionary containing CloudKit record fields
            
        Returns:
            PublicRecording instance
        """
        fields = record.get("fields", {})
        
        # Extract field values (CloudKit wraps values in type objects)
        def get_field(name: str, default=None):
            field_data = fields.get(name, {})
            return field_data.get("value", default)
        
        # Parse creation date
        created_timestamp = get_field("createdAt")
        if isinstance(created_timestamp, (int, float)):
            # CloudKit uses milliseconds since epoch
            created_at = datetime.fromtimestamp(created_timestamp / 1000)
        else:
            created_at = datetime.now()
        
        # Parse boolean fields (stored as 0/1 in CloudKit)
        def get_bool_field(name: str) -> Optional[bool]:
            val = get_field(name)
            if val is None:
                return None
            return bool(val)
        
        return cls(
            id=record.get("recordName", ""),
            recording_id=get_field("recordingID", ""),
            title=get_field("title", "Untitled"),
            cloud_url=get_field("cloudURL", ""),
            provider=get_field("provider", "unknown"),
            created_at=created_at,
            description=get_field("recodringDescription"),  # Note: typo matches CloudKit schema
            thumbnail_url=get_field("thumbnailURL"),
            recording_type=get_field("recordingType"),
            # Metadata fields
            duration=get_field("duration"),
            frame_count=get_field("frameCount"),
            has_video=get_bool_field("hasVideo"),
            has_left_hand=get_bool_field("hasLeftHand"),
            has_right_hand=get_bool_field("hasRightHand"),
            has_simulation_data=get_bool_field("hasSimulationData"),
            has_usdz=get_bool_field("hasUSDZ"),
            video_source=get_field("videoSource"),
            average_fps=get_field("averageFPS"),
        )
    
    def __repr__(self) -> str:
        parts = [f"title='{self.title}'", f"provider='{self.provider}'"]
        if self.recording_type:
            parts.append(f"type='{self.recording_type}'")
        if self.duration:
            parts.append(f"duration={self.duration:.1f}s")
        if self.frame_count:
            parts.append(f"frames={self.frame_count}")
        parts.append(f"created_at={self.created_at.isoformat()}")
        return f"PublicRecording({', '.join(parts)})"

