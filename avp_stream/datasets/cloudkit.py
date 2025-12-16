"""
CloudKit Web Services API client for accessing public recordings.

This module provides functions to query the CloudKit public database
for recordings that users have shared publicly.

Environment Variables:
    CLOUDKIT_ENVIRONMENT: Set to 'development' or 'production' (default: 'production')
"""

import os
import json
from typing import List, Optional
from urllib.request import Request, urlopen
from urllib.error import HTTPError, URLError

from .models import PublicRecording


# CloudKit configuration
_CONTAINER_ID = "iCloud.com.younghyopark.VisionProTeleop"
_RECORD_TYPE = "newpublicRecording"

# API Tokens for different environments
_API_TOKENS = {
    "production": "53fc9c269780d3b9121b9df1df04e5e199d4b01d18d9c351babfc135efb4535c",
    "development": "b6d581e25751389024c0872f0867ba8aeb676b21388712e5092b92c88ff05352",
}


def _get_environment() -> str:
    """Get the CloudKit environment from environment variable or default."""
    env = os.environ.get("CLOUDKIT_ENVIRONMENT", "production").lower()
    if env not in ("development", "production"):
        raise ValueError(f"Invalid CLOUDKIT_ENVIRONMENT: {env}. Must be 'development' or 'production'")
    return env


def _get_api_token() -> str:
    """Get the API token for the current environment."""
    return _API_TOKENS[_get_environment()]


def _get_base_url() -> str:
    """Get the base URL for the current environment."""
    env = _get_environment()
    return f"https://api.apple-cloudkit.com/database/1/{_CONTAINER_ID}/{env}/public"


def _make_request(endpoint: str, payload: dict) -> dict:
    """
    Make a request to the CloudKit Web Services API using API Token authentication.
    
    Args:
        endpoint: API endpoint path (e.g., "/records/query")
        payload: Request body as dictionary
        
    Returns:
        Response JSON as dictionary
        
    Raises:
        RuntimeError: If the request fails
    """
    url = f"{_get_base_url()}{endpoint}?ckAPIToken={_get_api_token()}"
    
    body = json.dumps(payload).encode("utf-8")
    
    headers = {
        "Content-Type": "application/json",
    }
    
    request = Request(url, data=body, headers=headers, method="POST")
    
    try:
        with urlopen(request, timeout=30) as response:
            return json.loads(response.read().decode("utf-8"))
    except HTTPError as e:
        error_body = e.read().decode("utf-8") if e.fp else ""
        raise RuntimeError(f"CloudKit API error ({e.code}): {error_body}") from e
    except URLError as e:
        raise RuntimeError(f"Network error: {e.reason}") from e


def list_public_recordings(
    limit: int = 50,
    recording_type: Optional[str] = None,
    has_video: Optional[bool] = None,
    has_simulation_data: Optional[bool] = None,
) -> List[PublicRecording]:
    """
    List public recordings from CloudKit with optional filtering.
    
    Args:
        limit: Maximum number of recordings to fetch (default: 50)
        recording_type: Filter by recording type. Options:
            - "egorecord" - EgoRecord mode recordings
            - "teleoperation-video" - Teleop with video only
            - "teleoperation-mujoco" - MuJoCo simulation recordings
            - "teleoperation-isaac" - Isaac Lab simulation recordings
        has_video: Filter by whether recording has video (True/False)
        has_simulation_data: Filter by whether recording has simulation data
    
    Returns:
        List of PublicRecording objects sorted by creation date (newest first)
        
    Raises:
        RuntimeError: If the CloudKit API request fails
        
    Example:
        >>> from avp_stream.datasets import list_public_recordings
        >>> # Get all recordings
        >>> recordings = list_public_recordings()
        >>> # Get only MuJoCo recordings
        >>> mujoco_recs = list_public_recordings(recording_type="teleoperation-mujoco")
        >>> # Get recordings with simulation data
        >>> sim_recs = list_public_recordings(has_simulation_data=True)
        
    Note:
        Set CLOUDKIT_ENVIRONMENT=development to query the development database.
    """
    # Build filter list
    filters = [{
        "fieldName": "recordingID",
        "comparator": "NOT_EQUALS",
        "fieldValue": {"value": ""}
    }]
    
    # Add optional filters
    if recording_type is not None:
        filters.append({
            "fieldName": "recordingType",
            "comparator": "EQUALS",
            "fieldValue": {"value": recording_type}
        })
    
    if has_video is not None:
        filters.append({
            "fieldName": "hasVideo",
            "comparator": "EQUALS",
            "fieldValue": {"value": 1 if has_video else 0}
        })
    
    if has_simulation_data is not None:
        filters.append({
            "fieldName": "hasSimulationData",
            "comparator": "EQUALS",
            "fieldValue": {"value": 1 if has_simulation_data else 0}
        })
    
    payload = {
        "query": {
            "recordType": _RECORD_TYPE,
            "filterBy": filters,
            "sortBy": [
                {"fieldName": "createdAt", "ascending": False}
            ]
        },
        "resultsLimit": limit
    }
    
    response = _make_request("/records/query", payload)
    
    recordings = []
    for record in response.get("records", []):
        try:
            recordings.append(PublicRecording.from_cloudkit_record(record))
        except Exception as e:
            # Skip malformed records
            print(f"Warning: Failed to parse record: {e}")
            continue
    
    return recordings


def get_recording(recording_id: str) -> Optional[PublicRecording]:
    """
    Get a specific public recording by ID.
    
    Args:
        recording_id: The recording ID to fetch
        
    Returns:
        PublicRecording if found, None otherwise
        
    Example:
        >>> from avp_stream.datasets import get_recording
        >>> rec = get_recording("my-recording-id")
        >>> if rec:
        ...     print(f"Found: {rec.title}")
    """
    payload = {
        "query": {
            "recordType": _RECORD_TYPE,
            "filterBy": [
                {
                    "fieldName": "recordingID",
                    "comparator": "EQUALS",
                    "fieldValue": {"value": recording_id}
                }
            ]
        },
        "resultsLimit": 1
    }
    
    response = _make_request("/records/query", payload)
    
    records = response.get("records", [])
    if not records:
        return None
    
    return PublicRecording.from_cloudkit_record(records[0])
