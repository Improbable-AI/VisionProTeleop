# WebRTC Video Streaming Integration

This integration allows the Vision Pro app to receive and display video streams from the Python WebRTC server.

## Architecture

1. **Python Server** (`avp_stream/visual/server.py`): Captures webcam feed and streams via WebRTC
2. **Swift Client** (`ImmersiveView.swift` + `WebRTCClient.swift`): Receives WebRTC stream and displays in AR

## Files Created

- `ImageData.swift`: ObservableObject that holds left/right UIImage frames
- `WebRTCClient.swift`: WebRTC client wrapper that handles connection and frame reception
- `ImmersiveView.swift`: RealityView that displays the video stream on a plane in AR
- `App.swift`: Modified to add video stream space option
- `ContentView.swift`: Added toggle for "Stream Back Video"

## Setup Instructions

### 1. Add WebRTC Dependency

Add the WebRTC framework to your Xcode project:

1. In Xcode, go to **File → Add Package Dependencies**
2. Enter the URL: `https://github.com/webrtc-sdk/Specs.git`
3. Or use the Google WebRTC pod: Add to your project manually

Alternatively, you can use the Swift Package Manager with:
```
https://github.com/stasel/WebRTC.git
```

### 2. Update Info.plist

Add camera and network permissions if needed:

```xml
<key>NSCameraUsageDescription</key>
<string>This app needs camera access to receive video</string>
<key>NSLocalNetworkUsageDescription</key>
<string>This app needs local network access for WebRTC streaming</string>
```

### 3. Configure Server IP

In `ImmersiveView.swift`, update the server IP address in the `VideoStreamManager.start()` method:

```swift
try await client.connect(to: "YOUR_SERVER_IP", port: 9999)
```

Replace `"YOUR_SERVER_IP"` with your Mac's local IP address (shown in ContentView).

### 4. Create Stereo Shader Material (Optional)

For true stereo vision with separate left/right images:

#### In Reality Composer Pro:

1. Create a new **Sphere** entity named "Sphere"
2. Add a **Shader Graph Material** to it
3. In the shader graph:
   - Add two **Image** parameters named "left" and "right"
   - Add a **Custom Node** that selects left/right texture based on eye (if available)
   - Connect to the **Base Color** output
4. Export the scene as "Immersive"

#### Update ImmersiveView.swift:

Uncomment the stereo material code block in the `update` closure:

```swift
if let sphereEntity = updateContent.entities.first(where: { $0.name == "Sphere" }),
   var stereoMaterial = sphereEntity.components[ModelComponent.self]?.materials.first as? ShaderGraphMaterial {
    do {
        let textureLeft = try TextureResource.generate(
            from: imageLeft.cgImage!,
            options: TextureResource.CreateOptions.init(semantic: nil)
        )
        let textureRight = try TextureResource.generate(
            from: imageRight.cgImage!,
            options: TextureResource.CreateOptions.init(semantic: nil)
        )
        
        try stereoMaterial.setParameter(name: "left", value: .textureResource(textureLeft))
        try stereoMaterial.setParameter(name: "right", value: .textureResource(textureRight))
        
        sphereEntity.components[ModelComponent.self]?.materials = [stereoMaterial]
    } catch {
        print("ERROR: Failed to update stereo material: \(error)")
    }
}
```

## Usage

1. **Start the Python server**:
   ```bash
   python avp_stream/visual/server.py
   ```

2. **Run the Vision Pro app**
3. **Toggle "Stream Back Video"** to enable video streaming
4. **Press "Start"** to enter immersive space with video feed

## Current Limitations

1. **Mono Display**: Currently displays the same image to both eyes. For true stereo, you need:
   - Two separate video streams (left/right cameras)
   - Shader graph material that routes textures to correct eyes
   
2. **Same Network**: The Vision Pro and Mac must be on the same local network

3. **Performance**: Real-time video streaming may have latency depending on network conditions

## Next Steps

To implement true stereo vision:

1. Modify `server.py` to capture stereo camera feed (two webcams or stereo camera)
2. Stream two separate video tracks (left/right)
3. Modify `WebRTCClient.swift` to receive both tracks
4. Create shader material in Reality Composer Pro for per-eye rendering
5. Update `ImmersiveView.swift` to use stereo shader material

## Troubleshooting

- **Connection fails**: Check that both devices are on same network and firewall allows connection
- **No video**: Check DEBUG logs in Xcode console for WebRTC connection state
- **Black screen**: Ensure Python server is running and sending frames
- **Build errors**: Make sure WebRTC framework is properly linked in Xcode

## References

- [WebRTC iOS Documentation](https://webrtc.github.io/webrtc-org/native-code/ios/)
- [RealityKit Documentation](https://developer.apple.com/documentation/realitykit)
- [aiortc Python WebRTC](https://github.com/aiortc/aiortc)
