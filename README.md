VisionProTeleop
===========



Wanna use your new Apple Vision Pro to control your robot?  Wanna record your how humans navigate and manipulate the world to train robots? This app streams your (a) Wrist / Hand Tracking, and (b) Head Tracking result via gRPC over network, so any machines can subscribe and use. 


## Usage


### Run the app on Vision Pro 

![](assets/visionpro_main.png)

Click on the installed app on Vision Pro and click `Start` -- that's it!  Vision Pro is now streaming the tracking data over local network. To know about how to install the app, take a look at this [documentation](). 

Tip -- Remember the IP address before you click start; you need to specify this IP address to subscribe to the streaming data. Once you click start, the app will immediately enter into pass-through mode. Click on the digital crown to stop streaming.  


### Subscribe to data from anywhere

You can `git clone` this repository and install the pacakge: 

```
pip install -e . -v
```

Then, add this code snippet to any of your projects you were developing: 

```python
from avp_stream import VisionProStreamer
s = VisionProStreamer(ip = avp_ip, record = True, up_axis = 'Z')

while True:
    latest = s.latest
    print(latest['head'], latest['right_wrist'], latest['right_fingers'])
```

### Vision Pro (gRPC server)



## Data Type 

The `HandUpdate` structure contains (1) wristMatrix and (2) skeleton containing spatial poses of 24 hand joints.  

```yaml
HandUpdate
├── Head: Matrix4x4   # from global frame 
├── left_hand: Hand   
│   ├── wristMatrix: Matrix4x4   # from glboal frame
│   └── skeleton: Skeleton
│       └── jointMatrices: Matrix4x4[]   # from wrist frame 
└── right_hand: Hand
    ├── wristMatrix: Matrix4x4  # from global frame
    └── skeleton: Skeleton
        └── jointMatrices: Matrix4x4[]   # from wrist frame
```



## Details 

### Axis Convention

![](assets/coord_system.png)

### Hand Skeleton used in VisionOS


![](assets/hand_skeleton_convention.png)



### Recompiling Proto

In any case you want to recompile your `.proto` file, this is how you do it. 

#### for Python

```bash
python -m grpc_tools.protoc -I. --python_out=. --grpc_python_out=. handtracking.proto
```


#### for Swift
```bash
protoc handtracking.proto --swift_out=. --grpc-swift_out=.
```