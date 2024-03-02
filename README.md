VisionProTeleop
===========

![](assets/visionpro_main.png)


Wanna use your new Apple Vision Pro to control your robot?  Wanna record your manipulation demos? 

This app streams your 

1. Hand (Wrist + Finger) Tracking
2. Head Tracking 

data via gRPC over local network, so any machines can subscribe and use. 

## Data Type 

The `HandUpdate` structure contains (1) wristMatrix and (2) skeleton containing spatial poses of 24 hand joints.  

```yaml
HandUpdate
├── Head: Matrix4x4
├── left_hand: Hand
│   ├── wristMatrix: Matrix4x4
│   └── skeleton: Skeleton
│       └── jointMatrices: Matrix4x4[]
└── right_hand: Hand
    ├── wristMatrix: Matrix4x4
    └── skeleton: Skeleton
        └── jointMatrices: Matrix4x4[]
```


## Usage

### Vision Pro (Publisher)

Click on DexTeleop app and click `Start`. Remember the IP address you're seeing before you click start -- you might need it to subscribe the streaming data. 

Click on the digital crown to stop streaming. 

### Python (Subscriber)

Execute below on any machine attached to the same network. 

```python
s = VisionProStreamer(ip = VISIONPRO_IP, record = True, up_axis = 'Z')

while True:
    latest = s.latest
    print(latest['head'], latest['right_wrist'], latest['right_fingers'])
```

Your device ip will be shown in the first start screen once you start the app. 

**Disclaimer:**  Maybe don't use it over MIT network unless absolutely necessary -- TIG might not like it ... Try to use it in your own local network. 


## Axis Convention

![](assets/coord_system.png)



## Recompiling Proto

In any case you want to recompile your `.proto` file, this is how you do it. 

### for Python

```bash
python -m grpc_tools.protoc -I. --python_out=. --grpc_python_out=. handtracking.proto
```


### for Swift
```bash
protoc handtracking.proto --swift_out=. --grpc-swift_out=.
```