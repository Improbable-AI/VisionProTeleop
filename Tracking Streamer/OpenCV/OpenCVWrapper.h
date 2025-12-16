#import <Foundation/Foundation.h>
#import <CoreVideo/CoreVideo.h>
#import <simd/simd.h>

NS_ASSUME_NONNULL_BEGIN

/// ArUco dictionary types matching OpenCV's predefined dictionaries
typedef NS_ENUM(NSInteger, ArucoDictionaryType) {
    ArucoDictionaryTypeDict4X4_50 = 0,
    ArucoDictionaryTypeDict4X4_100 = 1,
    ArucoDictionaryTypeDict4X4_250 = 2,
    ArucoDictionaryTypeDict4X4_1000 = 3,
    ArucoDictionaryTypeDict5X5_50 = 4,
    ArucoDictionaryTypeDict5X5_100 = 5,
    ArucoDictionaryTypeDict5X5_250 = 6,
    ArucoDictionaryTypeDict5X5_1000 = 7,
    ArucoDictionaryTypeDict6X6_50 = 8,
    ArucoDictionaryTypeDict6X6_100 = 9,
    ArucoDictionaryTypeDict6X6_250 = 10,
    ArucoDictionaryTypeDict6X6_1000 = 11,
    ArucoDictionaryTypeDict7X7_50 = 12,
    ArucoDictionaryTypeDict7X7_100 = 13,
    ArucoDictionaryTypeDict7X7_250 = 14,
    ArucoDictionaryTypeDict7X7_1000 = 15,
    ArucoDictionaryTypeDictOriginal = 16,
    ArucoDictionaryTypeDictAprilTag16h5 = 17,
    ArucoDictionaryTypeDictAprilTag25h9 = 18,
    ArucoDictionaryTypeDictAprilTag36h10 = 19,
    ArucoDictionaryTypeDictAprilTag36h11 = 20,
};


/// Struct to hold stereo calibration results
@interface StereoCalibrationResult : NSObject
@property (nonatomic, assign) BOOL success;
@property (nonatomic, copy) NSString *errorMessage;

// Left camera intrinsics (3x3 matrix, row-major)
@property (nonatomic, strong) NSArray<NSNumber *> *leftIntrinsicMatrix;
@property (nonatomic, strong) NSArray<NSNumber *> *leftDistortionCoeffs;
@property (nonatomic, assign) double leftReprojectionError;

// Right camera intrinsics (3x3 matrix, row-major)
@property (nonatomic, strong) NSArray<NSNumber *> *rightIntrinsicMatrix;
@property (nonatomic, strong) NSArray<NSNumber *> *rightDistortionCoeffs;
@property (nonatomic, assign) double rightReprojectionError;

// Stereo calibration (R, T from left to right)
@property (nonatomic, strong) NSArray<NSNumber *> *rotationMatrix;  // 3x3, row-major
@property (nonatomic, strong) NSArray<NSNumber *> *translationVector;  // 3x1
@property (nonatomic, assign) double stereoReprojectionError;

// Image size
@property (nonatomic, assign) int imageWidth;
@property (nonatomic, assign) int imageHeight;
@end

/// Struct to hold checkerboard detection result
@interface CheckerboardDetection : NSObject
@property (nonatomic, assign) BOOL foundLeft;
@property (nonatomic, assign) BOOL foundRight;
@property (nonatomic, strong, nullable) NSArray<NSValue *> *leftCorners;   // CGPoints
@property (nonatomic, strong, nullable) NSArray<NSValue *> *rightCorners;  // CGPoints
@property (nonatomic, strong, nullable) NSData *visualizedImageData;
@end

/// Struct to hold ChArUco detection result
@interface CharucoDetection : NSObject
@property (nonatomic, assign) BOOL foundLeft;
@property (nonatomic, assign) BOOL foundRight;
@property (nonatomic, strong, nullable) NSArray<NSValue *> *leftCorners;   // Interpolated corners
@property (nonatomic, strong, nullable) NSArray<NSNumber *> *leftIds;      // IDs of corners
@property (nonatomic, strong, nullable) NSArray<NSValue *> *rightCorners;
@property (nonatomic, strong, nullable) NSArray<NSNumber *> *rightIds;
@property (nonatomic, strong, nullable) NSData *visualizedImageData;
@end

/// Objective-C++ wrapper for OpenCV camera calibration operations
@interface OpenCVCalibrator : NSObject

/// Initialize with checkerboard parameters
- (instancetype)initWithCheckerboardCornersX:(int)innerCornersX
                                     cornersY:(int)innerCornersY
                                   squareSize:(float)squareSize;

/// Initialize with ChArUco parameters
/// @param squaresX Number of squares horizontally (e.g., 3 for 3x4 board)
/// @param squaresY Number of squares vertically (e.g., 4 for 3x4 board)
/// @param squareSize Physical size of one square in meters
/// @param markerSize Physical size of the marker in meters
/// @param dictionaryType ArUco dictionary type
- (instancetype)initWithCharucoSquaresX:(int)squaresX
                               squaresY:(int)squaresY
                             squareSize:(float)squareSize
                             markerSize:(float)markerSize
                             dictionary:(ArucoDictionaryType)dictionaryType;

/// Detect checkerboard in a side-by-side stereo frame
- (CheckerboardDetection * _Nullable)detectCheckerboardInStereoFrame:(CVPixelBufferRef)pixelBuffer NS_SWIFT_NAME(detectCheckerboard(stereoFrame:));

/// Detect ChArUco board in a side-by-side stereo frame
- (CharucoDetection * _Nullable)detectCharucoInStereoFrame:(CVPixelBufferRef)pixelBuffer NS_SWIFT_NAME(detectCharuco(stereoFrame:));

/// Detect checkerboard in a mono frame
- (CheckerboardDetection * _Nullable)detectCheckerboardInMonoFrame:(CVPixelBufferRef)pixelBuffer NS_SWIFT_NAME(detectCheckerboard(monoFrame:));

/// Detect ChArUco board in a mono frame
- (CharucoDetection * _Nullable)detectCharucoInMonoFrame:(CVPixelBufferRef)pixelBuffer NS_SWIFT_NAME(detectCharuco(monoFrame:));

/// Add a checkerboard calibration sample
- (int)addCalibrationSampleWithLeftCorners:(NSArray<NSValue *> *)leftCorners
                              rightCorners:(NSArray<NSValue *> *)rightCorners
                                imageWidth:(int)imageWidth
                               imageHeight:(int)imageHeight NS_SWIFT_NAME(addCalibrationSample(leftCorners:rightCorners:width:height:));

/// Add a ChArUco calibration sample
- (int)addCharucoSampleWithLeftCorners:(NSArray<NSValue *> *)leftCorners
                               leftIds:(NSArray<NSNumber *> *)leftIds
                          rightCorners:(NSArray<NSValue *> *)rightCorners
                              rightIds:(NSArray<NSNumber *> *)rightIds
                            imageWidth:(int)imageWidth
                           imageHeight:(int)imageHeight NS_SWIFT_NAME(addCharucoSample(leftCorners:leftIds:rightCorners:rightIds:width:height:));

/// Add a mono checkerboard sample
- (int)addMonoCalibrationSampleWithCorners:(NSArray<NSValue *> *)corners
                                imageWidth:(int)imageWidth
                               imageHeight:(int)imageHeight NS_SWIFT_NAME(addMonoCalibrationSample(corners:width:height:));

/// Add a mono ChArUco sample
- (int)addMonoCharucoSampleWithCorners:(NSArray<NSValue *> *)corners
                                   ids:(NSArray<NSNumber *> *)ids
                            imageWidth:(int)imageWidth
                           imageHeight:(int)imageHeight NS_SWIFT_NAME(addMonoCharucoSample(corners:ids:width:height:));

/// Get the current number of calibration samples
- (int)sampleCount;

/// Clear all calibration samples
- (void)clearSamples;

/// Perform stereo calibration with collected samples (works for both Checkerboard and ChArUco)
- (StereoCalibrationResult * _Nullable)performStereoCalibrationWithMinSamples:(int)minSamples NS_SWIFT_NAME(performStereoCalibration(minSamples:));

/// Perform mono calibration with collected samples
/// @param minSamples Minimum number of samples required
/// @param outlierRejection Whether to perform iterative outlier rejection (slower but more accurate)
- (StereoCalibrationResult * _Nullable)performMonoCalibrationWithMinSamples:(int)minSamples
                                                          outlierRejection:(BOOL)outlierRejection NS_SWIFT_NAME(performMonoCalibration(minSamples:outlierRejection:));

/// Calculate mean corner movement between two sets of corners
- (float)meanCornerMovementFrom:(NSArray<NSValue *> *)cornersA
                             to:(NSArray<NSValue *> *)cornersB NS_SWIFT_NAME(meanCornerMovement(from:to:));

/// Create a visualization of the detected checkerboard
- (NSData * _Nullable)visualizeCheckerboardInPixelBuffer:(CVPixelBufferRef)pixelBuffer
                                                 corners:(NSArray<NSValue *> * _Nullable)corners
                                                   found:(BOOL)found NS_SWIFT_NAME(visualizeCheckerboard(pixelBuffer:corners:found:));

/// Create a visualization of the detected ChArUco board
- (NSData * _Nullable)visualizeCharucoInPixelBuffer:(CVPixelBufferRef)pixelBuffer
                                            corners:(NSArray<NSValue *> * _Nullable)corners
                                                ids:(NSArray<NSNumber *> * _Nullable)ids
                                              found:(BOOL)found NS_SWIFT_NAME(visualizeCharuco(pixelBuffer:corners:ids:found:));

/// Create a visualization of the detected checkerboard in a stereo frame
- (NSData * _Nullable)visualizeStereoCheckerboardInPixelBuffer:(CVPixelBufferRef)pixelBuffer
                                                   leftCorners:(NSArray<NSValue *> * _Nullable)leftCorners
                                                  rightCorners:(NSArray<NSValue *> * _Nullable)rightCorners
                                                     foundLeft:(BOOL)foundLeft
                                                    foundRight:(BOOL)foundRight NS_SWIFT_NAME(visualizeStereoCheckerboard(pixelBuffer:leftCorners:rightCorners:foundLeft:foundRight:));

/// Create a visualization of the detected ChArUco board in a stereo frame
- (NSData * _Nullable)visualizeStereoCharucoInPixelBuffer:(CVPixelBufferRef)pixelBuffer
                                              leftCorners:(NSArray<NSValue *> * _Nullable)leftCorners
                                                  leftIds:(NSArray<NSNumber *> * _Nullable)leftIds
                                             rightCorners:(NSArray<NSValue *> * _Nullable)rightCorners
                                                 rightIds:(NSArray<NSNumber *> * _Nullable)rightIds
                                                foundLeft:(BOOL)foundLeft
                                               foundRight:(BOOL)foundRight NS_SWIFT_NAME(visualizeStereoCharuco(pixelBuffer:leftCorners:leftIds:rightCorners:rightIds:foundLeft:foundRight:));

@end

// MARK: - ArUco Detection Result

/// Holds the result of ArUco marker detection including pose
@interface ArucoDetectionResult : NSObject
/// Marker ID
@property (nonatomic, assign) int markerId;
/// Four corner points of the marker in image coordinates (CGPoints)
@property (nonatomic, strong) NSArray<NSValue *> *corners;
/// Whether pose was successfully estimated
@property (nonatomic, assign) BOOL poseValid;
/// Rotation vector (Rodrigues) - 3 elements
@property (nonatomic, strong, nullable) NSArray<NSNumber *> *rvec;
/// Translation vector - 3 elements (in meters)
@property (nonatomic, strong, nullable) NSArray<NSNumber *> *tvec;
/// 4x4 transformation matrix (marker in camera frame, column-major for simd)
@property (nonatomic, strong, nullable) NSArray<NSNumber *> *transformMatrix;
@end



/// Objective-C++ wrapper for OpenCV ArUco marker detection
@interface OpenCVArucoDetector : NSObject

/// Initialize with a specific ArUco dictionary
/// @param dictionaryType The type of ArUco dictionary to use
- (instancetype)initWithDictionary:(ArucoDictionaryType)dictionaryType;

/// Detect ArUco markers in a pixel buffer (without pose estimation)
/// @param pixelBuffer The image to detect markers in
/// @return Array of ArucoDetectionResult objects (without pose)
- (NSArray<ArucoDetectionResult *> * _Nullable)detectMarkersInPixelBuffer:(CVPixelBufferRef)pixelBuffer NS_SWIFT_NAME(detectMarkers(in:));

/// Detect ArUco markers and estimate their poses
/// @param pixelBuffer The image to detect markers in
/// @param cameraMatrix 3x3 camera intrinsic matrix (row-major, 9 elements)
/// @param distCoeffs Distortion coefficients (typically 5 elements)
/// @param markerLength Physical size of the marker side in meters
/// @return Array of ArucoDetectionResult objects with pose information
- (NSArray<ArucoDetectionResult *> * _Nullable)detectMarkersInPixelBuffer:(CVPixelBufferRef)pixelBuffer
                                                             cameraMatrix:(NSArray<NSNumber *> *)cameraMatrix
                                                               distCoeffs:(NSArray<NSNumber *> *)distCoeffs
                                                             markerLength:(float)markerLength NS_SWIFT_NAME(detectMarkers(in:cameraMatrix:distCoeffs:markerLength:));

/// Update the ArUco dictionary
/// @param dictionaryType New dictionary type
- (void)updateDictionary:(ArucoDictionaryType)dictionaryType;

/// Generate an ArUco marker image
/// @param markerId The ID of the marker to generate
/// @param sizePixels Size of the output image in pixels
/// @return PNG image data of the marker
+ (NSData * _Nullable)generateMarkerImageWithId:(int)markerId
                                     sizePixels:(int)sizePixels
                                     dictionary:(ArucoDictionaryType)dictionaryType NS_SWIFT_NAME(generateMarkerImage(id:sizePixels:dictionary:));

/// Visualize detected ArUco markers on a frame
/// @param pixelBuffer The original image
/// @param detections Array of ArucoDetectionResult objects to visualize
/// @param drawAxes Whether to draw pose axes (requires valid pose)
/// @param cameraMatrix Camera intrinsic matrix (required for axes, nil otherwise)
/// @param distCoeffs Distortion coefficients (required for axes, nil otherwise)
/// @param axisLength Length of axes in meters (default 0.03)
/// @return JPEG data of the visualization, or nil on failure
- (NSData * _Nullable)visualizeMarkersInPixelBuffer:(CVPixelBufferRef)pixelBuffer
                                         detections:(NSArray<ArucoDetectionResult *> *)detections
                                           drawAxes:(BOOL)drawAxes
                                       cameraMatrix:(NSArray<NSNumber *> * _Nullable)cameraMatrix
                                         distCoeffs:(NSArray<NSNumber *> * _Nullable)distCoeffs
                                         axisLength:(float)axisLength NS_SWIFT_NAME(visualizeMarkers(pixelBuffer:detections:drawAxes:cameraMatrix:distCoeffs:axisLength:));

/// Visualize detected ArUco markers on a stereo frame (both halves)
/// @param pixelBuffer The original side-by-side stereo image
/// @param leftDetections Array of ArucoDetectionResult objects from left image
/// @param rightDetections Array of ArucoDetectionResult objects from right image (corners relative to right half)
/// @return JPEG data of the visualization, or nil on failure
- (NSData * _Nullable)visualizeStereoMarkersInPixelBuffer:(CVPixelBufferRef)pixelBuffer
                                           leftDetections:(NSArray<ArucoDetectionResult *> *)leftDetections
                                          rightDetections:(NSArray<ArucoDetectionResult *> *)rightDetections NS_SWIFT_NAME(visualizeStereoMarkers(pixelBuffer:leftDetections:rightDetections:));

@end

NS_ASSUME_NONNULL_END
