#import <Foundation/Foundation.h>
#import <CoreVideo/CoreVideo.h>
#import <simd/simd.h>

NS_ASSUME_NONNULL_BEGIN

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
@property (nonatomic, strong, nullable) NSData *visualizedImageData;  // JPEG data of visualization
@end

/// Objective-C++ wrapper for OpenCV camera calibration operations
@interface OpenCVCalibrator : NSObject

/// Initialize with checkerboard parameters
/// @param innerCornersX Number of inner corners horizontally (e.g., 9 for 10x7 squares)
/// @param innerCornersY Number of inner corners vertically (e.g., 6 for 10x7 squares)
/// @param squareSize Physical size of one square in meters (e.g., 0.024 for 24mm)
- (instancetype)initWithCheckerboardCornersX:(int)innerCornersX
                                     cornersY:(int)innerCornersY
                                   squareSize:(float)squareSize;

/// Detect checkerboard in a side-by-side stereo frame
/// @param pixelBuffer The stereo image (left|right side-by-side)
/// @return Detection result with corners if found
- (CheckerboardDetection * _Nullable)detectCheckerboardInStereoFrame:(CVPixelBufferRef)pixelBuffer NS_SWIFT_NAME(detectCheckerboard(stereoFrame:));

/// Detect checkerboard in a mono frame
/// @param pixelBuffer The mono image
/// @return Detection result with corners if found (in leftCorners)
- (CheckerboardDetection * _Nullable)detectCheckerboardInMonoFrame:(CVPixelBufferRef)pixelBuffer NS_SWIFT_NAME(detectCheckerboard(monoFrame:));

/// Add a calibration sample (must have detected checkerboard in both views)
/// @param leftCorners Array of CGPoint values for left image corners
/// @param rightCorners Array of CGPoint values for right image corners
/// @param imageWidth Width of a single image (not the combined stereo)
/// @param imageHeight Height of the image
/// @return Number of samples collected so far
- (int)addCalibrationSampleWithLeftCorners:(NSArray<NSValue *> *)leftCorners
                              rightCorners:(NSArray<NSValue *> *)rightCorners
                                imageWidth:(int)imageWidth
                               imageHeight:(int)imageHeight NS_SWIFT_NAME(addCalibrationSample(leftCorners:rightCorners:width:height:));

/// Add a calibration sample for mono camera
/// @param corners Array of CGPoint values for image corners
/// @param imageWidth Width of the image
/// @param imageHeight Height of the image
/// @return Number of samples collected so far
- (int)addMonoCalibrationSampleWithCorners:(NSArray<NSValue *> *)corners
                                imageWidth:(int)imageWidth
                               imageHeight:(int)imageHeight NS_SWIFT_NAME(addMonoCalibrationSample(corners:width:height:));

/// Get the current number of calibration samples
- (int)sampleCount;

/// Clear all calibration samples
- (void)clearSamples;

/// Perform stereo calibration with collected samples
/// @param minSamples Minimum number of samples required (e.g., 20)
/// @return Calibration result with intrinsics and stereo parameters
- (StereoCalibrationResult * _Nullable)performStereoCalibrationWithMinSamples:(int)minSamples NS_SWIFT_NAME(performStereoCalibration(minSamples:));

/// Perform mono camera calibration with collected samples
/// @param minSamples Minimum number of samples required (e.g., 20)
/// @return Calibration result with intrinsics (only left camera values are populated)
- (StereoCalibrationResult * _Nullable)performMonoCalibrationWithMinSamples:(int)minSamples NS_SWIFT_NAME(performMonoCalibration(minSamples:));

/// Calculate mean corner movement between two sets of corners
/// @param cornersA First set of corners
/// @param cornersB Second set of corners
/// @return Mean Euclidean distance in pixels
- (float)meanCornerMovementFrom:(NSArray<NSValue *> *)cornersA
                             to:(NSArray<NSValue *> *)cornersB NS_SWIFT_NAME(meanCornerMovement(from:to:));

/// Create a visualization of the detected checkerboard (for debugging)
/// @param pixelBuffer Original image
/// @param corners Detected corners
/// @param found Whether the checkerboard was found
/// @return JPEG data of the visualization, or nil on failure
- (NSData * _Nullable)visualizeCheckerboardInPixelBuffer:(CVPixelBufferRef)pixelBuffer
                                                 corners:(NSArray<NSValue *> * _Nullable)corners
                                                   found:(BOOL)found NS_SWIFT_NAME(visualizeCheckerboard(pixelBuffer:corners:found:));

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

/// ArUco dictionary types matching OpenCV's predefined dictionaries
typedef NS_ENUM(NSInteger, ArucoDictionaryType) {
    ArucoDictionary_4X4_50 = 0,
    ArucoDictionary_4X4_100 = 1,
    ArucoDictionary_4X4_250 = 2,
    ArucoDictionary_4X4_1000 = 3,
    ArucoDictionary_5X5_50 = 4,
    ArucoDictionary_5X5_100 = 5,
    ArucoDictionary_5X5_250 = 6,
    ArucoDictionary_5X5_1000 = 7,
    ArucoDictionary_6X6_50 = 8,
    ArucoDictionary_6X6_100 = 9,
    ArucoDictionary_6X6_250 = 10,
    ArucoDictionary_6X6_1000 = 11,
    ArucoDictionary_7X7_50 = 12,
    ArucoDictionary_7X7_100 = 13,
    ArucoDictionary_7X7_250 = 14,
    ArucoDictionary_7X7_1000 = 15,
    ArucoDictionary_ARUCO_ORIGINAL = 16,
    ArucoDictionary_APRILTAG_16h5 = 17,
    ArucoDictionary_APRILTAG_25h9 = 18,
    ArucoDictionary_APRILTAG_36h10 = 19,
    ArucoDictionary_APRILTAG_36h11 = 20,
};

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

@end

NS_ASSUME_NONNULL_END
