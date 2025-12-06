#import "OpenCVWrapper.h"

#import <CoreGraphics/CoreGraphics.h>
#import <UIKit/UIKit.h>
#import <simd/simd.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <opencv2/calib3d.hpp>
#import <opencv2/imgproc.hpp>
#import <opencv2/imgcodecs.hpp>
#import <opencv2/objdetect/aruco_detector.hpp>

// MARK: - StereoCalibrationResult Implementation

@implementation StereoCalibrationResult
- (instancetype)init {
    self = [super init];
    if (self) {
        _success = NO;
        _errorMessage = @"";
        _leftReprojectionError = 0.0;
        _rightReprojectionError = 0.0;
        _stereoReprojectionError = 0.0;
        _imageWidth = 0;
        _imageHeight = 0;
    }
    return self;
}
@end

// MARK: - CheckerboardDetection Implementation

@implementation CheckerboardDetection
- (instancetype)init {
    self = [super init];
    if (self) {
        _foundLeft = NO;
        _foundRight = NO;
        _leftCorners = nil;
        _rightCorners = nil;
        _visualizedImageData = nil;
    }
    return self;
}
@end

// MARK: - Helper Functions

namespace {

bool CreateGrayImage(CVPixelBufferRef pixelBuffer, cv::Mat &gray) {
    const OSType pixelFormat = CVPixelBufferGetPixelFormatType(pixelBuffer);

    if (pixelFormat == kCVPixelFormatType_32BGRA || pixelFormat == kCVPixelFormatType_32ARGB) {
        const size_t width = CVPixelBufferGetWidth(pixelBuffer);
        const size_t height = CVPixelBufferGetHeight(pixelBuffer);
        const size_t bytesPerRow = CVPixelBufferGetBytesPerRow(pixelBuffer);
        void *baseAddress = CVPixelBufferGetBaseAddress(pixelBuffer);

        if (baseAddress == nullptr || width == 0 || height == 0) {
            return false;
        }

        cv::Mat bgra(static_cast<int>(height), static_cast<int>(width), CV_8UC4, baseAddress, bytesPerRow);
        cv::cvtColor(bgra, gray, cv::COLOR_BGRA2GRAY);
        return true;
    }

    if (pixelFormat == kCVPixelFormatType_420YpCbCr8BiPlanarFullRange || 
        pixelFormat == kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange) {
        const size_t width = CVPixelBufferGetWidthOfPlane(pixelBuffer, 0);
        const size_t height = CVPixelBufferGetHeightOfPlane(pixelBuffer, 0);
        const size_t bytesPerRow = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 0);
        void *lumaBase = CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 0);

        if (lumaBase == nullptr || width == 0 || height == 0) {
            return false;
        }

        cv::Mat luma(static_cast<int>(height), static_cast<int>(width), CV_8UC1, lumaBase, bytesPerRow);
        gray = luma.clone();
        return true;
    }

    return false;
}

bool CreateColorImage(CVPixelBufferRef pixelBuffer, cv::Mat &color) {
    const OSType pixelFormat = CVPixelBufferGetPixelFormatType(pixelBuffer);

    if (pixelFormat == kCVPixelFormatType_32BGRA) {
        const size_t width = CVPixelBufferGetWidth(pixelBuffer);
        const size_t height = CVPixelBufferGetHeight(pixelBuffer);
        const size_t bytesPerRow = CVPixelBufferGetBytesPerRow(pixelBuffer);
        void *baseAddress = CVPixelBufferGetBaseAddress(pixelBuffer);

        if (baseAddress == nullptr || width == 0 || height == 0) {
            return false;
        }

        cv::Mat bgra(static_cast<int>(height), static_cast<int>(width), CV_8UC4, baseAddress, bytesPerRow);
        cv::cvtColor(bgra, color, cv::COLOR_BGRA2BGR);
        return true;
    }

    return false;
}

NSArray<NSValue *> *cornersToNSArray(const std::vector<cv::Point2f> &corners) {
    NSMutableArray<NSValue *> *result = [NSMutableArray arrayWithCapacity:corners.size()];
    for (const auto &corner : corners) {
        CGPoint point = CGPointMake(corner.x, corner.y);
        [result addObject:[NSValue valueWithCGPoint:point]];
    }
    return result;
}

std::vector<cv::Point2f> nsArrayToCorners(NSArray<NSValue *> *array) {
    std::vector<cv::Point2f> corners;
    corners.reserve(array.count);
    for (NSValue *value in array) {
        CGPoint point = [value CGPointValue];
        corners.push_back(cv::Point2f(static_cast<float>(point.x), static_cast<float>(point.y)));
    }
    return corners;
}

} // namespace

// MARK: - OpenCVCalibrator Implementation

@interface OpenCVCalibrator () {
    cv::Size _boardSize;
    float _squareSize;
    std::vector<cv::Point3f> _objectPoints;
    
    // Collected calibration data
    std::vector<std::vector<cv::Point2f>> _leftImagePoints;
    std::vector<std::vector<cv::Point2f>> _rightImagePoints;
    std::vector<std::vector<cv::Point3f>> _objectPointsList;
    cv::Size _imageSize;
}
@end

@implementation OpenCVCalibrator

- (instancetype)initWithCheckerboardCornersX:(int)innerCornersX
                                     cornersY:(int)innerCornersY
                                   squareSize:(float)squareSize {
    self = [super init];
    if (self) {
        _boardSize = cv::Size(innerCornersX, innerCornersY);
        _squareSize = squareSize;
        
        // Prepare object points (same for every capture)
        _objectPoints.clear();
        for (int j = 0; j < innerCornersY; ++j) {
            for (int i = 0; i < innerCornersX; ++i) {
                _objectPoints.push_back(cv::Point3f(
                    static_cast<float>(i) * squareSize,
                    static_cast<float>(j) * squareSize,
                    0.0f
                ));
            }
        }
        
        NSLog(@"📷 [OpenCVCalibrator] Initialized with %dx%d inner corners, %.3fm square size",
              innerCornersX, innerCornersY, squareSize);
    }
    return self;
}

- (CheckerboardDetection * _Nullable)detectCheckerboardInStereoFrame:(CVPixelBufferRef)pixelBuffer {
    if (pixelBuffer == nil) {
        return nil;
    }

    CVPixelBufferLockBaseAddress(pixelBuffer, kCVPixelBufferLock_ReadOnly);
    cv::Mat gray;
    const bool hasGray = CreateGrayImage(pixelBuffer, gray);
    CVPixelBufferUnlockBaseAddress(pixelBuffer, kCVPixelBufferLock_ReadOnly);

    if (!hasGray || gray.empty()) {
        return nil;
    }

    // Split side-by-side stereo image
    int width = gray.cols;
    int height = gray.rows;
    int halfWidth = width / 2;

    cv::Mat grayLeft = gray(cv::Rect(0, 0, halfWidth, height));
    cv::Mat grayRight = gray(cv::Rect(halfWidth, 0, halfWidth, height));

    CheckerboardDetection *result = [[CheckerboardDetection alloc] init];
    cv::Size rotatedBoardSize(_boardSize.height, _boardSize.width);

    // Find corners in left image - try both orientations
    std::vector<cv::Point2f> cornersLeft;
    int flags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
    result.foundLeft = cv::findChessboardCorners(grayLeft, _boardSize, cornersLeft, flags);
    
    // If not found, try rotated 90° (swapped dimensions)
    if (!result.foundLeft) {
        result.foundLeft = cv::findChessboardCorners(grayLeft, rotatedBoardSize, cornersLeft, flags);
    }

    if (result.foundLeft) {
        cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);
        cv::cornerSubPix(grayLeft, cornersLeft, cv::Size(11, 11), cv::Size(-1, -1), criteria);
        result.leftCorners = cornersToNSArray(cornersLeft);
    }

    // Find corners in right image - try both orientations
    std::vector<cv::Point2f> cornersRight;
    result.foundRight = cv::findChessboardCorners(grayRight, _boardSize, cornersRight, flags);
    
    // If not found, try rotated 90° (swapped dimensions)
    if (!result.foundRight) {
        result.foundRight = cv::findChessboardCorners(grayRight, rotatedBoardSize, cornersRight, flags);
    }

    if (result.foundRight) {
        cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);
        cv::cornerSubPix(grayRight, cornersRight, cv::Size(11, 11), cv::Size(-1, -1), criteria);
        result.rightCorners = cornersToNSArray(cornersRight);
    }

    return result;
}

- (CheckerboardDetection * _Nullable)detectCheckerboardInMonoFrame:(CVPixelBufferRef)pixelBuffer {
    if (pixelBuffer == nil) {
        return nil;
    }

    CVPixelBufferLockBaseAddress(pixelBuffer, kCVPixelBufferLock_ReadOnly);
    cv::Mat gray;
    const bool hasGray = CreateGrayImage(pixelBuffer, gray);
    CVPixelBufferUnlockBaseAddress(pixelBuffer, kCVPixelBufferLock_ReadOnly);

    if (!hasGray || gray.empty()) {
        return nil;
    }

    CheckerboardDetection *result = [[CheckerboardDetection alloc] init];

    std::vector<cv::Point2f> corners;
    int flags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
    
    // Try original orientation first
    result.foundLeft = cv::findChessboardCorners(gray, _boardSize, corners, flags);
    
    // If not found, try rotated 90° (swapped dimensions)
    if (!result.foundLeft) {
        cv::Size rotatedBoardSize(_boardSize.height, _boardSize.width);
        result.foundLeft = cv::findChessboardCorners(gray, rotatedBoardSize, corners, flags);
    }

    if (result.foundLeft) {
        cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);
        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), criteria);
        result.leftCorners = cornersToNSArray(corners);
    }

    return result;
}

- (int)addCalibrationSampleWithLeftCorners:(NSArray<NSValue *> *)leftCorners
                              rightCorners:(NSArray<NSValue *> *)rightCorners
                                imageWidth:(int)imageWidth
                               imageHeight:(int)imageHeight {
    auto cornersL = nsArrayToCorners(leftCorners);
    auto cornersR = nsArrayToCorners(rightCorners);

    _leftImagePoints.push_back(cornersL);
    _rightImagePoints.push_back(cornersR);
    _objectPointsList.push_back(_objectPoints);
    _imageSize = cv::Size(imageWidth, imageHeight);

    NSLog(@"📷 [OpenCVCalibrator] Added stereo sample #%lu", _leftImagePoints.size());
    return static_cast<int>(_leftImagePoints.size());
}

- (int)addMonoCalibrationSampleWithCorners:(NSArray<NSValue *> *)corners
                                imageWidth:(int)imageWidth
                               imageHeight:(int)imageHeight {
    auto cornersVec = nsArrayToCorners(corners);

    _leftImagePoints.push_back(cornersVec);
    _objectPointsList.push_back(_objectPoints);
    _imageSize = cv::Size(imageWidth, imageHeight);

    NSLog(@"📷 [OpenCVCalibrator] Added mono sample #%lu", _leftImagePoints.size());
    return static_cast<int>(_leftImagePoints.size());
}

- (int)sampleCount {
    return static_cast<int>(_leftImagePoints.size());
}

- (void)clearSamples {
    _leftImagePoints.clear();
    _rightImagePoints.clear();
    _objectPointsList.clear();
    NSLog(@"📷 [OpenCVCalibrator] Cleared all calibration samples");
}

- (StereoCalibrationResult * _Nullable)performStereoCalibrationWithMinSamples:(int)minSamples {
    StereoCalibrationResult *result = [[StereoCalibrationResult alloc] init];

    if (_leftImagePoints.size() < static_cast<size_t>(minSamples)) {
        result.success = NO;
        result.errorMessage = [NSString stringWithFormat:@"Not enough samples: %lu < %d", 
                               _leftImagePoints.size(), minSamples];
        return result;
    }

    if (_leftImagePoints.size() != _rightImagePoints.size()) {
        result.success = NO;
        result.errorMessage = @"Left and right sample counts don't match";
        return result;
    }

    NSLog(@"📷 [OpenCVCalibrator] Starting stereo calibration with %lu samples...", _leftImagePoints.size());

    try {
        // Calibrate left camera
        cv::Mat K_L, dist_L;
        std::vector<cv::Mat> rvecs_L, tvecs_L;
        double retL = cv::calibrateCamera(_objectPointsList, _leftImagePoints, _imageSize,
                                          K_L, dist_L, rvecs_L, tvecs_L);
        result.leftReprojectionError = retL;
        NSLog(@"📷 [OpenCVCalibrator] Left camera reprojection error: %.4f", retL);

        // Calibrate right camera
        cv::Mat K_R, dist_R;
        std::vector<cv::Mat> rvecs_R, tvecs_R;
        double retR = cv::calibrateCamera(_objectPointsList, _rightImagePoints, _imageSize,
                                          K_R, dist_R, rvecs_R, tvecs_R);
        result.rightReprojectionError = retR;
        NSLog(@"📷 [OpenCVCalibrator] Right camera reprojection error: %.4f", retR);

        // Stereo calibration
        cv::Mat R, T, E, F;
        cv::TermCriteria criteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 100, 1e-5);
        double rms = cv::stereoCalibrate(
            _objectPointsList,
            _leftImagePoints, _rightImagePoints,
            K_L, dist_L,
            K_R, dist_R,
            _imageSize,
            R, T, E, F,
            cv::CALIB_FIX_INTRINSIC,
            criteria
        );
        result.stereoReprojectionError = rms;
        NSLog(@"📷 [OpenCVCalibrator] Stereo RMS error: %.4f", rms);

        // Convert results to NSArrays
        // Left intrinsic matrix (3x3, row-major)
        NSMutableArray<NSNumber *> *leftK = [NSMutableArray arrayWithCapacity:9];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                [leftK addObject:@(K_L.at<double>(i, j))];
            }
        }
        result.leftIntrinsicMatrix = leftK;

        // Left distortion coefficients
        NSMutableArray<NSNumber *> *leftDist = [NSMutableArray arrayWithCapacity:dist_L.total()];
        for (int i = 0; i < dist_L.total(); ++i) {
            [leftDist addObject:@(dist_L.at<double>(i))];
        }
        result.leftDistortionCoeffs = leftDist;

        // Right intrinsic matrix
        NSMutableArray<NSNumber *> *rightK = [NSMutableArray arrayWithCapacity:9];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                [rightK addObject:@(K_R.at<double>(i, j))];
            }
        }
        result.rightIntrinsicMatrix = rightK;

        // Right distortion coefficients
        NSMutableArray<NSNumber *> *rightDist = [NSMutableArray arrayWithCapacity:dist_R.total()];
        for (int i = 0; i < dist_R.total(); ++i) {
            [rightDist addObject:@(dist_R.at<double>(i))];
        }
        result.rightDistortionCoeffs = rightDist;

        // Rotation matrix (3x3, row-major)
        NSMutableArray<NSNumber *> *rotation = [NSMutableArray arrayWithCapacity:9];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                [rotation addObject:@(R.at<double>(i, j))];
            }
        }
        result.rotationMatrix = rotation;

        // Translation vector (3x1)
        NSMutableArray<NSNumber *> *translation = [NSMutableArray arrayWithCapacity:3];
        for (int i = 0; i < 3; ++i) {
            [translation addObject:@(T.at<double>(i))];
        }
        result.translationVector = translation;

        result.imageWidth = _imageSize.width;
        result.imageHeight = _imageSize.height;
        result.success = YES;

        NSLog(@"✅ [OpenCVCalibrator] Stereo calibration successful!");

    } catch (const cv::Exception &e) {
        result.success = NO;
        result.errorMessage = [NSString stringWithUTF8String:e.what()];
        NSLog(@"❌ [OpenCVCalibrator] Calibration failed: %s", e.what());
    }

    return result;
}

- (StereoCalibrationResult * _Nullable)performMonoCalibrationWithMinSamples:(int)minSamples {
    StereoCalibrationResult *result = [[StereoCalibrationResult alloc] init];

    if (_leftImagePoints.size() < static_cast<size_t>(minSamples)) {
        result.success = NO;
        result.errorMessage = [NSString stringWithFormat:@"Not enough samples: %lu < %d", 
                               _leftImagePoints.size(), minSamples];
        return result;
    }

    NSLog(@"📷 [OpenCVCalibrator] Starting mono calibration with %lu samples...", _leftImagePoints.size());

    try {
        cv::Mat K, dist;
        std::vector<cv::Mat> rvecs, tvecs;
        double ret = cv::calibrateCamera(_objectPointsList, _leftImagePoints, _imageSize,
                                         K, dist, rvecs, tvecs);
        result.leftReprojectionError = ret;
        NSLog(@"📷 [OpenCVCalibrator] Camera reprojection error: %.4f", ret);

        // Convert results to NSArrays
        NSMutableArray<NSNumber *> *intrinsicK = [NSMutableArray arrayWithCapacity:9];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                [intrinsicK addObject:@(K.at<double>(i, j))];
            }
        }
        result.leftIntrinsicMatrix = intrinsicK;

        NSMutableArray<NSNumber *> *distCoeffs = [NSMutableArray arrayWithCapacity:dist.total()];
        for (int i = 0; i < dist.total(); ++i) {
            [distCoeffs addObject:@(dist.at<double>(i))];
        }
        result.leftDistortionCoeffs = distCoeffs;

        result.imageWidth = _imageSize.width;
        result.imageHeight = _imageSize.height;
        result.success = YES;

        NSLog(@"✅ [OpenCVCalibrator] Mono calibration successful!");

    } catch (const cv::Exception &e) {
        result.success = NO;
        result.errorMessage = [NSString stringWithUTF8String:e.what()];
        NSLog(@"❌ [OpenCVCalibrator] Calibration failed: %s", e.what());
    }

    return result;
}

- (float)meanCornerMovementFrom:(NSArray<NSValue *> *)cornersA
                             to:(NSArray<NSValue *> *)cornersB {
    if (cornersA == nil || cornersB == nil || cornersA.count != cornersB.count) {
        return INFINITY;
    }

    float totalDist = 0.0f;
    for (NSUInteger i = 0; i < cornersA.count; ++i) {
        CGPoint pointA = [cornersA[i] CGPointValue];
        CGPoint pointB = [cornersB[i] CGPointValue];
        float dx = static_cast<float>(pointB.x - pointA.x);
        float dy = static_cast<float>(pointB.y - pointA.y);
        totalDist += sqrtf(dx * dx + dy * dy);
    }

    return totalDist / static_cast<float>(cornersA.count);
}

- (NSData * _Nullable)visualizeCheckerboardInPixelBuffer:(CVPixelBufferRef)pixelBuffer
                                                 corners:(NSArray<NSValue *> * _Nullable)corners
                                                   found:(BOOL)found {
    if (pixelBuffer == nil) {
        return nil;
    }

    CVPixelBufferLockBaseAddress(pixelBuffer, kCVPixelBufferLock_ReadOnly);
    cv::Mat color;
    const bool hasColor = CreateColorImage(pixelBuffer, color);
    CVPixelBufferUnlockBaseAddress(pixelBuffer, kCVPixelBufferLock_ReadOnly);

    if (!hasColor || color.empty()) {
        return nil;
    }

    if (found && corners != nil) {
        auto cornersVec = nsArrayToCorners(corners);
        cv::drawChessboardCorners(color, _boardSize, cornersVec, found);
    }

    // Encode as JPEG
    std::vector<uchar> buffer;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 85};
    if (!cv::imencode(".jpg", color, buffer, params)) {
        return nil;
    }

    return [NSData dataWithBytes:buffer.data() length:buffer.size()];
}

@end

// MARK: - ArucoDetectionResult Implementation

@implementation ArucoDetectionResult
- (instancetype)init {
    self = [super init];
    if (self) {
        _markerId = -1;
        _corners = @[];
        _poseValid = NO;
        _rvec = nil;
        _tvec = nil;
        _transformMatrix = nil;
    }
    return self;
}
@end

// MARK: - OpenCVArucoDetector Implementation

@interface OpenCVArucoDetector () {
    cv::aruco::Dictionary _dictionary;
    cv::aruco::DetectorParameters _parameters;
    cv::aruco::PredefinedDictionaryType _dictionaryType;
}
@end

@implementation OpenCVArucoDetector

- (instancetype)initWithDictionary:(ArucoDictionaryType)dictionaryType {
    self = [super init];
    if (self) {
        _dictionaryType = [self cvDictionaryType:dictionaryType];
        _dictionary = cv::aruco::getPredefinedDictionary(_dictionaryType);
        _parameters = cv::aruco::DetectorParameters();
        _parameters.cornerRefinementMethod = static_cast<int>(cv::aruco::CORNER_REFINE_SUBPIX);
        
        NSLog(@"📷 [OpenCVArucoDetector] Initialized with dictionary type: %ld", (long)dictionaryType);
    }
    return self;
}

- (cv::aruco::PredefinedDictionaryType)cvDictionaryType:(ArucoDictionaryType)type {
    switch (type) {
        case ArucoDictionary_4X4_50: return cv::aruco::DICT_4X4_50;
        case ArucoDictionary_4X4_100: return cv::aruco::DICT_4X4_100;
        case ArucoDictionary_4X4_250: return cv::aruco::DICT_4X4_250;
        case ArucoDictionary_4X4_1000: return cv::aruco::DICT_4X4_1000;
        case ArucoDictionary_5X5_50: return cv::aruco::DICT_5X5_50;
        case ArucoDictionary_5X5_100: return cv::aruco::DICT_5X5_100;
        case ArucoDictionary_5X5_250: return cv::aruco::DICT_5X5_250;
        case ArucoDictionary_5X5_1000: return cv::aruco::DICT_5X5_1000;
        case ArucoDictionary_6X6_50: return cv::aruco::DICT_6X6_50;
        case ArucoDictionary_6X6_100: return cv::aruco::DICT_6X6_100;
        case ArucoDictionary_6X6_250: return cv::aruco::DICT_6X6_250;
        case ArucoDictionary_6X6_1000: return cv::aruco::DICT_6X6_1000;
        case ArucoDictionary_7X7_50: return cv::aruco::DICT_7X7_50;
        case ArucoDictionary_7X7_100: return cv::aruco::DICT_7X7_100;
        case ArucoDictionary_7X7_250: return cv::aruco::DICT_7X7_250;
        case ArucoDictionary_7X7_1000: return cv::aruco::DICT_7X7_1000;
        case ArucoDictionary_ARUCO_ORIGINAL: return cv::aruco::DICT_ARUCO_ORIGINAL;
        case ArucoDictionary_APRILTAG_16h5: return cv::aruco::DICT_APRILTAG_16h5;
        case ArucoDictionary_APRILTAG_25h9: return cv::aruco::DICT_APRILTAG_25h9;
        case ArucoDictionary_APRILTAG_36h10: return cv::aruco::DICT_APRILTAG_36h10;
        case ArucoDictionary_APRILTAG_36h11: return cv::aruco::DICT_APRILTAG_36h11;
        default: return cv::aruco::DICT_4X4_50;
    }
}

- (void)updateDictionary:(ArucoDictionaryType)dictionaryType {
    _dictionaryType = [self cvDictionaryType:dictionaryType];
    _dictionary = cv::aruco::getPredefinedDictionary(_dictionaryType);
    NSLog(@"📷 [OpenCVArucoDetector] Updated dictionary to type: %ld", (long)dictionaryType);
}

- (NSArray<ArucoDetectionResult *> * _Nullable)detectMarkersInPixelBuffer:(CVPixelBufferRef)pixelBuffer {
    if (pixelBuffer == nil) {
        return nil;
    }

    CVPixelBufferLockBaseAddress(pixelBuffer, kCVPixelBufferLock_ReadOnly);
    cv::Mat gray;
    const bool hasGray = CreateGrayImage(pixelBuffer, gray);
    CVPixelBufferUnlockBaseAddress(pixelBuffer, kCVPixelBufferLock_ReadOnly);

    if (!hasGray || gray.empty()) {
        return @[];
    }

    std::vector<std::vector<cv::Point2f>> corners;
    cv::Mat ids;
    cv::aruco::ArucoDetector detector(_dictionary, _parameters);
    detector.detectMarkers(gray, corners, ids);

    if (ids.empty()) {
        return @[];
    }

    NSMutableArray<ArucoDetectionResult *> *results = [NSMutableArray arrayWithCapacity:static_cast<NSUInteger>(ids.rows)];
    const int markerCount = std::min(static_cast<int>(corners.size()), ids.rows);

    for (int i = 0; i < markerCount; ++i) {
        const auto &markerCorners = corners[static_cast<size_t>(i)];
        if (markerCorners.size() != 4) {
            continue;
        }

        ArucoDetectionResult *result = [[ArucoDetectionResult alloc] init];
        result.markerId = ids.at<int>(i, 0);
        
        NSMutableArray<NSValue *> *cornerPoints = [NSMutableArray arrayWithCapacity:4];
        for (const auto &corner : markerCorners) {
            CGPoint point = CGPointMake(corner.x, corner.y);
            [cornerPoints addObject:[NSValue valueWithCGPoint:point]];
        }
        result.corners = cornerPoints;
        result.poseValid = NO;

        [results addObject:result];
    }

    return results;
}

- (NSArray<ArucoDetectionResult *> * _Nullable)detectMarkersInPixelBuffer:(CVPixelBufferRef)pixelBuffer
                                                             cameraMatrix:(NSArray<NSNumber *> *)cameraMatrix
                                                               distCoeffs:(NSArray<NSNumber *> *)distCoeffs
                                                             markerLength:(float)markerLength {
    if (pixelBuffer == nil || cameraMatrix.count != 9) {
        return nil;
    }

    CVPixelBufferLockBaseAddress(pixelBuffer, kCVPixelBufferLock_ReadOnly);
    cv::Mat gray;
    const bool hasGray = CreateGrayImage(pixelBuffer, gray);
    CVPixelBufferUnlockBaseAddress(pixelBuffer, kCVPixelBufferLock_ReadOnly);

    if (!hasGray || gray.empty()) {
        return @[];
    }

    std::vector<std::vector<cv::Point2f>> corners;
    cv::Mat ids;
    cv::aruco::ArucoDetector detector(_dictionary, _parameters);
    detector.detectMarkers(gray, corners, ids);

    if (ids.empty()) {
        return @[];
    }

    // Build camera matrix
    cv::Mat K = cv::Mat::zeros(3, 3, CV_64F);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            K.at<double>(i, j) = cameraMatrix[i * 3 + j].doubleValue;
        }
    }

    // Build distortion coefficients
    cv::Mat dist = cv::Mat::zeros(1, static_cast<int>(distCoeffs.count), CV_64F);
    for (NSUInteger i = 0; i < distCoeffs.count; ++i) {
        dist.at<double>(0, static_cast<int>(i)) = distCoeffs[i].doubleValue;
    }

    // Create marker object points
    const float halfLength = markerLength * 0.5f;
    cv::Mat markerObjectPoints(4, 1, CV_32FC3);
    markerObjectPoints.at<cv::Vec3f>(0, 0) = cv::Vec3f(-halfLength, halfLength, 0.0f);
    markerObjectPoints.at<cv::Vec3f>(1, 0) = cv::Vec3f(halfLength, halfLength, 0.0f);
    markerObjectPoints.at<cv::Vec3f>(2, 0) = cv::Vec3f(halfLength, -halfLength, 0.0f);
    markerObjectPoints.at<cv::Vec3f>(3, 0) = cv::Vec3f(-halfLength, -halfLength, 0.0f);

    NSMutableArray<ArucoDetectionResult *> *results = [NSMutableArray arrayWithCapacity:static_cast<NSUInteger>(ids.rows)];
    const int markerCount = std::min(static_cast<int>(corners.size()), ids.rows);

    for (int i = 0; i < markerCount; ++i) {
        const auto &markerCorners = corners[static_cast<size_t>(i)];
        if (markerCorners.size() != 4) {
            continue;
        }

        ArucoDetectionResult *result = [[ArucoDetectionResult alloc] init];
        result.markerId = ids.at<int>(i, 0);
        
        NSMutableArray<NSValue *> *cornerPoints = [NSMutableArray arrayWithCapacity:4];
        for (const auto &corner : markerCorners) {
            CGPoint point = CGPointMake(corner.x, corner.y);
            [cornerPoints addObject:[NSValue valueWithCGPoint:point]];
        }
        result.corners = cornerPoints;

        // Estimate pose
        cv::Vec3d rvec, tvec;
        cv::Mat cornersMat(markerCorners);
        
        bool solved = cv::solvePnP(markerObjectPoints,
                                   cornersMat,
                                   K,
                                   dist,
                                   rvec,
                                   tvec,
                                   false,
                                   cv::SOLVEPNP_IPPE_SQUARE);

        if (solved) {
            result.poseValid = YES;
            result.rvec = @[@(rvec[0]), @(rvec[1]), @(rvec[2])];
            result.tvec = @[@(tvec[0]), @(tvec[1]), @(tvec[2])];

            // Build 4x4 transformation matrix
            cv::Mat rotationMatrix;
            cv::Rodrigues(rvec, rotationMatrix);

            // Create column-major 4x4 matrix for simd_float4x4
            // Column 0
            NSMutableArray<NSNumber *> *transform = [NSMutableArray arrayWithCapacity:16];
            [transform addObject:@(rotationMatrix.at<double>(0, 0))];
            [transform addObject:@(rotationMatrix.at<double>(1, 0))];
            [transform addObject:@(rotationMatrix.at<double>(2, 0))];
            [transform addObject:@(0.0)];
            // Column 1
            [transform addObject:@(rotationMatrix.at<double>(0, 1))];
            [transform addObject:@(rotationMatrix.at<double>(1, 1))];
            [transform addObject:@(rotationMatrix.at<double>(2, 1))];
            [transform addObject:@(0.0)];
            // Column 2
            [transform addObject:@(rotationMatrix.at<double>(0, 2))];
            [transform addObject:@(rotationMatrix.at<double>(1, 2))];
            [transform addObject:@(rotationMatrix.at<double>(2, 2))];
            [transform addObject:@(0.0)];
            // Column 3 (translation)
            [transform addObject:@(tvec[0])];
            [transform addObject:@(tvec[1])];
            [transform addObject:@(tvec[2])];
            [transform addObject:@(1.0)];

            result.transformMatrix = transform;
        } else {
            result.poseValid = NO;
        }

        [results addObject:result];
    }

    return results;
}

+ (NSData * _Nullable)generateMarkerImageWithId:(int)markerId
                                     sizePixels:(int)sizePixels
                                     dictionary:(ArucoDictionaryType)dictionaryType {
    cv::aruco::PredefinedDictionaryType cvType;
    switch (dictionaryType) {
        case ArucoDictionary_4X4_50: cvType = cv::aruco::DICT_4X4_50; break;
        case ArucoDictionary_4X4_100: cvType = cv::aruco::DICT_4X4_100; break;
        case ArucoDictionary_4X4_250: cvType = cv::aruco::DICT_4X4_250; break;
        case ArucoDictionary_4X4_1000: cvType = cv::aruco::DICT_4X4_1000; break;
        case ArucoDictionary_5X5_50: cvType = cv::aruco::DICT_5X5_50; break;
        case ArucoDictionary_5X5_100: cvType = cv::aruco::DICT_5X5_100; break;
        case ArucoDictionary_5X5_250: cvType = cv::aruco::DICT_5X5_250; break;
        case ArucoDictionary_5X5_1000: cvType = cv::aruco::DICT_5X5_1000; break;
        case ArucoDictionary_6X6_50: cvType = cv::aruco::DICT_6X6_50; break;
        case ArucoDictionary_6X6_100: cvType = cv::aruco::DICT_6X6_100; break;
        case ArucoDictionary_6X6_250: cvType = cv::aruco::DICT_6X6_250; break;
        case ArucoDictionary_6X6_1000: cvType = cv::aruco::DICT_6X6_1000; break;
        case ArucoDictionary_7X7_50: cvType = cv::aruco::DICT_7X7_50; break;
        case ArucoDictionary_7X7_100: cvType = cv::aruco::DICT_7X7_100; break;
        case ArucoDictionary_7X7_250: cvType = cv::aruco::DICT_7X7_250; break;
        case ArucoDictionary_7X7_1000: cvType = cv::aruco::DICT_7X7_1000; break;
        case ArucoDictionary_ARUCO_ORIGINAL: cvType = cv::aruco::DICT_ARUCO_ORIGINAL; break;
        case ArucoDictionary_APRILTAG_16h5: cvType = cv::aruco::DICT_APRILTAG_16h5; break;
        case ArucoDictionary_APRILTAG_25h9: cvType = cv::aruco::DICT_APRILTAG_25h9; break;
        case ArucoDictionary_APRILTAG_36h10: cvType = cv::aruco::DICT_APRILTAG_36h10; break;
        case ArucoDictionary_APRILTAG_36h11: cvType = cv::aruco::DICT_APRILTAG_36h11; break;
        default: cvType = cv::aruco::DICT_4X4_50; break;
    }

    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cvType);
    cv::Mat markerImage;
    cv::aruco::generateImageMarker(dictionary, markerId, sizePixels, markerImage);

    std::vector<uchar> buffer;
    if (!cv::imencode(".png", markerImage, buffer)) {
        return nil;
    }

    return [NSData dataWithBytes:buffer.data() length:buffer.size()];
}

@end
