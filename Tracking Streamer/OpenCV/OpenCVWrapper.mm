#import "OpenCVWrapper.h"

#import <CoreGraphics/CoreGraphics.h>
#import <UIKit/UIKit.h>
#import <simd/simd.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include <map>

#include <opencv2/calib3d.hpp>
#import <opencv2/imgproc.hpp>
#import <opencv2/imgcodecs.hpp>
#import <opencv2/objdetect/aruco_detector.hpp>
#import <opencv2/objdetect/charuco_detector.hpp>

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

// MARK: - CharucoDetection Implementation

@implementation CharucoDetection
- (instancetype)init {
    self = [super init];
    if (self) {
        _foundLeft = NO;
        _foundRight = NO;
        _leftCorners = nil;
        _leftIds = nil;
        _rightCorners = nil;
        _rightIds = nil;
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
    
    // Collected calibration data (protected by @synchronized(self))
    std::vector<std::vector<cv::Point2f>> _leftImagePoints;
    std::vector<std::vector<cv::Point2f>> _rightImagePoints;
    std::vector<std::vector<cv::Point3f>> _objectPointsList;
    cv::Size _imageSize;
    
    // ChArUco members
    cv::Ptr<cv::aruco::CharucoBoard> _charucoBoard;
    cv::Ptr<cv::aruco::CharucoDetector> _charucoDetector;
    
    // Lock object for thread-safe access to calibration data
    NSLock *_dataLock;
}
@end

@implementation OpenCVCalibrator

- (instancetype)initWithCheckerboardCornersX:(int)innerCornersX
                                     cornersY:(int)innerCornersY
                                   squareSize:(float)squareSize {
    self = [super init];
    if (self) {
        _dataLock = [[NSLock alloc] init];
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

// Helper to convert dictionary type (duplicated from OpenCVArucoDetector to avoid dependency)
- (cv::aruco::PredefinedDictionaryType)getDictionaryType:(ArucoDictionaryType)type {
    switch (type) {
        case ArucoDictionaryTypeDict4X4_50: return cv::aruco::DICT_4X4_50;
        case ArucoDictionaryTypeDict4X4_100: return cv::aruco::DICT_4X4_100;
        case ArucoDictionaryTypeDict4X4_250: return cv::aruco::DICT_4X4_250;
        case ArucoDictionaryTypeDict4X4_1000: return cv::aruco::DICT_4X4_1000;
        case ArucoDictionaryTypeDict5X5_50: return cv::aruco::DICT_5X5_50;
        case ArucoDictionaryTypeDict5X5_100: return cv::aruco::DICT_5X5_100;
        default: return cv::aruco::DICT_4X4_50;
    }
}

- (instancetype)initWithCharucoSquaresX:(int)squaresX
                               squaresY:(int)squaresY
                             squareSize:(float)squareSize
                             markerSize:(float)markerSize
                             dictionary:(ArucoDictionaryType)dictionaryType {
    self = [super init];
    if (self) {
        _dataLock = [[NSLock alloc] init];
        _boardSize = cv::Size(squaresX, squaresY);
        _squareSize = squareSize;
        
        cv::aruco::PredefinedDictionaryType dictType = [self getDictionaryType:dictionaryType];
        cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(dictType);
        
        _charucoBoard = new cv::aruco::CharucoBoard(cv::Size(squaresX, squaresY), squareSize, markerSize, dictionary);
        _charucoDetector = new cv::aruco::CharucoDetector(*_charucoBoard);
        
        // Refine parameters for better accuracy
        _charucoDetector->setDetectorParameters(cv::aruco::DetectorParameters());
        _charucoDetector->setRefineParameters(cv::aruco::RefineParameters(10.0f, 3.0f, true)); // minRepDistance, errorCorrectionRate, checkAllOrders
        
        NSLog(@"📷 [OpenCVCalibrator] Initialized ChArUco with %dx%d squares, %.3fm square size, %.3fm marker size",
              squaresX, squaresY, squareSize, markerSize);
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
    // FAST_CHECK quickly rejects frames where pattern is definitely not present
    int flags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK;
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

- (CharucoDetection * _Nullable)detectCharucoInStereoFrame:(CVPixelBufferRef)pixelBuffer {
    if (pixelBuffer == nil) {
        NSLog(@"📷 [OpenCVCalibrator] detectCharucoInStereoFrame: pixelBuffer is nil");
        return nil;
    }
    if (!_charucoDetector) {
        NSLog(@"📷 [OpenCVCalibrator] detectCharucoInStereoFrame: _charucoDetector is nil - was calibrator initialized with ChArUco?");
        return nil;
    }

    CVPixelBufferLockBaseAddress(pixelBuffer, kCVPixelBufferLock_ReadOnly);
    cv::Mat gray;
    const bool hasGray = CreateGrayImage(pixelBuffer, gray);
    CVPixelBufferUnlockBaseAddress(pixelBuffer, kCVPixelBufferLock_ReadOnly);

    if (!hasGray || gray.empty()) {
        NSLog(@"📷 [OpenCVCalibrator] detectCharucoInStereoFrame: Failed to create gray image");
        return nil;
    }

    // Split side-by-side stereo image
    int width = gray.cols;
    int height = gray.rows;
    int halfWidth = width / 2;

    NSLog(@"📷 [OpenCVCalibrator] detectCharucoInStereoFrame: Image size %dx%d, half width %d", width, height, halfWidth);

    cv::Mat grayLeft = gray(cv::Rect(0, 0, halfWidth, height));
    cv::Mat grayRight = gray(cv::Rect(halfWidth, 0, halfWidth, height));

    CharucoDetection *result = [[CharucoDetection alloc] init];

    // Detect in Left
    std::vector<int> charucoIdsLeft;
    std::vector<cv::Point2f> charucoCornersLeft;
    cv::Mat currentCharucoCornersLeft, currentCharucoIdsLeft;
    
    _charucoDetector->detectBoard(grayLeft, currentCharucoCornersLeft, currentCharucoIdsLeft);
    
    NSLog(@"📷 [OpenCVCalibrator] Left detection: found %lu corners", (unsigned long)currentCharucoIdsLeft.total());
    
    if (currentCharucoIdsLeft.total() > 0) {
        result.foundLeft = YES;
        currentCharucoCornersLeft.copyTo(charucoCornersLeft);
        currentCharucoIdsLeft.copyTo(charucoIdsLeft);
        
        result.leftCorners = cornersToNSArray(charucoCornersLeft);
        NSMutableArray<NSNumber *> *ids = [NSMutableArray arrayWithCapacity:charucoIdsLeft.size()];
        for (int id : charucoIdsLeft) {
            [ids addObject:@(id)];
        }
        result.leftIds = ids;
    }

    // Detect in Right
    std::vector<int> charucoIdsRight;
    std::vector<cv::Point2f> charucoCornersRight;
    cv::Mat currentCharucoCornersRight, currentCharucoIdsRight;
    
    _charucoDetector->detectBoard(grayRight, currentCharucoCornersRight, currentCharucoIdsRight);
    
    NSLog(@"📷 [OpenCVCalibrator] Right detection: found %lu corners", (unsigned long)currentCharucoIdsRight.total());
    
    if (currentCharucoIdsRight.total() > 0) {
        result.foundRight = YES;
        currentCharucoCornersRight.copyTo(charucoCornersRight);
        currentCharucoIdsRight.copyTo(charucoIdsRight);
        
        result.rightCorners = cornersToNSArray(charucoCornersRight);
        NSMutableArray<NSNumber *> *ids = [NSMutableArray arrayWithCapacity:charucoIdsRight.size()];
        for (int id : charucoIdsRight) {
            [ids addObject:@(id)];
        }
        result.rightIds = ids;
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
    // FAST_CHECK quickly rejects frames where pattern is definitely not present
    int flags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK;
    
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

- (CharucoDetection * _Nullable)detectCharucoInMonoFrame:(CVPixelBufferRef)pixelBuffer {
    if (pixelBuffer == nil || !_charucoDetector) {
        return nil;
    }

    CVPixelBufferLockBaseAddress(pixelBuffer, kCVPixelBufferLock_ReadOnly);
    cv::Mat gray;
    const bool hasGray = CreateGrayImage(pixelBuffer, gray);
    CVPixelBufferUnlockBaseAddress(pixelBuffer, kCVPixelBufferLock_ReadOnly);

    if (!hasGray || gray.empty()) {
        return nil;
    }

    CharucoDetection *result = [[CharucoDetection alloc] init];

    std::vector<int> charucoIds;
    std::vector<cv::Point2f> charucoCorners;
    cv::Mat currentCharucoCorners, currentCharucoIds;
    
    _charucoDetector->detectBoard(gray, currentCharucoCorners, currentCharucoIds);
    
    if (currentCharucoIds.total() > 0) {
        result.foundLeft = YES;  // Use 'foundLeft' for mono
        currentCharucoCorners.copyTo(charucoCorners);
        currentCharucoIds.copyTo(charucoIds);
        
        result.leftCorners = cornersToNSArray(charucoCorners);
        NSMutableArray<NSNumber *> *ids = [NSMutableArray arrayWithCapacity:charucoIds.size()];
        for (int id : charucoIds) {
            [ids addObject:@(id)];
        }
        result.leftIds = ids;
    }

    return result;
}

- (int)addCalibrationSampleWithLeftCorners:(NSArray<NSValue *> *)leftCorners
                              rightCorners:(NSArray<NSValue *> *)rightCorners
                                imageWidth:(int)imageWidth
                               imageHeight:(int)imageHeight {
    auto cornersL = nsArrayToCorners(leftCorners);
    auto cornersR = nsArrayToCorners(rightCorners);

    [_dataLock lock];
    _leftImagePoints.push_back(cornersL);
    _rightImagePoints.push_back(cornersR);
    _objectPointsList.push_back(_objectPoints);
    _imageSize = cv::Size(imageWidth, imageHeight);
    int count = static_cast<int>(_leftImagePoints.size());
    [_dataLock unlock];

    NSLog(@"📷 [OpenCVCalibrator] Added stereo sample #%d", count);
    return count;
}

- (int)addMonoCalibrationSampleWithCorners:(NSArray<NSValue *> *)corners
                                imageWidth:(int)imageWidth
                               imageHeight:(int)imageHeight {
    auto cornersVec = nsArrayToCorners(corners);

    [_dataLock lock];
    _leftImagePoints.push_back(cornersVec);
    _objectPointsList.push_back(_objectPoints);
    _imageSize = cv::Size(imageWidth, imageHeight);
    int count = static_cast<int>(_leftImagePoints.size());
    [_dataLock unlock];

    NSLog(@"📷 [OpenCVCalibrator] Added mono sample #%d", count);
    return count;
}

- (int)addCharucoSampleWithLeftCorners:(NSArray<NSValue *> *)leftCorners
                               leftIds:(NSArray<NSNumber *> *)leftIds
                          rightCorners:(NSArray<NSValue *> *)rightCorners
                              rightIds:(NSArray<NSNumber *> *)rightIds
                            imageWidth:(int)imageWidth
                           imageHeight:(int)imageHeight {
    if (!_charucoBoard) return 0;
    
    auto cornersL = nsArrayToCorners(leftCorners);
    auto cornersR = nsArrayToCorners(rightCorners);
    
    std::vector<int> idsL;
    for (NSNumber *num in leftIds) idsL.push_back([num intValue]);
    
    std::vector<int> idsR;
    for (NSNumber *num in rightIds) idsR.push_back([num intValue]);
    
    // For ChArUco, we need to find the common subset of object points for both views if we want stereo calibration?
    // Actually, cv::stereoCalibrate takes objectPoints for each image pair. 
    // BUT, the objectPoints for the pair MUST be the same (the pattern in the world).
    // The detected corners must correspond to the SAME object points.
    // In ChArUco, left and right might detect slightly different subsets of corners.
    // We must find the INTERSECTION of detected corner IDs and only use those.
    
    // Find common IDs
    std::vector<int> commonIds;
    std::vector<cv::Point2f> commonCornersL;
    std::vector<cv::Point2f> commonCornersR;
    
    // Map ID -> Index for fast lookup
    std::map<int, int> leftMap;
    for (size_t i = 0; i < idsL.size(); i++) leftMap[idsL[i]] = i;
    
    for (size_t i = 0; i < idsR.size(); i++) {
        int id = idsR[i];
        if (leftMap.count(id)) {
            commonIds.push_back(id);
            commonCornersL.push_back(cornersL[leftMap[id]]);
            commonCornersR.push_back(cornersR[i]);
        }
    }
    
    if (commonIds.size() < 4) {
        [_dataLock lock];
        int count = static_cast<int>(_leftImagePoints.size());
        [_dataLock unlock];
        NSLog(@"⚠️ [OpenCVCalibrator] Skipping ChArUco sample: only %lu common corners (need 4+)", commonIds.size());
        return count;
    }
    
    // Get object points for common IDs
    std::vector<cv::Point3f> objPoints;
    std::vector<cv::Point2f> imgPointsUnused; // We already have them
    _charucoBoard->matchImagePoints(commonCornersL, commonIds, objPoints, imgPointsUnused);
    
    [_dataLock lock];
    _leftImagePoints.push_back(commonCornersL);
    _rightImagePoints.push_back(commonCornersR);
    _objectPointsList.push_back(objPoints);
    _imageSize = cv::Size(imageWidth, imageHeight);
    int count = static_cast<int>(_leftImagePoints.size());
    [_dataLock unlock];
    
    NSLog(@"📷 [OpenCVCalibrator] Added ChArUco stereo sample #%d with %lu common corners", 
          count, commonIds.size());
    return count;
}

- (int)addMonoCharucoSampleWithCorners:(NSArray<NSValue *> *)corners
                                   ids:(NSArray<NSNumber *> *)ids
                            imageWidth:(int)imageWidth
                           imageHeight:(int)imageHeight {
    if (!_charucoBoard) return 0;

    auto cornersVec = nsArrayToCorners(corners);
    std::vector<int> idsVec;
    for (NSNumber *num in ids) idsVec.push_back([num intValue]);
    
    if (idsVec.size() < 4) {
        [_dataLock lock];
        int count = static_cast<int>(_leftImagePoints.size());
        [_dataLock unlock];
        return count;
    }
    
    // Get object points
    std::vector<cv::Point3f> objPoints;
    std::vector<cv::Point2f> imgPointsUnused;
    _charucoBoard->matchImagePoints(cornersVec, idsVec, objPoints, imgPointsUnused);
    
    [_dataLock lock];
    _leftImagePoints.push_back(cornersVec);
    _objectPointsList.push_back(objPoints);
    _imageSize = cv::Size(imageWidth, imageHeight);
    int count = static_cast<int>(_leftImagePoints.size());
    [_dataLock unlock];
    
    NSLog(@"📷 [OpenCVCalibrator] Added ChArUco mono sample #%d with %lu corners", 
          count, idsVec.size());
    return count;
}



- (int)sampleCount {
    [_dataLock lock];
    int count = static_cast<int>(_leftImagePoints.size());
    [_dataLock unlock];
    return count;
}

- (void)clearSamples {
    [_dataLock lock];
    _leftImagePoints.clear();
    _rightImagePoints.clear();
    _objectPointsList.clear();
    [_dataLock unlock];
    NSLog(@"📷 [OpenCVCalibrator] Cleared all calibration samples");
}

- (StereoCalibrationResult * _Nullable)performStereoCalibrationWithMinSamples:(int)minSamples {
    StereoCalibrationResult *result = [[StereoCalibrationResult alloc] init];

    // Copy data under lock to avoid holding lock during calibration
    [_dataLock lock];
    std::vector<std::vector<cv::Point3f>> objectPoints = _objectPointsList;
    std::vector<std::vector<cv::Point2f>> leftImagePoints = _leftImagePoints;
    std::vector<std::vector<cv::Point2f>> rightImagePoints = _rightImagePoints;
    cv::Size imageSize = _imageSize;
    [_dataLock unlock];
    
    if (leftImagePoints.size() < static_cast<size_t>(minSamples)) {
        result.success = NO;
        result.errorMessage = [NSString stringWithFormat:@"Not enough samples: %lu < %d", 
                               leftImagePoints.size(), minSamples];
        return result;
    }

    if (leftImagePoints.size() != rightImagePoints.size()) {
        result.success = NO;
        result.errorMessage = @"Left and right sample counts don't match";
        return result;
    }

    NSLog(@"📷 [OpenCVCalibrator] Starting stereo calibration with %lu samples...", leftImagePoints.size());

    try {
        // Calibrate left camera
        cv::Mat K_L, dist_L;
        std::vector<cv::Mat> rvecs_L, tvecs_L;
        double retL = cv::calibrateCamera(objectPoints, leftImagePoints, imageSize,
                                          K_L, dist_L, rvecs_L, tvecs_L);
        result.leftReprojectionError = retL;
        NSLog(@"📷 [OpenCVCalibrator] Left camera reprojection error: %.4f", retL);

        // Calibrate right camera
        cv::Mat K_R, dist_R;
        std::vector<cv::Mat> rvecs_R, tvecs_R;
        double retR = cv::calibrateCamera(objectPoints, rightImagePoints, imageSize,
                                          K_R, dist_R, rvecs_R, tvecs_R);
        result.rightReprojectionError = retR;
        NSLog(@"📷 [OpenCVCalibrator] Right camera reprojection error: %.4f", retR);

        // Stereo calibration
        cv::Mat R, T, E, F;
        cv::TermCriteria criteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 100, 1e-5);
        double rms = cv::stereoCalibrate(
            objectPoints,
            leftImagePoints, rightImagePoints,
            K_L, dist_L,
            K_R, dist_R,
            imageSize,
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

        result.imageWidth = imageSize.width;
        result.imageHeight = imageSize.height;
        result.success = YES;

        NSLog(@"✅ [OpenCVCalibrator] Stereo calibration successful!");

    } catch (const cv::Exception &e) {
        result.success = NO;
        result.errorMessage = [NSString stringWithUTF8String:e.what()];
        NSLog(@"❌ [OpenCVCalibrator] Calibration failed: %s", e.what());
    }

    return result;
}

- (StereoCalibrationResult * _Nullable)performMonoCalibrationWithMinSamples:(int)minSamples
                                                          outlierRejection:(BOOL)outlierRejection {
    StereoCalibrationResult *result = [[StereoCalibrationResult alloc] init];

    // Copy data under lock to avoid holding lock during calibration
    [_dataLock lock];
    std::vector<std::vector<cv::Point3f>> objectPoints = _objectPointsList;
    std::vector<std::vector<cv::Point2f>> imagePoints = _leftImagePoints;
    cv::Size imageSize = _imageSize;
    size_t totalSampleCount = _leftImagePoints.size();
    [_dataLock unlock];
    
    if (imagePoints.size() < static_cast<size_t>(minSamples)) {
        result.success = NO;
        result.errorMessage = [NSString stringWithFormat:@"Not enough samples: %lu < %d", 
                               imagePoints.size(), minSamples];
        return result;
    }

    NSLog(@"📷 [OpenCVCalibrator] Starting mono calibration with %lu samples (outlier rejection: %@)...", 
          imagePoints.size(), outlierRejection ? @"YES" : @"NO");

    try {
        cv::Mat K, dist;
        std::vector<cv::Mat> rvecs, tvecs;
        
        // Initial calibration
        double ret = cv::calibrateCamera(objectPoints, imagePoints, imageSize,
                                         K, dist, rvecs, tvecs);
        NSLog(@"📷 [OpenCVCalibrator] Initial reprojection error: %.4f", ret);
        
        // Outlier rejection: only run when requested (slower but more accurate)
        if (outlierRejection) {
            const int maxIterations = 3;
            for (int iteration = 0; iteration < maxIterations; ++iteration) {
            if (imagePoints.size() < static_cast<size_t>(minSamples)) break;
            
            std::vector<double> sampleErrors;
            for (size_t i = 0; i < imagePoints.size(); ++i) {
                std::vector<cv::Point2f> projected;
                cv::projectPoints(objectPoints[i], rvecs[i], tvecs[i], K, dist, projected);
                
                double sumErr = 0.0;
                for (size_t j = 0; j < projected.size(); ++j) {
                    double dx = projected[j].x - imagePoints[i][j].x;
                    double dy = projected[j].y - imagePoints[i][j].y;
                    sumErr += sqrt(dx*dx + dy*dy);
                }
                sampleErrors.push_back(sumErr / projected.size());
            }
            
            // Compute mean and std of sample errors
            double mean = 0.0;
            for (double e : sampleErrors) mean += e;
            mean /= sampleErrors.size();
            
            double variance = 0.0;
            for (double e : sampleErrors) variance += (e - mean) * (e - mean);
            double std = sqrt(variance / sampleErrors.size());
            
            // Threshold: mean + 1.5 * std
            double threshold = mean + 1.5 * std;
            
            // Remove outliers
            std::vector<std::vector<cv::Point3f>> newObjectPoints;
            std::vector<std::vector<cv::Point2f>> newImagePoints;
            int removed = 0;
            for (size_t i = 0; i < sampleErrors.size(); ++i) {
                if (sampleErrors[i] <= threshold) {
                    newObjectPoints.push_back(objectPoints[i]);
                    newImagePoints.push_back(imagePoints[i]);
                } else {
                    removed++;
                }
            }
            
            if (removed == 0) break;  // No outliers to remove
            
            NSLog(@"📷 [OpenCVCalibrator] Outlier rejection iter %d: removed %d samples (threshold: %.3f)", 
                  iteration + 1, removed, threshold);
            
            // Re-calibrate without outliers
            objectPoints = newObjectPoints;
            imagePoints = newImagePoints;
            
            if (imagePoints.size() < static_cast<size_t>(minSamples)) {
                NSLog(@"⚠️ [OpenCVCalibrator] Too few samples after outlier rejection");
                break;
            }
            
            rvecs.clear();
            tvecs.clear();
            ret = cv::calibrateCamera(objectPoints, imagePoints, imageSize,
                                      K, dist, rvecs, tvecs);
            NSLog(@"📷 [OpenCVCalibrator] Reprojection error after removal: %.4f", ret);
            }
        }  // end if (outlierRejection)
        
        result.leftReprojectionError = ret;
        NSLog(@"📷 [OpenCVCalibrator] Final reprojection error: %.4f (used %lu samples)", ret, imagePoints.size());

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

        result.imageWidth = imageSize.width;
        result.imageHeight = imageSize.height;
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

- (NSData * _Nullable)visualizeStereoCheckerboardInPixelBuffer:(CVPixelBufferRef)pixelBuffer
                                                   leftCorners:(NSArray<NSValue *> * _Nullable)leftCorners
                                                  rightCorners:(NSArray<NSValue *> * _Nullable)rightCorners
                                                     foundLeft:(BOOL)foundLeft
                                                    foundRight:(BOOL)foundRight {
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

    int width = color.cols;
    int halfWidth = width / 2;
    int height = color.rows;

    // Draw on left half
    if (foundLeft && leftCorners != nil) {
        cv::Mat leftHalf = color(cv::Rect(0, 0, halfWidth, height));
        auto cornersVec = nsArrayToCorners(leftCorners);
        cv::drawChessboardCorners(leftHalf, _boardSize, cornersVec, foundLeft);
    }

    // Draw on right half (corners need to be offset by halfWidth)
    if (foundRight && rightCorners != nil) {
        auto cornersVec = nsArrayToCorners(rightCorners);
        // Offset corners to the right half of the image
        for (auto& corner : cornersVec) {
            corner.x += halfWidth;
        }
        cv::Mat fullImage = color;  // Work on full image with offset corners
        cv::drawChessboardCorners(fullImage, _boardSize, cornersVec, foundRight);
    }

    // Encode as JPEG
    std::vector<uchar> buffer;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 85};
    if (!cv::imencode(".jpg", color, buffer, params)) {
        return nil;
    }

    return [NSData dataWithBytes:buffer.data() length:buffer.size()];
}

- (NSData * _Nullable)visualizeCharucoInPixelBuffer:(CVPixelBufferRef)pixelBuffer
                                            corners:(NSArray<NSValue *> * _Nullable)corners
                                                ids:(NSArray<NSNumber *> * _Nullable)ids
                                              found:(BOOL)found {
    if (pixelBuffer == nil) return nil;

    CVPixelBufferLockBaseAddress(pixelBuffer, kCVPixelBufferLock_ReadOnly);
    cv::Mat color;
    const bool hasColor = CreateColorImage(pixelBuffer, color);
    CVPixelBufferUnlockBaseAddress(pixelBuffer, kCVPixelBufferLock_ReadOnly);

    if (!hasColor || color.empty()) return nil;

    if (found && corners != nil && ids != nil) {
        auto cornersVec = nsArrayToCorners(corners);
        std::vector<int> idsVec;
        for (NSNumber *num in ids) idsVec.push_back([num intValue]);
        
        cv::aruco::drawDetectedCornersCharuco(color, cornersVec, idsVec, cv::Scalar(0, 255, 0));
    }

    std::vector<uchar> buffer;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 85};
    if (!cv::imencode(".jpg", color, buffer, params)) return nil;
    return [NSData dataWithBytes:buffer.data() length:buffer.size()];
}

- (NSData * _Nullable)visualizeStereoCharucoInPixelBuffer:(CVPixelBufferRef)pixelBuffer
                                              leftCorners:(NSArray<NSValue *> * _Nullable)leftCorners
                                                  leftIds:(NSArray<NSNumber *> * _Nullable)leftIds
                                             rightCorners:(NSArray<NSValue *> * _Nullable)rightCorners
                                                 rightIds:(NSArray<NSNumber *> * _Nullable)rightIds
                                                foundLeft:(BOOL)foundLeft
                                               foundRight:(BOOL)foundRight {
    if (pixelBuffer == nil) return nil;

    CVPixelBufferLockBaseAddress(pixelBuffer, kCVPixelBufferLock_ReadOnly);
    cv::Mat color;
    const bool hasColor = CreateColorImage(pixelBuffer, color);
    CVPixelBufferUnlockBaseAddress(pixelBuffer, kCVPixelBufferLock_ReadOnly);

    if (!hasColor || color.empty()) return nil;

    int width = color.cols;
    int halfWidth = width / 2;
    int height = color.rows;

    // Draw Left
    if (foundLeft && leftCorners != nil && leftIds != nil) {
        cv::Mat leftHalf = color(cv::Rect(0, 0, halfWidth, height));
        auto cornersVec = nsArrayToCorners(leftCorners);
        std::vector<int> idsVec;
        for (NSNumber *num in leftIds) idsVec.push_back([num intValue]);
        
        cv::aruco::drawDetectedCornersCharuco(leftHalf, cornersVec, idsVec, cv::Scalar(0, 255, 0));
    }

    // Draw Right
    if (foundRight && rightCorners != nil && rightIds != nil) {
        auto cornersVec = nsArrayToCorners(rightCorners);
        std::vector<int> idsVec;
        for (NSNumber *num in rightIds) idsVec.push_back([num intValue]);
        
        // Offset
        for (auto& corner : cornersVec) corner.x += halfWidth;
        
        cv::Mat fullImage = color;
        cv::aruco::drawDetectedCornersCharuco(fullImage, cornersVec, idsVec, cv::Scalar(0, 255, 0));
    }

    std::vector<uchar> buffer;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 85};
    if (!cv::imencode(".jpg", color, buffer, params)) return nil;
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
        case ArucoDictionaryTypeDict4X4_50: return cv::aruco::DICT_4X4_50;
        case ArucoDictionaryTypeDict4X4_100: return cv::aruco::DICT_4X4_100;
        case ArucoDictionaryTypeDict4X4_250: return cv::aruco::DICT_4X4_250;
        case ArucoDictionaryTypeDict4X4_1000: return cv::aruco::DICT_4X4_1000;
        case ArucoDictionaryTypeDict5X5_50: return cv::aruco::DICT_5X5_50;
        case ArucoDictionaryTypeDict5X5_100: return cv::aruco::DICT_5X5_100;
        case ArucoDictionaryTypeDict5X5_250: return cv::aruco::DICT_5X5_250;
        case ArucoDictionaryTypeDict5X5_1000: return cv::aruco::DICT_5X5_1000;
        case ArucoDictionaryTypeDict6X6_50: return cv::aruco::DICT_6X6_50;
        case ArucoDictionaryTypeDict6X6_100: return cv::aruco::DICT_6X6_100;
        case ArucoDictionaryTypeDict6X6_250: return cv::aruco::DICT_6X6_250;
        case ArucoDictionaryTypeDict6X6_1000: return cv::aruco::DICT_6X6_1000;
        case ArucoDictionaryTypeDict7X7_50: return cv::aruco::DICT_7X7_50;
        case ArucoDictionaryTypeDict7X7_100: return cv::aruco::DICT_7X7_100;
        case ArucoDictionaryTypeDict7X7_250: return cv::aruco::DICT_7X7_250;
        case ArucoDictionaryTypeDict7X7_1000: return cv::aruco::DICT_7X7_1000;
        case ArucoDictionaryTypeDictOriginal: return cv::aruco::DICT_ARUCO_ORIGINAL;
        case ArucoDictionaryTypeDictAprilTag16h5: return cv::aruco::DICT_APRILTAG_16h5;
        case ArucoDictionaryTypeDictAprilTag25h9: return cv::aruco::DICT_APRILTAG_25h9;
        case ArucoDictionaryTypeDictAprilTag36h10: return cv::aruco::DICT_APRILTAG_36h10;
        case ArucoDictionaryTypeDictAprilTag36h11: return cv::aruco::DICT_APRILTAG_36h11;
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
        case ArucoDictionaryTypeDict4X4_50: cvType = cv::aruco::DICT_4X4_50; break;
        case ArucoDictionaryTypeDict4X4_100: cvType = cv::aruco::DICT_4X4_100; break;
        case ArucoDictionaryTypeDict4X4_250: cvType = cv::aruco::DICT_4X4_250; break;
        case ArucoDictionaryTypeDict4X4_1000: cvType = cv::aruco::DICT_4X4_1000; break;
        case ArucoDictionaryTypeDict5X5_50: cvType = cv::aruco::DICT_5X5_50; break;
        case ArucoDictionaryTypeDict5X5_100: cvType = cv::aruco::DICT_5X5_100; break;
        case ArucoDictionaryTypeDict5X5_250: cvType = cv::aruco::DICT_5X5_250; break;
        case ArucoDictionaryTypeDict5X5_1000: cvType = cv::aruco::DICT_5X5_1000; break;
        case ArucoDictionaryTypeDict6X6_50: cvType = cv::aruco::DICT_6X6_50; break;
        case ArucoDictionaryTypeDict6X6_100: cvType = cv::aruco::DICT_6X6_100; break;
        case ArucoDictionaryTypeDict6X6_250: cvType = cv::aruco::DICT_6X6_250; break;
        case ArucoDictionaryTypeDict6X6_1000: cvType = cv::aruco::DICT_6X6_1000; break;
        case ArucoDictionaryTypeDict7X7_50: cvType = cv::aruco::DICT_7X7_50; break;
        case ArucoDictionaryTypeDict7X7_100: cvType = cv::aruco::DICT_7X7_100; break;
        case ArucoDictionaryTypeDict7X7_250: cvType = cv::aruco::DICT_7X7_250; break;
        case ArucoDictionaryTypeDict7X7_1000: cvType = cv::aruco::DICT_7X7_1000; break;
        case ArucoDictionaryTypeDictOriginal: cvType = cv::aruco::DICT_ARUCO_ORIGINAL; break;
        case ArucoDictionaryTypeDictAprilTag16h5: cvType = cv::aruco::DICT_APRILTAG_16h5; break;
        case ArucoDictionaryTypeDictAprilTag25h9: cvType = cv::aruco::DICT_APRILTAG_25h9; break;
        case ArucoDictionaryTypeDictAprilTag36h10: cvType = cv::aruco::DICT_APRILTAG_36h10; break;
        case ArucoDictionaryTypeDictAprilTag36h11: cvType = cv::aruco::DICT_APRILTAG_36h11; break;
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

- (NSData * _Nullable)visualizeMarkersInPixelBuffer:(CVPixelBufferRef)pixelBuffer
                                         detections:(NSArray<ArucoDetectionResult *> *)detections
                                           drawAxes:(BOOL)drawAxes
                                       cameraMatrix:(NSArray<NSNumber *> * _Nullable)cameraMatrix
                                         distCoeffs:(NSArray<NSNumber *> * _Nullable)distCoeffs
                                         axisLength:(float)axisLength {
    if (pixelBuffer == nil || detections == nil || detections.count == 0) {
        return nil;
    }

    CVPixelBufferLockBaseAddress(pixelBuffer, kCVPixelBufferLock_ReadOnly);
    cv::Mat color;
    const bool hasColor = CreateColorImage(pixelBuffer, color);
    CVPixelBufferUnlockBaseAddress(pixelBuffer, kCVPixelBufferLock_ReadOnly);

    if (!hasColor || color.empty()) {
        return nil;
    }

    // Build camera matrix if provided (for axis drawing)
    cv::Mat K, dist;
    bool canDrawAxes = drawAxes && cameraMatrix != nil && cameraMatrix.count == 9;
    if (canDrawAxes) {
        K = cv::Mat::zeros(3, 3, CV_64F);
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                K.at<double>(i, j) = cameraMatrix[i * 3 + j].doubleValue;
            }
        }
        if (distCoeffs != nil) {
            dist = cv::Mat::zeros(1, static_cast<int>(distCoeffs.count), CV_64F);
            for (NSUInteger i = 0; i < distCoeffs.count; ++i) {
                dist.at<double>(0, static_cast<int>(i)) = distCoeffs[i].doubleValue;
            }
        } else {
            dist = cv::Mat::zeros(1, 5, CV_64F);
        }
    }

    // Draw each detected marker
    for (ArucoDetectionResult *detection in detections) {
        if (detection.corners.count != 4) {
            continue;
        }

        // Convert corners to OpenCV format
        std::vector<cv::Point2f> markerCorners;
        for (NSValue *value in detection.corners) {
            CGPoint point = [value CGPointValue];
            markerCorners.push_back(cv::Point2f(static_cast<float>(point.x), static_cast<float>(point.y)));
        }

        // Draw marker outline (thick green line)
        for (int j = 0; j < 4; ++j) {
            cv::line(color, 
                     cv::Point(static_cast<int>(markerCorners[j].x), static_cast<int>(markerCorners[j].y)),
                     cv::Point(static_cast<int>(markerCorners[(j + 1) % 4].x), static_cast<int>(markerCorners[(j + 1) % 4].y)),
                     cv::Scalar(0, 255, 0), 3);
        }

        // Draw corner circles
        for (int j = 0; j < 4; ++j) {
            cv::Scalar cornerColor = (j == 0) ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 0, 255);  // First corner blue, others red
            cv::circle(color, 
                       cv::Point(static_cast<int>(markerCorners[j].x), static_cast<int>(markerCorners[j].y)),
                       6, cornerColor, -1);
        }

        // Draw marker ID
        cv::Point textPos(static_cast<int>(markerCorners[0].x), static_cast<int>(markerCorners[0].y) - 10);
        std::string idText = "ID: " + std::to_string(detection.markerId);
        cv::putText(color, idText, textPos, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 0), 2);

        // Draw pose axes if available and requested
        if (canDrawAxes && detection.poseValid && detection.rvec != nil && detection.tvec != nil) {
            cv::Vec3d rvec, tvec;
            rvec[0] = detection.rvec[0].doubleValue;
            rvec[1] = detection.rvec[1].doubleValue;
            rvec[2] = detection.rvec[2].doubleValue;
            tvec[0] = detection.tvec[0].doubleValue;
            tvec[1] = detection.tvec[1].doubleValue;
            tvec[2] = detection.tvec[2].doubleValue;

            cv::drawFrameAxes(color, K, dist, rvec, tvec, axisLength, 3);
        }
    }

    // Encode as JPEG
    std::vector<uchar> buffer;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 85};
    if (!cv::imencode(".jpg", color, buffer, params)) {
        return nil;
    }

    return [NSData dataWithBytes:buffer.data() length:buffer.size()];
}

- (NSData * _Nullable)visualizeStereoMarkersInPixelBuffer:(CVPixelBufferRef)pixelBuffer
                                           leftDetections:(NSArray<ArucoDetectionResult *> *)leftDetections
                                          rightDetections:(NSArray<ArucoDetectionResult *> *)rightDetections {
    if (pixelBuffer == nil) {
        return nil;
    }

    BOOL hasAnyDetections = (leftDetections != nil && leftDetections.count > 0) || 
                            (rightDetections != nil && rightDetections.count > 0);
    if (!hasAnyDetections) {
        return nil;
    }

    CVPixelBufferLockBaseAddress(pixelBuffer, kCVPixelBufferLock_ReadOnly);
    cv::Mat color;
    const bool hasColor = CreateColorImage(pixelBuffer, color);
    CVPixelBufferUnlockBaseAddress(pixelBuffer, kCVPixelBufferLock_ReadOnly);

    if (!hasColor || color.empty()) {
        return nil;
    }

    int width = color.cols;
    int halfWidth = width / 2;

    // Helper lambda to draw a marker
    auto drawMarker = [&color](ArucoDetectionResult *detection, int xOffset) {
        if (detection.corners.count != 4) {
            return;
        }

        // Convert and offset corners
        std::vector<cv::Point2f> markerCorners;
        for (NSValue *value in detection.corners) {
            CGPoint point = [value CGPointValue];
            markerCorners.push_back(cv::Point2f(static_cast<float>(point.x) + xOffset, static_cast<float>(point.y)));
        }

        // Draw marker outline (thick green line)
        for (int j = 0; j < 4; ++j) {
            cv::line(color, 
                     cv::Point(static_cast<int>(markerCorners[j].x), static_cast<int>(markerCorners[j].y)),
                     cv::Point(static_cast<int>(markerCorners[(j + 1) % 4].x), static_cast<int>(markerCorners[(j + 1) % 4].y)),
                     cv::Scalar(0, 255, 0), 3);
        }

        // Draw corner circles
        for (int j = 0; j < 4; ++j) {
            cv::Scalar cornerColor = (j == 0) ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 0, 255);
            cv::circle(color, 
                       cv::Point(static_cast<int>(markerCorners[j].x), static_cast<int>(markerCorners[j].y)),
                       6, cornerColor, -1);
        }

        // Draw marker ID
        cv::Point textPos(static_cast<int>(markerCorners[0].x), static_cast<int>(markerCorners[0].y) - 10);
        std::string idText = "ID: " + std::to_string(detection.markerId);
        cv::putText(color, idText, textPos, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 0), 2);
    };

    // Draw left detections (no offset)
    if (leftDetections != nil) {
        for (ArucoDetectionResult *detection in leftDetections) {
            drawMarker(detection, 0);
        }
    }

    // Draw right detections (offset by halfWidth)
    if (rightDetections != nil) {
        for (ArucoDetectionResult *detection in rightDetections) {
            drawMarker(detection, halfWidth);
        }
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
