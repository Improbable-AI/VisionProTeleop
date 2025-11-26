#import "DracoWrapper.h"

#ifdef __cplusplus
#include <draco/compression/decode.h>
#include <draco/point_cloud/point_cloud.h>
#include <vector>
#endif

@implementation DracoDecodedData
- (instancetype)initWithPositions:(NSData *)positions colors:(NSData *)colors pointCount:(NSInteger)pointCount {
    self = [super init];
    if (self) {
        _positions = positions;
        _colors = colors;
        _pointCount = pointCount;
    }
    return self;
}
@end

@implementation DracoWrapper

+ (nullable DracoDecodedData *)decode:(NSData *)data {
#ifdef __cplusplus
    if (data.length == 0) return nil;

    draco::DecoderBuffer buffer;
    buffer.Init((const char *)data.bytes, data.length);

    draco::Decoder decoder;
    auto statusor = decoder.DecodePointCloudFromBuffer(&buffer);
    if (!statusor.ok()) {
        printf("❌ [DracoWrapper] Decoding failed: %s\n", statusor.status().error_msg());
        return nil;
    } else {
        // printf("✅ [DracoWrapper] Decoding successful!\n");
    }

    std::unique_ptr<draco::PointCloud> pc = std::move(statusor).value();
    if (!pc) {
        printf("❌ [DracoWrapper] PointCloud is null\n");
        return nil;
    }

    int pointCount = pc->num_points();
    // printf("ℹ️ [DracoWrapper] Point count: %d\n", pointCount);
    if (pointCount == 0) return nil;

    // 1. Extract Positions
    const draco::PointAttribute *posAtt = pc->GetNamedAttribute(draco::GeometryAttribute::POSITION);
    if (!posAtt) return nil;

    NSMutableData *posData = [NSMutableData dataWithLength:pointCount * 3 * sizeof(float)];
    float *posPtr = (float *)posData.mutableBytes;

    for (draco::PointIndex i(0); i < pointCount; ++i) {
        draco::Vector3f val;
        posAtt->GetMappedValue(i, &val[0]);
        posPtr[i.value() * 3 + 0] = val[0];
        posPtr[i.value() * 3 + 1] = val[1];
        posPtr[i.value() * 3 + 2] = val[2];
    }

    // 2. Extract Colors (if any)
    NSMutableData *colData = [NSMutableData data];
    const draco::PointAttribute *colAtt = pc->GetNamedAttribute(draco::GeometryAttribute::COLOR);
    if (colAtt) {
        [colData setLength:pointCount * 4 * sizeof(uint8_t)]; // RGBA = 4 bytes
        uint8_t *colPtr = (uint8_t *)colData.mutableBytes;

        for (draco::PointIndex i(0); i < pointCount; ++i) {
            // Colors are usually uint8
            if (colAtt->data_type() == draco::DT_UINT8) {
                uint8_t val[3];
                colAtt->GetMappedValue(i, &val[0]);
                colPtr[i.value() * 4 + 0] = val[0];
                colPtr[i.value() * 4 + 1] = val[1];
                colPtr[i.value() * 4 + 2] = val[2];
                colPtr[i.value() * 4 + 3] = 255; // Alpha
            } else {
                // Handle other types if needed, but usually it's uint8
                // Fallback to 0 if unknown
            }
        }
    }

    return [[DracoDecodedData alloc] initWithPositions:posData colors:colData pointCount:pointCount];
#else
    return nil;
#endif
}

@end
