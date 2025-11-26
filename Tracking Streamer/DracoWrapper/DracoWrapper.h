#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN

@interface DracoDecodedData : NSObject
@property (nonatomic, readonly) NSData *positions; // Float32 [x, y, z, ...]
@property (nonatomic, readonly) NSData *colors;    // UInt8 [r, g, b, ...] (empty if none)
@property (nonatomic, readonly) NSInteger pointCount;
@end

@interface DracoWrapper : NSObject
+ (nullable DracoDecodedData *)decode:(NSData *)data;
@end

NS_ASSUME_NONNULL_END
