// Minimal C interface to Syphon.framework for Photon Engine.
// Frames are received as Metal textures and copied back to CPU memory (BGRA8).

#import <Foundation/Foundation.h>
#import <Metal/Metal.h>
#import <Syphon/Syphon.h>
#include <stdint.h>
#include <string.h>

static NSArray<NSDictionary *> *gSnapshot = nil;

static void copy_str(NSString *s, char *dst, int len) {
    if (!dst || len <= 0) return;
    const char *c = s ? [s UTF8String] : "";
    strncpy(dst, c ? c : "", (size_t)len - 1);
    dst[len - 1] = 0;
}

// Refresh the server list snapshot and return how many servers there are.
int pe_syphon_server_count(void) {
    @autoreleasepool {
        NSArray *servers = [[SyphonServerDirectory sharedDirectory] servers];
        @synchronized([SyphonServerDirectory class]) {
            gSnapshot = [servers copy];
            return (int)gSnapshot.count;
        }
    }
}

int pe_syphon_server_info(int index, char *app, char *name, int len) {
    @autoreleasepool {
        @synchronized([SyphonServerDirectory class]) {
            if (!gSnapshot || index < 0 || index >= (int)gSnapshot.count) return 0;
            NSDictionary *d = gSnapshot[(NSUInteger)index];
            copy_str(d[SyphonServerDescriptionAppNameKey], app, len);
            copy_str(d[SyphonServerDescriptionNameKey], name, len);
            return 1;
        }
    }
}

@interface PEReceiver : NSObject
@property(nonatomic, strong) SyphonMetalClient *client;
@property(nonatomic, strong) id<MTLDevice> device;
@property(nonatomic, strong) id<MTLCommandQueue> queue;
@property(nonatomic, strong) id<MTLBuffer> buffer;
@property(atomic, assign) uint64_t frames;
@property(nonatomic, assign) uint64_t consumed;
@end

@implementation PEReceiver
@end

// Connect to the first server matching app and name (either may be empty to match anything).
void *pe_syphon_open(const char *app, const char *name) {
    @autoreleasepool {
        NSString *wantApp = [NSString stringWithUTF8String:app ? app : ""];
        NSString *wantName = [NSString stringWithUTF8String:name ? name : ""];
        NSDictionary *match = nil;
        for (NSDictionary *d in [[SyphonServerDirectory sharedDirectory] servers]) {
            NSString *a = d[SyphonServerDescriptionAppNameKey] ?: @"";
            NSString *n = d[SyphonServerDescriptionNameKey] ?: @"";
            if ((wantApp.length == 0 || [a isEqualToString:wantApp]) &&
                (wantName.length == 0 || [n isEqualToString:wantName])) {
                match = d;
                break;
            }
        }
        if (!match) return NULL;

        PEReceiver *r = [PEReceiver new];
        r.device = MTLCreateSystemDefaultDevice();
        if (!r.device) return NULL;
        r.queue = [r.device newCommandQueue];
        __weak PEReceiver *weak = r;
        r.client = [[SyphonMetalClient alloc] initWithServerDescription:match
                                                                 device:r.device
                                                                options:nil
                                                        newFrameHandler:^(SyphonMetalClient *c) {
                                                            PEReceiver *s = weak;
                                                            if (s) s.frames = s.frames + 1;
                                                        }];
        if (!r.client) return NULL;
        return (__bridge_retained void *)r;
    }
}

// 1 = new frame copied, 0 = nothing new, -1 = disconnected, -2 = dst too small (w/h set).
int pe_syphon_read(void *handle, uint8_t *dst, size_t cap, int *out_w, int *out_h) {
    @autoreleasepool {
        PEReceiver *r = (__bridge PEReceiver *)handle;
        if (!r.client.isValid) return -1;
        uint64_t frames = r.frames;
        if (frames == r.consumed) return 0;

        id<MTLTexture> tex = [r.client newFrameImage];
        if (!tex) return 0;
        NSUInteger w = tex.width, h = tex.height;
        *out_w = (int)w;
        *out_h = (int)h;
        size_t bytes = w * h * 4;
        if (cap < bytes) return -2;
        r.consumed = frames;

        if (!r.buffer || r.buffer.length < bytes) {
            r.buffer = [r.device newBufferWithLength:bytes options:MTLResourceStorageModeShared];
        }
        id<MTLCommandBuffer> cb = [r.queue commandBuffer];
        id<MTLBlitCommandEncoder> blit = [cb blitCommandEncoder];
        [blit copyFromTexture:tex
                     sourceSlice:0
                     sourceLevel:0
                    sourceOrigin:MTLOriginMake(0, 0, 0)
                      sourceSize:MTLSizeMake(w, h, 1)
                        toBuffer:r.buffer
               destinationOffset:0
          destinationBytesPerRow:w * 4
        destinationBytesPerImage:bytes];
        [blit endEncoding];
        [cb commit];
        [cb waitUntilCompleted];
        memcpy(dst, r.buffer.contents, bytes);
        return 1;
    }
}

void pe_syphon_close(void *handle) {
    @autoreleasepool {
        if (!handle) return;
        PEReceiver *r = (__bridge_transfer PEReceiver *)handle;
        [r.client stop];
        r.client = nil;
    }
}
