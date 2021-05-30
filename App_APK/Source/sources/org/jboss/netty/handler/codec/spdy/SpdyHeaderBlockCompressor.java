package org.jboss.netty.handler.codec.spdy;

import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.util.internal.DetectionUtil;

abstract class SpdyHeaderBlockCompressor {
    /* access modifiers changed from: package-private */
    public abstract void encode(ChannelBuffer channelBuffer);

    /* access modifiers changed from: package-private */
    public abstract void end();

    /* access modifiers changed from: package-private */
    public abstract void setInput(ChannelBuffer channelBuffer);

    SpdyHeaderBlockCompressor() {
    }

    static SpdyHeaderBlockCompressor newInstance(int version, int compressionLevel, int windowBits, int memLevel) {
        if (DetectionUtil.javaVersion() >= 7) {
            return new SpdyHeaderBlockZlibCompressor(version, compressionLevel);
        }
        return new SpdyHeaderBlockJZlibCompressor(version, compressionLevel, windowBits, memLevel);
    }
}
