package org.jboss.netty.handler.codec.spdy;

import org.jboss.netty.buffer.ChannelBuffer;

abstract class SpdyHeaderBlockDecompressor {
    /* access modifiers changed from: package-private */
    public abstract void decode(ChannelBuffer channelBuffer) throws Exception;

    /* access modifiers changed from: package-private */
    public abstract void end();

    /* access modifiers changed from: package-private */
    public abstract void setInput(ChannelBuffer channelBuffer);

    SpdyHeaderBlockDecompressor() {
    }

    static SpdyHeaderBlockDecompressor newInstance(int version) {
        return new SpdyHeaderBlockZlibDecompressor(version);
    }
}
