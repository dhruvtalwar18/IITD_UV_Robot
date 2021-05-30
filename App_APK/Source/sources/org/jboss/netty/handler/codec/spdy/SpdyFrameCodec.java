package org.jboss.netty.handler.codec.spdy;

import org.jboss.netty.channel.ChannelDownstreamHandler;
import org.jboss.netty.channel.ChannelEvent;
import org.jboss.netty.channel.ChannelHandlerContext;
import org.jboss.netty.channel.ChannelUpstreamHandler;

public class SpdyFrameCodec implements ChannelUpstreamHandler, ChannelDownstreamHandler {
    private final SpdyFrameDecoder decoder;
    private final SpdyFrameEncoder encoder;

    @Deprecated
    public SpdyFrameCodec() {
        this(2);
    }

    public SpdyFrameCodec(int version) {
        this(version, 8192, 16384, 6, 15, 8);
    }

    public SpdyFrameCodec(int version, int maxChunkSize, int maxHeaderSize, int compressionLevel, int windowBits, int memLevel) {
        this.decoder = new SpdyFrameDecoder(version, maxChunkSize, maxHeaderSize);
        this.encoder = new SpdyFrameEncoder(version, compressionLevel, windowBits, memLevel);
    }

    public void handleUpstream(ChannelHandlerContext ctx, ChannelEvent e) throws Exception {
        this.decoder.handleUpstream(ctx, e);
    }

    public void handleDownstream(ChannelHandlerContext ctx, ChannelEvent e) throws Exception {
        this.encoder.handleDownstream(ctx, e);
    }
}
