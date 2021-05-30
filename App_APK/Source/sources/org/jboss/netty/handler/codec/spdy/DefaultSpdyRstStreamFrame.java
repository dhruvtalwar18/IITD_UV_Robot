package org.jboss.netty.handler.codec.spdy;

import org.jboss.netty.util.internal.StringUtil;

public class DefaultSpdyRstStreamFrame implements SpdyRstStreamFrame {
    private SpdyStreamStatus status;
    private int streamId;

    public DefaultSpdyRstStreamFrame(int streamId2, int statusCode) {
        this(streamId2, SpdyStreamStatus.valueOf(statusCode));
    }

    public DefaultSpdyRstStreamFrame(int streamId2, SpdyStreamStatus status2) {
        setStreamId(streamId2);
        setStatus(status2);
    }

    public int getStreamID() {
        return getStreamId();
    }

    public int getStreamId() {
        return this.streamId;
    }

    public void setStreamID(int streamId2) {
        setStreamId(streamId2);
    }

    public void setStreamId(int streamId2) {
        if (streamId2 > 0) {
            this.streamId = streamId2;
            return;
        }
        throw new IllegalArgumentException("Stream-ID must be positive: " + streamId2);
    }

    public SpdyStreamStatus getStatus() {
        return this.status;
    }

    public void setStatus(SpdyStreamStatus status2) {
        this.status = status2;
    }

    public String toString() {
        return getClass().getSimpleName() + StringUtil.NEWLINE + "--> Stream-ID = " + this.streamId + StringUtil.NEWLINE + "--> Status: " + this.status.toString();
    }
}
