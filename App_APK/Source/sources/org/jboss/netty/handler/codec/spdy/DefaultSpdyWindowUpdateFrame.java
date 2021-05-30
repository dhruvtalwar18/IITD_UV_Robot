package org.jboss.netty.handler.codec.spdy;

import org.jboss.netty.util.internal.StringUtil;

public class DefaultSpdyWindowUpdateFrame implements SpdyWindowUpdateFrame {
    private int deltaWindowSize;
    private int streamId;

    public DefaultSpdyWindowUpdateFrame(int streamId2, int deltaWindowSize2) {
        setStreamId(streamId2);
        setDeltaWindowSize(deltaWindowSize2);
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

    public int getDeltaWindowSize() {
        return this.deltaWindowSize;
    }

    public void setDeltaWindowSize(int deltaWindowSize2) {
        if (deltaWindowSize2 > 0) {
            this.deltaWindowSize = deltaWindowSize2;
            return;
        }
        throw new IllegalArgumentException("Delta-Window-Size must be positive: " + deltaWindowSize2);
    }

    public String toString() {
        return getClass().getSimpleName() + StringUtil.NEWLINE + "--> Stream-ID = " + this.streamId + StringUtil.NEWLINE + "--> Delta-Window-Size = " + this.deltaWindowSize;
    }
}
