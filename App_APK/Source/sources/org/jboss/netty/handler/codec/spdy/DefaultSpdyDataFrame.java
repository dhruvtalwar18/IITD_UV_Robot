package org.jboss.netty.handler.codec.spdy;

import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;
import org.jboss.netty.util.internal.StringUtil;

public class DefaultSpdyDataFrame implements SpdyDataFrame {
    private boolean compressed;
    private ChannelBuffer data = ChannelBuffers.EMPTY_BUFFER;
    private boolean last;
    private int streamId;

    public DefaultSpdyDataFrame(int streamId2) {
        setStreamId(streamId2);
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

    public boolean isLast() {
        return this.last;
    }

    public void setLast(boolean last2) {
        this.last = last2;
    }

    public boolean isCompressed() {
        return this.compressed;
    }

    public void setCompressed(boolean compressed2) {
        this.compressed = compressed2;
    }

    public ChannelBuffer getData() {
        return this.data;
    }

    public void setData(ChannelBuffer data2) {
        if (data2 == null) {
            data2 = ChannelBuffers.EMPTY_BUFFER;
        }
        if (data2.readableBytes() <= 16777215) {
            this.data = data2;
            return;
        }
        throw new IllegalArgumentException("data payload cannot exceed 16777215 bytes");
    }

    public String toString() {
        return getClass().getSimpleName() + "(last: " + isLast() + "; compressed: " + isCompressed() + ')' + StringUtil.NEWLINE + "--> Stream-ID = " + this.streamId + StringUtil.NEWLINE + "--> Size = " + this.data.readableBytes();
    }
}
