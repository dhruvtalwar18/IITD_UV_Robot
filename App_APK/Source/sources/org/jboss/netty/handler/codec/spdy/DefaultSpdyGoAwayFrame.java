package org.jboss.netty.handler.codec.spdy;

import org.jboss.netty.util.internal.StringUtil;

public class DefaultSpdyGoAwayFrame implements SpdyGoAwayFrame {
    private int lastGoodStreamId;
    private SpdySessionStatus status;

    public DefaultSpdyGoAwayFrame(int lastGoodStreamId2) {
        this(lastGoodStreamId2, 0);
    }

    public DefaultSpdyGoAwayFrame(int lastGoodStreamId2, int statusCode) {
        this(lastGoodStreamId2, SpdySessionStatus.valueOf(statusCode));
    }

    public DefaultSpdyGoAwayFrame(int lastGoodStreamId2, SpdySessionStatus status2) {
        setLastGoodStreamId(lastGoodStreamId2);
        setStatus(status2);
    }

    public int getLastGoodStreamID() {
        return getLastGoodStreamId();
    }

    public int getLastGoodStreamId() {
        return this.lastGoodStreamId;
    }

    public void setLastGoodStreamID(int lastGoodStreamId2) {
        setLastGoodStreamId(lastGoodStreamId2);
    }

    public void setLastGoodStreamId(int lastGoodStreamId2) {
        if (lastGoodStreamId2 >= 0) {
            this.lastGoodStreamId = lastGoodStreamId2;
            return;
        }
        throw new IllegalArgumentException("Last-good-stream-ID cannot be negative: " + lastGoodStreamId2);
    }

    public SpdySessionStatus getStatus() {
        return this.status;
    }

    public void setStatus(SpdySessionStatus status2) {
        this.status = status2;
    }

    public String toString() {
        return getClass().getSimpleName() + StringUtil.NEWLINE + "--> Last-good-stream-ID = " + this.lastGoodStreamId + StringUtil.NEWLINE + "--> Status: " + this.status.toString();
    }
}
