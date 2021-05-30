package org.jboss.netty.handler.codec.spdy;

import org.jboss.netty.util.internal.StringUtil;

public class DefaultSpdySynReplyFrame extends DefaultSpdyHeaderBlock implements SpdySynReplyFrame {
    private boolean last;
    private int streamId;

    public DefaultSpdySynReplyFrame(int streamId2) {
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

    public String toString() {
        StringBuilder buf = new StringBuilder();
        buf.append(getClass().getSimpleName());
        buf.append("(last: ");
        buf.append(isLast());
        buf.append(')');
        buf.append(StringUtil.NEWLINE);
        buf.append("--> Stream-ID = ");
        buf.append(this.streamId);
        buf.append(StringUtil.NEWLINE);
        buf.append("--> Headers:");
        buf.append(StringUtil.NEWLINE);
        appendHeaders(buf);
        buf.setLength(buf.length() - StringUtil.NEWLINE.length());
        return buf.toString();
    }
}
