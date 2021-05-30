package org.jboss.netty.handler.codec.spdy;

import org.jboss.netty.util.internal.StringUtil;

public class DefaultSpdySynStreamFrame extends DefaultSpdyHeaderBlock implements SpdySynStreamFrame {
    private int associatedToStreamId;
    private boolean last;
    private byte priority;
    private int streamId;
    private boolean unidirectional;

    public DefaultSpdySynStreamFrame(int streamID, int associatedToStreamId2, byte priority2) {
        setStreamId(streamID);
        setAssociatedToStreamId(associatedToStreamId2);
        setPriority(priority2);
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

    public int getAssociatedToStreamID() {
        return getAssociatedToStreamId();
    }

    public int getAssociatedToStreamId() {
        return this.associatedToStreamId;
    }

    public void setAssociatedToStreamID(int associatedToStreamId2) {
        setAssociatedToStreamId(associatedToStreamId2);
    }

    public void setAssociatedToStreamId(int associatedToStreamId2) {
        if (associatedToStreamId2 >= 0) {
            this.associatedToStreamId = associatedToStreamId2;
            return;
        }
        throw new IllegalArgumentException("Associated-To-Stream-ID cannot be negative: " + associatedToStreamId2);
    }

    public byte getPriority() {
        return this.priority;
    }

    public void setPriority(byte priority2) {
        if (priority2 < 0 || priority2 > 7) {
            throw new IllegalArgumentException("Priority must be between 0 and 7 inclusive: " + priority2);
        }
        this.priority = priority2;
    }

    public boolean isLast() {
        return this.last;
    }

    public void setLast(boolean last2) {
        this.last = last2;
    }

    public boolean isUnidirectional() {
        return this.unidirectional;
    }

    public void setUnidirectional(boolean unidirectional2) {
        this.unidirectional = unidirectional2;
    }

    public String toString() {
        StringBuilder buf = new StringBuilder();
        buf.append(getClass().getSimpleName());
        buf.append("(last: ");
        buf.append(isLast());
        buf.append("; unidirectional: ");
        buf.append(isUnidirectional());
        buf.append(')');
        buf.append(StringUtil.NEWLINE);
        buf.append("--> Stream-ID = ");
        buf.append(this.streamId);
        buf.append(StringUtil.NEWLINE);
        if (this.associatedToStreamId != 0) {
            buf.append("--> Associated-To-Stream-ID = ");
            buf.append(this.associatedToStreamId);
            buf.append(StringUtil.NEWLINE);
        }
        buf.append("--> Priority = ");
        buf.append(this.priority);
        buf.append(StringUtil.NEWLINE);
        buf.append("--> Headers:");
        buf.append(StringUtil.NEWLINE);
        appendHeaders(buf);
        buf.setLength(buf.length() - StringUtil.NEWLINE.length());
        return buf.toString();
    }
}
