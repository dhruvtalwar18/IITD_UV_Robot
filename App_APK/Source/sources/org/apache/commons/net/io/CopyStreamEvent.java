package org.apache.commons.net.io;

import java.util.EventObject;

public class CopyStreamEvent extends EventObject {
    public static final long UNKNOWN_STREAM_SIZE = -1;
    private int bytesTransferred;
    private long streamSize;
    private long totalBytesTransferred;

    public CopyStreamEvent(Object source, long totalBytesTransferred2, int bytesTransferred2, long streamSize2) {
        super(source);
        this.bytesTransferred = bytesTransferred2;
        this.totalBytesTransferred = totalBytesTransferred2;
        this.streamSize = streamSize2;
    }

    public int getBytesTransferred() {
        return this.bytesTransferred;
    }

    public long getTotalBytesTransferred() {
        return this.totalBytesTransferred;
    }

    public long getStreamSize() {
        return this.streamSize;
    }
}
