package org.apache.commons.httpclient;

import java.io.IOException;
import java.io.InputStream;

public class ContentLengthInputStream extends InputStream {
    private boolean closed;
    private long contentLength;
    private long pos;
    private InputStream wrappedStream;

    public ContentLengthInputStream(InputStream in, int contentLength2) {
        this(in, (long) contentLength2);
    }

    public ContentLengthInputStream(InputStream in, long contentLength2) {
        this.pos = 0;
        this.closed = false;
        this.wrappedStream = null;
        this.wrappedStream = in;
        this.contentLength = contentLength2;
    }

    public void close() throws IOException {
        if (!this.closed) {
            try {
                ChunkedInputStream.exhaustInputStream(this);
            } finally {
                this.closed = true;
            }
        }
    }

    public int read() throws IOException {
        if (this.closed) {
            throw new IOException("Attempted read from closed stream.");
        } else if (this.pos >= this.contentLength) {
            return -1;
        } else {
            this.pos++;
            return this.wrappedStream.read();
        }
    }

    public int read(byte[] b, int off, int len) throws IOException {
        if (this.closed) {
            throw new IOException("Attempted read from closed stream.");
        } else if (this.pos >= this.contentLength) {
            return -1;
        } else {
            if (this.pos + ((long) len) > this.contentLength) {
                len = (int) (this.contentLength - this.pos);
            }
            int count = this.wrappedStream.read(b, off, len);
            this.pos += (long) count;
            return count;
        }
    }

    public int read(byte[] b) throws IOException {
        return read(b, 0, b.length);
    }

    public long skip(long n) throws IOException {
        long length = this.wrappedStream.skip(Math.min(n, this.contentLength - this.pos));
        if (length > 0) {
            this.pos += length;
        }
        return length;
    }

    public int available() throws IOException {
        if (this.closed) {
            return 0;
        }
        int avail = this.wrappedStream.available();
        if (this.pos + ((long) avail) > this.contentLength) {
            return (int) (this.contentLength - this.pos);
        }
        return avail;
    }
}
