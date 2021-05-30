package org.apache.xmlrpc.util;

import java.io.IOException;
import java.io.InputStream;

public class LimitedInputStream extends InputStream {
    private long available;
    private InputStream in;
    private long markedAvailable;

    public LimitedInputStream(InputStream pIn, int pAvailable) {
        this.in = pIn;
        this.available = (long) pAvailable;
    }

    public int read() throws IOException {
        if (this.available <= 0) {
            return -1;
        }
        this.available--;
        return this.in.read();
    }

    public int read(byte[] b, int off, int len) throws IOException {
        if (this.available <= 0) {
            return -1;
        }
        if (((long) len) > this.available) {
            len = (int) this.available;
        }
        int read = this.in.read(b, off, len);
        if (read == -1) {
            this.available = 0;
        } else {
            this.available -= (long) read;
        }
        return read;
    }

    public long skip(long n) throws IOException {
        long skip = this.in.skip(n);
        if (this.available > 0) {
            this.available -= skip;
        }
        return skip;
    }

    public void mark(int readlimit) {
        this.in.mark(readlimit);
        this.markedAvailable = this.available;
    }

    public void reset() throws IOException {
        this.in.reset();
        this.available = this.markedAvailable;
    }

    public boolean markSupported() {
        return true;
    }
}
