package org.apache.commons.io.input;

import java.io.FilterInputStream;
import java.io.IOException;
import java.io.InputStream;

public abstract class ProxyInputStream extends FilterInputStream {
    public ProxyInputStream(InputStream proxy) {
        super(proxy);
    }

    public int read() throws IOException {
        return this.in.read();
    }

    public int read(byte[] bts) throws IOException {
        return this.in.read(bts);
    }

    public int read(byte[] bts, int st, int end) throws IOException {
        return this.in.read(bts, st, end);
    }

    public long skip(long ln) throws IOException {
        return this.in.skip(ln);
    }

    public int available() throws IOException {
        return this.in.available();
    }

    public void close() throws IOException {
        this.in.close();
    }

    public synchronized void mark(int idx) {
        this.in.mark(idx);
    }

    public synchronized void reset() throws IOException {
        this.in.reset();
    }

    public boolean markSupported() {
        return this.in.markSupported();
    }
}
