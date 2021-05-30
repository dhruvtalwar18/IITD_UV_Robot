package org.apache.commons.io.input;

import java.io.FilterReader;
import java.io.IOException;
import java.io.Reader;

public abstract class ProxyReader extends FilterReader {
    public ProxyReader(Reader proxy) {
        super(proxy);
    }

    public int read() throws IOException {
        return this.in.read();
    }

    public int read(char[] chr) throws IOException {
        return this.in.read(chr);
    }

    public int read(char[] chr, int st, int end) throws IOException {
        return this.in.read(chr, st, end);
    }

    public long skip(long ln) throws IOException {
        return this.in.skip(ln);
    }

    public boolean ready() throws IOException {
        return this.in.ready();
    }

    public void close() throws IOException {
        this.in.close();
    }

    public synchronized void mark(int idx) throws IOException {
        this.in.mark(idx);
    }

    public synchronized void reset() throws IOException {
        this.in.reset();
    }

    public boolean markSupported() {
        return this.in.markSupported();
    }
}
