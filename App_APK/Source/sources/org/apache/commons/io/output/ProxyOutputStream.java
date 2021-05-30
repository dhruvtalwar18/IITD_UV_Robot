package org.apache.commons.io.output;

import java.io.FilterOutputStream;
import java.io.IOException;
import java.io.OutputStream;

public class ProxyOutputStream extends FilterOutputStream {
    public ProxyOutputStream(OutputStream proxy) {
        super(proxy);
    }

    public void write(int idx) throws IOException {
        this.out.write(idx);
    }

    public void write(byte[] bts) throws IOException {
        this.out.write(bts);
    }

    public void write(byte[] bts, int st, int end) throws IOException {
        this.out.write(bts, st, end);
    }

    public void flush() throws IOException {
        this.out.flush();
    }

    public void close() throws IOException {
        this.out.close();
    }
}
