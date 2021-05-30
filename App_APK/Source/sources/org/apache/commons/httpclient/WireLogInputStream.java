package org.apache.commons.httpclient;

import java.io.FilterInputStream;
import java.io.IOException;
import java.io.InputStream;

class WireLogInputStream extends FilterInputStream {
    private InputStream in;
    private Wire wire;

    public WireLogInputStream(InputStream in2, Wire wire2) {
        super(in2);
        this.in = in2;
        this.wire = wire2;
    }

    public int read(byte[] b, int off, int len) throws IOException {
        int l = this.in.read(b, off, len);
        if (l > 0) {
            this.wire.input(b, off, l);
        }
        return l;
    }

    public int read() throws IOException {
        int l = this.in.read();
        if (l > 0) {
            this.wire.input(l);
        }
        return l;
    }

    public int read(byte[] b) throws IOException {
        int l = this.in.read(b);
        if (l > 0) {
            this.wire.input(b, 0, l);
        }
        return l;
    }
}
