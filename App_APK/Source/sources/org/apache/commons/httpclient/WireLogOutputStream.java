package org.apache.commons.httpclient;

import java.io.FilterOutputStream;
import java.io.IOException;
import java.io.OutputStream;

class WireLogOutputStream extends FilterOutputStream {
    private OutputStream out;
    private Wire wire;

    public WireLogOutputStream(OutputStream out2, Wire wire2) {
        super(out2);
        this.out = out2;
        this.wire = wire2;
    }

    public void write(byte[] b, int off, int len) throws IOException {
        this.out.write(b, off, len);
        this.wire.output(b, off, len);
    }

    public void write(int b) throws IOException {
        this.out.write(b);
        this.wire.output(b);
    }

    public void write(byte[] b) throws IOException {
        this.out.write(b);
        this.wire.output(b);
    }
}
