package org.apache.commons.io.input;

import java.io.IOException;
import java.io.InputStream;

public class AutoCloseInputStream extends ProxyInputStream {
    public AutoCloseInputStream(InputStream in) {
        super(in);
    }

    public void close() throws IOException {
        this.in.close();
        this.in = new ClosedInputStream();
    }

    public int read() throws IOException {
        int n = this.in.read();
        if (n == -1) {
            close();
        }
        return n;
    }

    public int read(byte[] b) throws IOException {
        int n = this.in.read(b);
        if (n == -1) {
            close();
        }
        return n;
    }

    public int read(byte[] b, int off, int len) throws IOException {
        int n = this.in.read(b, off, len);
        if (n == -1) {
            close();
        }
        return n;
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        close();
        super.finalize();
    }
}
