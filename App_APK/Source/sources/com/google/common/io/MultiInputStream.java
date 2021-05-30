package com.google.common.io;

import java.io.IOException;
import java.io.InputStream;
import java.util.Iterator;

final class MultiInputStream extends InputStream {
    private InputStream in;
    private Iterator<? extends InputSupplier<? extends InputStream>> it;

    public MultiInputStream(Iterator<? extends InputSupplier<? extends InputStream>> it2) throws IOException {
        this.it = it2;
        advance();
    }

    public void close() throws IOException {
        if (this.in != null) {
            try {
                this.in.close();
            } finally {
                this.in = null;
            }
        }
    }

    private void advance() throws IOException {
        close();
        if (this.it.hasNext()) {
            this.in = (InputStream) ((InputSupplier) this.it.next()).getInput();
        }
    }

    public int available() throws IOException {
        if (this.in == null) {
            return 0;
        }
        return this.in.available();
    }

    public boolean markSupported() {
        return false;
    }

    public int read() throws IOException {
        if (this.in == null) {
            return -1;
        }
        int result = this.in.read();
        if (result != -1) {
            return result;
        }
        advance();
        return read();
    }

    public int read(byte[] b, int off, int len) throws IOException {
        if (this.in == null) {
            return -1;
        }
        int result = this.in.read(b, off, len);
        if (result != -1) {
            return result;
        }
        advance();
        return read(b, off, len);
    }

    public long skip(long n) throws IOException {
        if (this.in == null || n <= 0) {
            return 0;
        }
        long result = this.in.skip(n);
        if (result != 0) {
            return result;
        }
        if (read() == -1) {
            return 0;
        }
        return this.in.skip(n - 1) + 1;
    }
}
