package org.apache.commons.httpclient;

import java.io.FilterInputStream;
import java.io.IOException;
import java.io.InputStream;

class AutoCloseInputStream extends FilterInputStream {
    private boolean selfClosed = false;
    private boolean streamOpen = true;
    private ResponseConsumedWatcher watcher = null;

    public AutoCloseInputStream(InputStream in, ResponseConsumedWatcher watcher2) {
        super(in);
        this.watcher = watcher2;
    }

    public int read() throws IOException {
        if (!isReadAllowed()) {
            return -1;
        }
        int l = super.read();
        checkClose(l);
        return l;
    }

    public int read(byte[] b, int off, int len) throws IOException {
        if (!isReadAllowed()) {
            return -1;
        }
        int l = super.read(b, off, len);
        checkClose(l);
        return l;
    }

    public int read(byte[] b) throws IOException {
        if (!isReadAllowed()) {
            return -1;
        }
        int l = super.read(b);
        checkClose(l);
        return l;
    }

    public int available() throws IOException {
        if (isReadAllowed()) {
            return super.available();
        }
        return 0;
    }

    public void close() throws IOException {
        if (!this.selfClosed) {
            this.selfClosed = true;
            notifyWatcher();
        }
    }

    private void checkClose(int readResult) throws IOException {
        if (readResult == -1) {
            notifyWatcher();
        }
    }

    private boolean isReadAllowed() throws IOException {
        if (this.streamOpen || !this.selfClosed) {
            return this.streamOpen;
        }
        throw new IOException("Attempted read on closed stream.");
    }

    private void notifyWatcher() throws IOException {
        if (this.streamOpen) {
            super.close();
            this.streamOpen = false;
            if (this.watcher != null) {
                this.watcher.responseConsumed();
            }
        }
    }
}
