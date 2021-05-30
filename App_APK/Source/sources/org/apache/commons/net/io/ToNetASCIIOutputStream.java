package org.apache.commons.net.io;

import java.io.FilterOutputStream;
import java.io.IOException;
import java.io.OutputStream;

public final class ToNetASCIIOutputStream extends FilterOutputStream {
    private boolean __lastWasCR = false;

    public ToNetASCIIOutputStream(OutputStream output) {
        super(output);
    }

    public synchronized void write(int ch) throws IOException {
        if (ch != 10) {
            if (ch == 13) {
                this.__lastWasCR = true;
                this.out.write(13);
                return;
            }
        } else if (!this.__lastWasCR) {
            this.out.write(13);
        }
        this.__lastWasCR = false;
        this.out.write(ch);
    }

    public synchronized void write(byte[] buffer) throws IOException {
        write(buffer, 0, buffer.length);
    }

    public synchronized void write(byte[] buffer, int offset, int offset2) throws IOException {
        while (true) {
            int length = offset2 - 1;
            if (offset2 > 0) {
                int offset3 = offset + 1;
                write((int) buffer[offset]);
                offset = offset3;
                offset2 = length;
            }
        }
    }
}
