package org.apache.commons.net.io;

import java.io.FilterOutputStream;
import java.io.IOException;
import java.io.OutputStream;

public final class FromNetASCIIOutputStream extends FilterOutputStream {
    private boolean __lastWasCR = false;

    public FromNetASCIIOutputStream(OutputStream output) {
        super(output);
    }

    private void __write(int ch) throws IOException {
        if (ch != 10) {
            if (ch != 13) {
                if (this.__lastWasCR) {
                    this.out.write(13);
                    this.__lastWasCR = false;
                }
                this.out.write(ch);
                return;
            }
            this.__lastWasCR = true;
        } else if (this.__lastWasCR) {
            this.out.write(FromNetASCIIInputStream._lineSeparatorBytes);
            this.__lastWasCR = false;
        } else {
            this.__lastWasCR = false;
            this.out.write(10);
        }
    }

    public synchronized void write(int ch) throws IOException {
        if (FromNetASCIIInputStream._noConversionRequired) {
            this.out.write(ch);
        } else {
            __write(ch);
        }
    }

    public synchronized void write(byte[] buffer) throws IOException {
        write(buffer, 0, buffer.length);
    }

    public synchronized void write(byte[] buffer, int offset, int offset2) throws IOException {
        if (FromNetASCIIInputStream._noConversionRequired) {
            this.out.write(buffer, offset, offset2);
            return;
        }
        while (true) {
            int length = offset2 - 1;
            if (offset2 > 0) {
                int offset3 = offset + 1;
                __write(buffer[offset]);
                offset = offset3;
                offset2 = length;
            } else {
                return;
            }
        }
    }

    public synchronized void close() throws IOException {
        if (FromNetASCIIInputStream._noConversionRequired) {
            super.close();
            return;
        }
        if (this.__lastWasCR) {
            this.out.write(13);
        }
        super.close();
    }
}
