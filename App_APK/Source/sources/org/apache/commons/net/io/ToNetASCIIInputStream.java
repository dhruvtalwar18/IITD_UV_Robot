package org.apache.commons.net.io;

import java.io.FilterInputStream;
import java.io.IOException;
import java.io.InputStream;

public final class ToNetASCIIInputStream extends FilterInputStream {
    private static final int __LAST_WAS_CR = 1;
    private static final int __LAST_WAS_NL = 2;
    private static final int __NOTHING_SPECIAL = 0;
    private int __status = 0;

    public ToNetASCIIInputStream(InputStream input) {
        super(input);
    }

    public int read() throws IOException {
        if (this.__status == 2) {
            this.__status = 0;
            return 10;
        }
        int ch = this.in.read();
        if (ch != 10) {
            if (ch == 13) {
                this.__status = 1;
                return 13;
            }
        } else if (this.__status != 1) {
            this.__status = 2;
            return 13;
        }
        this.__status = 0;
        return ch;
    }

    public int read(byte[] buffer) throws IOException {
        return read(buffer, 0, buffer.length);
    }

    public int read(byte[] buffer, int offset, int length) throws IOException {
        int offset2;
        if (length < 1) {
            return 0;
        }
        int ch = available();
        if (length > ch) {
            length = ch;
        }
        if (length < 1) {
            length = 1;
        }
        int read = read();
        int ch2 = read;
        if (read == -1) {
            return -1;
        }
        int length2 = length;
        int length3 = offset;
        while (true) {
            offset2 = length3 + 1;
            buffer[length3] = (byte) ch2;
            length2--;
            if (length2 <= 0) {
                break;
            }
            int read2 = read();
            ch2 = read2;
            if (read2 == -1) {
                break;
            }
            length3 = offset2;
        }
        return offset2 - offset;
    }

    public boolean markSupported() {
        return false;
    }

    public int available() throws IOException {
        int result = this.in.available();
        if (this.__status == 2) {
            return result + 1;
        }
        return result;
    }
}
