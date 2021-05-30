package org.apache.commons.net.io;

import java.io.IOException;
import java.io.InputStream;
import java.io.PushbackInputStream;

public final class FromNetASCIIInputStream extends PushbackInputStream {
    static final String _lineSeparator = System.getProperty("line.separator");
    static final byte[] _lineSeparatorBytes = _lineSeparator.getBytes();
    static final boolean _noConversionRequired = _lineSeparator.equals("\r\n");
    private int __length = 0;

    public static final boolean isConversionRequired() {
        return !_noConversionRequired;
    }

    public FromNetASCIIInputStream(InputStream input) {
        super(input, _lineSeparatorBytes.length + 1);
    }

    private int __read() throws IOException {
        int ch = super.read();
        if (ch != 13) {
            return ch;
        }
        int ch2 = super.read();
        if (ch2 == 10) {
            unread(_lineSeparatorBytes);
            this.__length--;
            return super.read();
        }
        if (ch2 != -1) {
            unread(ch2);
        }
        return 13;
    }

    public int read() throws IOException {
        if (_noConversionRequired) {
            return super.read();
        }
        return __read();
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
        this.__length = length > ch ? ch : length;
        if (this.__length < 1) {
            this.__length = 1;
        }
        if (_noConversionRequired) {
            return super.read(buffer, offset, this.__length);
        }
        int __read = __read();
        int ch2 = __read;
        if (__read == -1) {
            return -1;
        }
        int ch3 = ch2;
        int ch4 = offset;
        while (true) {
            offset2 = ch4 + 1;
            buffer[ch4] = (byte) ch3;
            int i = this.__length - 1;
            this.__length = i;
            if (i <= 0) {
                break;
            }
            int __read2 = __read();
            ch3 = __read2;
            if (__read2 == -1) {
                break;
            }
            ch4 = offset2;
        }
        return offset2 - offset;
    }

    public int available() throws IOException {
        return (this.buf.length - this.pos) + this.in.available();
    }
}
