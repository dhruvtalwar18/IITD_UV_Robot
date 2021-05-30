package org.apache.commons.net.io;

import java.io.IOException;
import java.io.Writer;

public final class DotTerminatedMessageWriter extends Writer {
    private static final int __LAST_WAS_CR_STATE = 1;
    private static final int __LAST_WAS_NL_STATE = 2;
    private static final int __NOTHING_SPECIAL_STATE = 0;
    private Writer __output;
    private int __state = 0;

    public DotTerminatedMessageWriter(Writer output) {
        super(output);
        this.__output = output;
    }

    public void write(int ch) throws IOException {
        synchronized (this.lock) {
            if (ch == 10) {
                if (this.__state != 1) {
                    this.__output.write(13);
                }
                this.__output.write(10);
                this.__state = 2;
            } else if (ch != 13) {
                if (ch == 46) {
                    try {
                        if (this.__state == 2) {
                            this.__output.write(46);
                        }
                    } catch (Throwable th) {
                        throw th;
                    }
                }
                this.__state = 0;
                this.__output.write(ch);
            } else {
                this.__state = 1;
                this.__output.write(13);
            }
        }
    }

    public void write(char[] buffer, int offset, int offset2) throws IOException {
        Throwable th;
        synchronized (this.lock) {
            while (true) {
                int length = offset2 - 1;
                if (offset2 > 0) {
                    int offset3 = offset + 1;
                    try {
                        write((int) buffer[offset]);
                        offset = offset3;
                        offset2 = length;
                    } catch (Throwable th2) {
                        th = th2;
                        throw th;
                    }
                } else {
                    try {
                        return;
                    } catch (Throwable th3) {
                        Throwable th4 = th3;
                        int i = offset;
                        th = th4;
                        throw th;
                    }
                }
            }
        }
    }

    public void write(char[] buffer) throws IOException {
        write(buffer, 0, buffer.length);
    }

    public void write(String string) throws IOException {
        write(string.toCharArray());
    }

    public void write(String string, int offset, int length) throws IOException {
        write(string.toCharArray(), offset, length);
    }

    public void flush() throws IOException {
        synchronized (this.lock) {
            this.__output.flush();
        }
    }

    public void close() throws IOException {
        synchronized (this.lock) {
            if (this.__output != null) {
                if (this.__state == 1) {
                    this.__output.write(10);
                } else if (this.__state != 2) {
                    this.__output.write("\r\n");
                }
                this.__output.write(".\r\n");
                this.__output.flush();
                this.__output = null;
            }
        }
    }
}
