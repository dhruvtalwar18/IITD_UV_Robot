package org.jboss.netty.handler.codec.http.websocketx;

import com.google.common.base.Ascii;
import rocon_app_manager_msgs.ErrorCodes;

final class UTF8Output {
    private static final byte[] STATES = {0, Ascii.FF, Ascii.CAN, ErrorCodes.CLIENT_CONNECTION_DISRUPTED, 60, 96, 84, Ascii.FF, Ascii.FF, Ascii.FF, 48, 72, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, 0, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, 0, Ascii.FF, 0, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.CAN, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.CAN, Ascii.FF, Ascii.CAN, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.CAN, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.CAN, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.CAN, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, ErrorCodes.CLIENT_CONNECTION_DISRUPTED, Ascii.FF, ErrorCodes.CLIENT_CONNECTION_DISRUPTED, Ascii.FF, Ascii.FF, Ascii.FF, ErrorCodes.CLIENT_CONNECTION_DISRUPTED, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, ErrorCodes.CLIENT_CONNECTION_DISRUPTED, Ascii.FF, ErrorCodes.CLIENT_CONNECTION_DISRUPTED, Ascii.FF, Ascii.FF, Ascii.FF, ErrorCodes.CLIENT_CONNECTION_DISRUPTED, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF, Ascii.FF};
    private static final byte[] TYPES = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 8, 8, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 10, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 3, 3, 11, 6, 6, 6, 5, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8};
    private static final int UTF8_ACCEPT = 0;
    private static final int UTF8_REJECT = 12;
    private int codep;
    private int state = 0;
    private final StringBuilder stringBuilder;

    UTF8Output(byte[] bytes) {
        this.stringBuilder = new StringBuilder(bytes.length);
        write(bytes);
    }

    public void write(byte[] bytes) {
        for (byte b : bytes) {
            write((int) b);
        }
    }

    public void write(int b) {
        byte type = TYPES[b & 255];
        this.codep = this.state != 0 ? (b & 63) | (this.codep << 6) : (255 >> type) & b;
        this.state = STATES[this.state + type];
        if (this.state == 0) {
            this.stringBuilder.append((char) this.codep);
        } else if (this.state == 12) {
            throw new UTF8Exception("bytes are not UTF-8");
        }
    }

    public String toString() {
        if (this.state == 0) {
            return this.stringBuilder.toString();
        }
        throw new UTF8Exception("bytes are not UTF-8");
    }
}
