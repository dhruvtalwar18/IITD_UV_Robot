package org.xbill.DNS;

import sensor_msgs.NavSatStatus;

public class DNSInput {
    private byte[] array;
    private int end = this.array.length;
    private int pos = 0;
    private int saved_end = -1;
    private int saved_pos = -1;

    public DNSInput(byte[] input) {
        this.array = input;
    }

    public int current() {
        return this.pos;
    }

    public int remaining() {
        return this.end - this.pos;
    }

    private void require(int n) throws WireParseException {
        if (n > remaining()) {
            throw new WireParseException("end of input");
        }
    }

    public void setActive(int len) {
        if (len <= this.array.length - this.pos) {
            this.end = this.pos + len;
            return;
        }
        throw new IllegalArgumentException("cannot set active region past end of input");
    }

    public void clearActive() {
        this.end = this.array.length;
    }

    public void jump(int index) {
        if (index < this.array.length) {
            this.pos = index;
            this.end = this.array.length;
            return;
        }
        throw new IllegalArgumentException("cannot jump past end of input");
    }

    public void save() {
        this.saved_pos = this.pos;
        this.saved_end = this.end;
    }

    public void restore() {
        if (this.saved_pos >= 0) {
            this.pos = this.saved_pos;
            this.end = this.saved_end;
            this.saved_pos = -1;
            this.saved_end = -1;
            return;
        }
        throw new IllegalStateException("no previous state");
    }

    public int readU8() throws WireParseException {
        require(1);
        byte[] bArr = this.array;
        int i = this.pos;
        this.pos = i + 1;
        return bArr[i] & NavSatStatus.STATUS_NO_FIX;
    }

    public int readU16() throws WireParseException {
        require(2);
        byte[] bArr = this.array;
        int i = this.pos;
        this.pos = i + 1;
        byte[] bArr2 = this.array;
        int i2 = this.pos;
        this.pos = i2 + 1;
        return ((bArr[i] & 255) << 8) + (bArr2[i2] & 255);
    }

    public long readU32() throws WireParseException {
        require(4);
        byte[] bArr = this.array;
        int i = this.pos;
        this.pos = i + 1;
        byte[] bArr2 = this.array;
        int i2 = this.pos;
        this.pos = i2 + 1;
        byte[] bArr3 = this.array;
        int i3 = this.pos;
        this.pos = i3 + 1;
        byte[] bArr4 = this.array;
        int i4 = this.pos;
        this.pos = i4 + 1;
        return (((long) (bArr[i] & 255)) << 24) + ((long) ((bArr2[i2] & 255) << 16)) + ((long) ((bArr3[i3] & 255) << 8)) + ((long) (bArr4[i4] & 255));
    }

    public void readByteArray(byte[] b, int off, int len) throws WireParseException {
        require(len);
        System.arraycopy(this.array, this.pos, b, off, len);
        this.pos += len;
    }

    public byte[] readByteArray(int len) throws WireParseException {
        require(len);
        byte[] out = new byte[len];
        System.arraycopy(this.array, this.pos, out, 0, len);
        this.pos += len;
        return out;
    }

    public byte[] readByteArray() {
        int len = remaining();
        byte[] out = new byte[len];
        System.arraycopy(this.array, this.pos, out, 0, len);
        this.pos += len;
        return out;
    }

    public byte[] readCountedString() throws WireParseException {
        require(1);
        byte[] bArr = this.array;
        int i = this.pos;
        this.pos = i + 1;
        return readByteArray(bArr[i] & 255);
    }
}
