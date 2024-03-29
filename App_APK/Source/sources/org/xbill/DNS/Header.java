package org.xbill.DNS;

import java.io.IOException;
import java.util.Random;

public class Header implements Cloneable {
    public static final int LENGTH = 12;
    private static Random random = new Random();
    private int[] counts;
    private int flags;
    private int id;

    private void init() {
        this.counts = new int[4];
        this.flags = 0;
        this.id = -1;
    }

    public Header(int id2) {
        init();
        setID(id2);
    }

    public Header() {
        init();
    }

    Header(DNSInput in) throws IOException {
        this(in.readU16());
        this.flags = in.readU16();
        for (int i = 0; i < this.counts.length; i++) {
            this.counts[i] = in.readU16();
        }
    }

    public Header(byte[] b) throws IOException {
        this(new DNSInput(b));
    }

    /* access modifiers changed from: package-private */
    public void toWire(DNSOutput out) {
        out.writeU16(getID());
        out.writeU16(this.flags);
        for (int writeU16 : this.counts) {
            out.writeU16(writeU16);
        }
    }

    public byte[] toWire() {
        DNSOutput out = new DNSOutput();
        toWire(out);
        return out.toByteArray();
    }

    private static boolean validFlag(int bit) {
        return bit >= 0 && bit <= 15 && Flags.isFlag(bit);
    }

    private static void checkFlag(int bit) {
        if (!validFlag(bit)) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("invalid flag bit ");
            stringBuffer.append(bit);
            throw new IllegalArgumentException(stringBuffer.toString());
        }
    }

    public void setFlag(int bit) {
        checkFlag(bit);
        this.flags |= 1 << (15 - bit);
    }

    public void unsetFlag(int bit) {
        checkFlag(bit);
        this.flags &= (1 << (15 - bit)) ^ -1;
    }

    public boolean getFlag(int bit) {
        checkFlag(bit);
        return (this.flags & (1 << (15 - bit))) != 0;
    }

    /* access modifiers changed from: package-private */
    public boolean[] getFlags() {
        boolean[] array = new boolean[16];
        for (int i = 0; i < array.length; i++) {
            if (validFlag(i)) {
                array[i] = getFlag(i);
            }
        }
        return array;
    }

    public int getID() {
        int i;
        if (this.id >= 0) {
            return this.id;
        }
        synchronized (this) {
            if (this.id < 0) {
                this.id = random.nextInt(65535);
            }
            i = this.id;
        }
        return i;
    }

    public void setID(int id2) {
        if (id2 < 0 || id2 > 65535) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("DNS message ID ");
            stringBuffer.append(id2);
            stringBuffer.append(" is out of range");
            throw new IllegalArgumentException(stringBuffer.toString());
        }
        this.id = id2;
    }

    public void setRcode(int value) {
        if (value < 0 || value > 15) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("DNS Rcode ");
            stringBuffer.append(value);
            stringBuffer.append(" is out of range");
            throw new IllegalArgumentException(stringBuffer.toString());
        }
        this.flags &= -16;
        this.flags |= value;
    }

    public int getRcode() {
        return this.flags & 15;
    }

    public void setOpcode(int value) {
        if (value < 0 || value > 15) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("DNS Opcode ");
            stringBuffer.append(value);
            stringBuffer.append("is out of range");
            throw new IllegalArgumentException(stringBuffer.toString());
        }
        this.flags &= 34815;
        this.flags |= value << 11;
    }

    public int getOpcode() {
        return (this.flags >> 11) & 15;
    }

    /* access modifiers changed from: package-private */
    public void setCount(int field, int value) {
        if (value < 0 || value > 65535) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("DNS section count ");
            stringBuffer.append(value);
            stringBuffer.append(" is out of range");
            throw new IllegalArgumentException(stringBuffer.toString());
        }
        this.counts[field] = value;
    }

    /* access modifiers changed from: package-private */
    public void incCount(int field) {
        if (this.counts[field] != 65535) {
            int[] iArr = this.counts;
            iArr[field] = iArr[field] + 1;
            return;
        }
        throw new IllegalStateException("DNS section count cannot be incremented");
    }

    /* access modifiers changed from: package-private */
    public void decCount(int field) {
        if (this.counts[field] != 0) {
            int[] iArr = this.counts;
            iArr[field] = iArr[field] - 1;
            return;
        }
        throw new IllegalStateException("DNS section count cannot be decremented");
    }

    public int getCount(int field) {
        return this.counts[field];
    }

    public String printFlags() {
        StringBuffer sb = new StringBuffer();
        for (int i = 0; i < 16; i++) {
            if (validFlag(i) && getFlag(i)) {
                sb.append(Flags.string(i));
                sb.append(" ");
            }
        }
        return sb.toString();
    }

    /* access modifiers changed from: package-private */
    public String toStringWithRcode(int newrcode) {
        StringBuffer sb = new StringBuffer();
        sb.append(";; ->>HEADER<<- ");
        StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append("opcode: ");
        stringBuffer.append(Opcode.string(getOpcode()));
        sb.append(stringBuffer.toString());
        StringBuffer stringBuffer2 = new StringBuffer();
        stringBuffer2.append(", status: ");
        stringBuffer2.append(Rcode.string(newrcode));
        sb.append(stringBuffer2.toString());
        StringBuffer stringBuffer3 = new StringBuffer();
        stringBuffer3.append(", id: ");
        stringBuffer3.append(getID());
        sb.append(stringBuffer3.toString());
        sb.append("\n");
        StringBuffer stringBuffer4 = new StringBuffer();
        stringBuffer4.append(";; flags: ");
        stringBuffer4.append(printFlags());
        sb.append(stringBuffer4.toString());
        sb.append("; ");
        for (int i = 0; i < 4; i++) {
            StringBuffer stringBuffer5 = new StringBuffer();
            stringBuffer5.append(Section.string(i));
            stringBuffer5.append(": ");
            stringBuffer5.append(getCount(i));
            stringBuffer5.append(" ");
            sb.append(stringBuffer5.toString());
        }
        return sb.toString();
    }

    public String toString() {
        return toStringWithRcode(getRcode());
    }

    public Object clone() {
        Header h = new Header();
        h.id = this.id;
        h.flags = this.flags;
        System.arraycopy(this.counts, 0, h.counts, 0, this.counts.length);
        return h;
    }
}
