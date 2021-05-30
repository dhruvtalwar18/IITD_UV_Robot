package org.xbill.DNS;

import java.io.IOException;
import java.io.PrintStream;
import java.io.Serializable;
import java.text.DecimalFormat;
import org.apache.commons.io.IOUtils;
import sensor_msgs.NavSatStatus;

public class Name implements Comparable, Serializable {
    private static final int LABEL_COMPRESSION = 192;
    private static final int LABEL_MASK = 192;
    private static final int LABEL_NORMAL = 0;
    private static final int MAXLABEL = 63;
    private static final int MAXLABELS = 128;
    private static final int MAXNAME = 255;
    private static final int MAXOFFSETS = 7;
    private static final DecimalFormat byteFormat = new DecimalFormat();
    public static final Name empty = new Name();
    private static final byte[] emptyLabel = {0};
    private static final byte[] lowercase = new byte[256];
    public static final Name root = new Name();
    private static final long serialVersionUID = -7257019940971525644L;
    private static final Name wild = new Name();
    private static final byte[] wildLabel = {1, 42};
    private int hashcode;
    private byte[] name;
    private long offsets;

    static {
        byteFormat.setMinimumIntegerDigits(3);
        for (int i = 0; i < lowercase.length; i++) {
            if (i < 65 || i > 90) {
                lowercase[i] = (byte) i;
            } else {
                lowercase[i] = (byte) ((i - 65) + 97);
            }
        }
        root.appendSafe(emptyLabel, 0, 1);
        empty.name = new byte[0];
        wild.appendSafe(wildLabel, 0, 1);
    }

    private Name() {
    }

    private final void setoffset(int n, int offset) {
        if (n < 7) {
            int shift = (7 - n) * 8;
            this.offsets &= (255 << shift) ^ -1;
            this.offsets |= ((long) offset) << shift;
        }
    }

    private final int offset(int n) {
        if (n == 0 && getlabels() == 0) {
            return 0;
        }
        if (n < 0 || n >= getlabels()) {
            throw new IllegalArgumentException("label out of range");
        } else if (n < 7) {
            return ((int) (this.offsets >>> ((7 - n) * 8))) & 255;
        } else {
            int pos = offset(6);
            for (int i = 6; i < n; i++) {
                pos += this.name[pos] + 1;
            }
            return pos;
        }
    }

    private final void setlabels(int labels) {
        this.offsets &= -256;
        this.offsets |= (long) labels;
    }

    private final int getlabels() {
        return (int) (this.offsets & 255);
    }

    private static final void copy(Name src, Name dst) {
        int i = 0;
        if (src.offset(0) == 0) {
            dst.name = src.name;
            dst.offsets = src.offsets;
            return;
        }
        int offset0 = src.offset(0);
        int namelen = src.name.length - offset0;
        int labels = src.labels();
        dst.name = new byte[namelen];
        System.arraycopy(src.name, offset0, dst.name, 0, namelen);
        while (i < labels && i < 7) {
            dst.setoffset(i, src.offset(i) - offset0);
            i++;
        }
        dst.setlabels(labels);
    }

    private final void append(byte[] array, int start, int n) throws NameTooLongException {
        int length = this.name == null ? 0 : this.name.length - offset(0);
        int i = 0;
        int alength = 0;
        int pos = start;
        while (i < n) {
            byte len = array[pos];
            if (len <= 63) {
                int len2 = len + 1;
                pos += len2;
                alength += len2;
                i++;
            } else {
                throw new IllegalStateException("invalid label");
            }
        }
        int newlength = length + alength;
        if (newlength <= 255) {
            int labels = getlabels();
            int newlabels = labels + n;
            if (newlabels <= 128) {
                byte[] newname = new byte[newlength];
                if (length != 0) {
                    System.arraycopy(this.name, offset(0), newname, 0, length);
                }
                System.arraycopy(array, start, newname, length, alength);
                this.name = newname;
                int pos2 = length;
                for (int i2 = 0; i2 < n; i2++) {
                    setoffset(labels + i2, pos2);
                    pos2 += newname[pos2] + 1;
                }
                setlabels(newlabels);
                return;
            }
            throw new IllegalStateException("too many labels");
        }
        throw new NameTooLongException();
    }

    private static TextParseException parseException(String str, String message) {
        StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append("'");
        stringBuffer.append(str);
        stringBuffer.append("': ");
        stringBuffer.append(message);
        return new TextParseException(stringBuffer.toString());
    }

    private final void appendFromString(String fullName, byte[] array, int start, int n) throws TextParseException {
        try {
            append(array, start, n);
        } catch (NameTooLongException e) {
            throw parseException(fullName, "Name too long");
        }
    }

    private final void appendSafe(byte[] array, int start, int n) {
        try {
            append(array, start, n);
        } catch (NameTooLongException e) {
        }
    }

    public Name(String s, Name origin) throws TextParseException {
        int pos;
        String str = s;
        Name name2 = origin;
        if (str.equals("")) {
            throw parseException(str, "empty name");
        } else if (str.equals("@")) {
            if (name2 == null) {
                copy(empty, this);
            } else {
                copy(name2, this);
            }
        } else if (str.equals(".")) {
            copy(root, this);
        } else {
            byte[] label = new byte[64];
            boolean escaped = false;
            int digits = 0;
            int intval = 0;
            boolean absolute = false;
            int pos2 = 1;
            int labelstart = -1;
            for (int i = 0; i < s.length(); i++) {
                byte b = (byte) str.charAt(i);
                if (escaped) {
                    if (b >= 48 && b <= 57 && digits < 3) {
                        digits++;
                        intval = (intval * 10) + (b - 48);
                        if (intval > 255) {
                            throw parseException(str, "bad escape");
                        } else if (digits < 3) {
                            continue;
                        } else {
                            b = (byte) intval;
                        }
                    } else if (digits > 0 && digits < 3) {
                        throw parseException(str, "bad escape");
                    }
                    if (pos2 <= 63) {
                        labelstart = pos2;
                        pos = pos2 + 1;
                        label[pos2] = b;
                        escaped = false;
                    } else {
                        throw parseException(str, "label too long");
                    }
                } else {
                    if (b == 92) {
                        escaped = true;
                        digits = 0;
                        intval = 0;
                    } else if (b != 46) {
                        labelstart = labelstart == -1 ? i : labelstart;
                        if (pos2 <= 63) {
                            pos = pos2 + 1;
                            label[pos2] = b;
                        } else {
                            throw parseException(str, "label too long");
                        }
                    } else if (labelstart != -1) {
                        label[0] = (byte) (pos2 - 1);
                        appendFromString(str, label, 0, 1);
                        labelstart = -1;
                        pos2 = 1;
                    } else {
                        throw parseException(str, "invalid empty label");
                    }
                }
                pos2 = pos;
            }
            if (digits > 0 && digits < 3) {
                throw parseException(str, "bad escape");
            } else if (!escaped) {
                if (labelstart == -1) {
                    appendFromString(str, emptyLabel, 0, 1);
                    absolute = true;
                } else {
                    label[0] = (byte) (pos2 - 1);
                    appendFromString(str, label, 0, 1);
                }
                if (name2 != null && !absolute) {
                    appendFromString(str, name2.name, 0, origin.getlabels());
                }
            } else {
                throw parseException(str, "bad escape");
            }
        }
    }

    public Name(String s) throws TextParseException {
        this(s, (Name) null);
    }

    public static Name fromString(String s, Name origin) throws TextParseException {
        if (s.equals("@") && origin != null) {
            return origin;
        }
        if (s.equals(".")) {
            return root;
        }
        return new Name(s, origin);
    }

    public static Name fromString(String s) throws TextParseException {
        return fromString(s, (Name) null);
    }

    public static Name fromConstantString(String s) {
        try {
            return fromString(s, (Name) null);
        } catch (TextParseException e) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Invalid name '");
            stringBuffer.append(s);
            stringBuffer.append("'");
            throw new IllegalArgumentException(stringBuffer.toString());
        }
    }

    public Name(DNSInput in) throws WireParseException {
        byte[] label = new byte[64];
        boolean done = false;
        boolean savedState = false;
        while (!done) {
            int len = in.readU8();
            int i = len & 192;
            if (i != 0) {
                if (i == 192) {
                    int pos = in.readU8() + ((len & -193) << 8);
                    if (Options.check("verbosecompression")) {
                        PrintStream printStream = System.err;
                        StringBuffer stringBuffer = new StringBuffer();
                        stringBuffer.append("currently ");
                        stringBuffer.append(in.current());
                        stringBuffer.append(", pointer to ");
                        stringBuffer.append(pos);
                        printStream.println(stringBuffer.toString());
                    }
                    if (pos < in.current() - 2) {
                        if (!savedState) {
                            in.save();
                            savedState = true;
                        }
                        in.jump(pos);
                        if (Options.check("verbosecompression")) {
                            PrintStream printStream2 = System.err;
                            StringBuffer stringBuffer2 = new StringBuffer();
                            stringBuffer2.append("current name '");
                            stringBuffer2.append(this);
                            stringBuffer2.append("', seeking to ");
                            stringBuffer2.append(pos);
                            printStream2.println(stringBuffer2.toString());
                        }
                    } else {
                        throw new WireParseException("bad compression");
                    }
                } else {
                    throw new WireParseException("bad label type");
                }
            } else if (getlabels() >= 128) {
                throw new WireParseException("too many labels");
            } else if (len == 0) {
                append(emptyLabel, 0, 1);
                done = true;
            } else {
                label[0] = (byte) len;
                in.readByteArray(label, 1, len);
                append(label, 0, 1);
            }
        }
        if (savedState) {
            in.restore();
        }
    }

    public Name(byte[] b) throws IOException {
        this(new DNSInput(b));
    }

    public Name(Name src, int n) {
        int slabels = src.labels();
        if (n <= slabels) {
            this.name = src.name;
            setlabels(slabels - n);
            int i = 0;
            while (i < 7 && i < slabels - n) {
                setoffset(i, src.offset(i + n));
                i++;
            }
            return;
        }
        throw new IllegalArgumentException("attempted to remove too many labels");
    }

    public static Name concatenate(Name prefix, Name suffix) throws NameTooLongException {
        if (prefix.isAbsolute()) {
            return prefix;
        }
        Name newname = new Name();
        copy(prefix, newname);
        newname.append(suffix.name, suffix.offset(0), suffix.getlabels());
        return newname;
    }

    public Name relativize(Name origin) {
        if (origin == null || !subdomain(origin)) {
            return this;
        }
        Name newname = new Name();
        copy(this, newname);
        int length = length() - origin.length();
        newname.setlabels(newname.labels() - origin.labels());
        newname.name = new byte[length];
        System.arraycopy(this.name, offset(0), newname.name, 0, length);
        return newname;
    }

    public Name wild(int n) {
        if (n >= 1) {
            try {
                Name newname = new Name();
                copy(wild, newname);
                newname.append(this.name, offset(n), getlabels() - n);
                return newname;
            } catch (NameTooLongException e) {
                throw new IllegalStateException("Name.wild: concatenate failed");
            }
        } else {
            throw new IllegalArgumentException("must replace 1 or more labels");
        }
    }

    public Name fromDNAME(DNAMERecord dname) throws NameTooLongException {
        Name dnameowner = dname.getName();
        Name dnametarget = dname.getTarget();
        if (!subdomain(dnameowner)) {
            return null;
        }
        int plabels = labels() - dnameowner.labels();
        int plength = length() - dnameowner.length();
        int pos = 0;
        int pstart = offset(0);
        int dlabels = dnametarget.labels();
        int dlength = dnametarget.length();
        if (plength + dlength <= 255) {
            Name newname = new Name();
            newname.setlabels(plabels + dlabels);
            newname.name = new byte[(plength + dlength)];
            System.arraycopy(this.name, pstart, newname.name, 0, plength);
            System.arraycopy(dnametarget.name, 0, newname.name, plength, dlength);
            int i = 0;
            while (i < 7 && i < plabels + dlabels) {
                newname.setoffset(i, pos);
                pos += newname.name[pos] + 1;
                i++;
            }
            return newname;
        }
        throw new NameTooLongException();
    }

    public boolean isWild() {
        if (labels() != 0 && this.name[0] == 1 && this.name[1] == 42) {
            return true;
        }
        return false;
    }

    public boolean isAbsolute() {
        if (labels() != 0 && this.name[this.name.length - 1] == 0) {
            return true;
        }
        return false;
    }

    public short length() {
        if (getlabels() == 0) {
            return 0;
        }
        return (short) (this.name.length - offset(0));
    }

    public int labels() {
        return getlabels();
    }

    public boolean subdomain(Name domain) {
        int labels = labels();
        int dlabels = domain.labels();
        if (dlabels > labels) {
            return false;
        }
        if (dlabels == labels) {
            return equals(domain);
        }
        return domain.equals(this.name, offset(labels - dlabels));
    }

    private String byteString(byte[] array, int pos) {
        StringBuffer sb = new StringBuffer();
        int pos2 = pos + 1;
        byte len = array[pos];
        for (int i = pos2; i < pos2 + len; i++) {
            int b = array[i] & 255;
            if (b <= 32 || b >= 127) {
                sb.append(IOUtils.DIR_SEPARATOR_WINDOWS);
                sb.append(byteFormat.format((long) b));
            } else if (b == 34 || b == 40 || b == 41 || b == 46 || b == 59 || b == 92 || b == 64 || b == 36) {
                sb.append(IOUtils.DIR_SEPARATOR_WINDOWS);
                sb.append((char) b);
            } else {
                sb.append((char) b);
            }
        }
        return sb.toString();
    }

    public String toString() {
        int labels = labels();
        if (labels == 0) {
            return "@";
        }
        if (labels == 1 && this.name[offset(0)] == 0) {
            return ".";
        }
        StringBuffer sb = new StringBuffer();
        int i = 0;
        int pos = offset(0);
        while (i < labels) {
            byte len = this.name[pos];
            if (len > 63) {
                throw new IllegalStateException("invalid label");
            } else if (len == 0) {
                break;
            } else {
                sb.append(byteString(this.name, pos));
                sb.append('.');
                pos += len + 1;
                i++;
            }
        }
        if (isAbsolute() == 0) {
            sb.deleteCharAt(sb.length() - 1);
        }
        return sb.toString();
    }

    public byte[] getLabel(int n) {
        int pos = offset(n);
        int len = (byte) (this.name[pos] + 1);
        byte[] label = new byte[len];
        System.arraycopy(this.name, pos, label, 0, len);
        return label;
    }

    public String getLabelString(int n) {
        return byteString(this.name, offset(n));
    }

    public void toWire(DNSOutput out, Compression c) {
        Name tname;
        if (isAbsolute()) {
            int labels = labels();
            for (int i = 0; i < labels - 1; i++) {
                if (i == 0) {
                    tname = this;
                } else {
                    tname = new Name(this, i);
                }
                int pos = -1;
                if (c != null) {
                    pos = c.get(tname);
                }
                if (pos >= 0) {
                    out.writeU16(49152 | pos);
                    return;
                }
                if (c != null) {
                    c.add(out.current(), tname);
                }
                int off = offset(i);
                out.writeByteArray(this.name, off, this.name[off] + 1);
            }
            out.writeU8(0);
            return;
        }
        throw new IllegalArgumentException("toWire() called on non-absolute name");
    }

    public byte[] toWire() {
        DNSOutput out = new DNSOutput();
        toWire(out, (Compression) null);
        return out.toByteArray();
    }

    public void toWireCanonical(DNSOutput out) {
        out.writeByteArray(toWireCanonical());
    }

    public byte[] toWireCanonical() {
        int labels = labels();
        if (labels == 0) {
            return new byte[0];
        }
        byte[] b = new byte[(this.name.length - offset(0))];
        int spos = offset(0);
        int i = 0;
        int i2 = 0;
        while (i < labels) {
            byte len = this.name[spos];
            if (len <= 63) {
                int dpos = i2 + 1;
                b[i2] = this.name[spos];
                spos++;
                int j = 0;
                while (j < len) {
                    b[dpos] = lowercase[this.name[spos] & 255];
                    j++;
                    dpos++;
                    spos++;
                }
                i++;
                i2 = dpos;
            } else {
                throw new IllegalStateException("invalid label");
            }
        }
        return b;
    }

    public void toWire(DNSOutput out, Compression c, boolean canonical) {
        if (canonical) {
            toWireCanonical(out);
        } else {
            toWire(out, c);
        }
    }

    private final boolean equals(byte[] b, int bpos) {
        int labels = labels();
        int i = 0;
        int pos = offset(0);
        while (i < labels) {
            if (this.name[pos] != b[bpos]) {
                return false;
            }
            int pos2 = pos + 1;
            byte len = this.name[pos];
            int bpos2 = bpos + 1;
            if (len <= 63) {
                int bpos3 = bpos2;
                int j = 0;
                while (j < len) {
                    int pos3 = pos2 + 1;
                    int bpos4 = bpos3 + 1;
                    if (lowercase[this.name[pos2] & 255] != lowercase[b[bpos3] & 255]) {
                        return false;
                    }
                    j++;
                    bpos3 = bpos4;
                    pos2 = pos3;
                }
                i++;
                bpos = bpos3;
                pos = pos2;
            } else {
                throw new IllegalStateException("invalid label");
            }
        }
        return true;
    }

    public boolean equals(Object arg) {
        if (arg == this) {
            return true;
        }
        if (arg == null || !(arg instanceof Name)) {
            return false;
        }
        Name d = (Name) arg;
        if (d.hashcode == 0) {
            d.hashCode();
        }
        if (this.hashcode == 0) {
            hashCode();
        }
        if (d.hashcode == this.hashcode && d.labels() == labels()) {
            return equals(d.name, d.offset(0));
        }
        return false;
    }

    public int hashCode() {
        if (this.hashcode != 0) {
            return this.hashcode;
        }
        int code = 0;
        for (int i = offset(0); i < this.name.length; i++) {
            code += (code << 3) + lowercase[this.name[i] & NavSatStatus.STATUS_NO_FIX];
        }
        this.hashcode = code;
        return this.hashcode;
    }

    /* JADX WARNING: type inference failed for: r17v0, types: [java.lang.Object] */
    /* JADX WARNING: Unknown variable types count: 1 */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public int compareTo(java.lang.Object r17) {
        /*
            r16 = this;
            r0 = r16
            r1 = r17
            org.xbill.DNS.Name r1 = (org.xbill.DNS.Name) r1
            r2 = 0
            if (r0 != r1) goto L_0x000a
            return r2
        L_0x000a:
            int r3 = r16.labels()
            int r4 = r1.labels()
            if (r3 <= r4) goto L_0x0016
            r5 = r4
            goto L_0x0017
        L_0x0016:
            r5 = r3
        L_0x0017:
            r6 = 1
            r7 = 1
        L_0x0019:
            if (r7 > r5) goto L_0x005f
            int r8 = r3 - r7
            int r8 = r0.offset(r8)
            int r9 = r4 - r7
            int r9 = r1.offset(r9)
            byte[] r10 = r0.name
            byte r10 = r10[r8]
            byte[] r11 = r1.name
            byte r11 = r11[r9]
            r12 = 0
        L_0x0030:
            if (r12 >= r10) goto L_0x0056
            if (r12 >= r11) goto L_0x0056
            byte[] r13 = lowercase
            byte[] r14 = r0.name
            int r15 = r12 + r8
            int r15 = r15 + r6
            byte r14 = r14[r15]
            r14 = r14 & 255(0xff, float:3.57E-43)
            byte r13 = r13[r14]
            byte[] r14 = lowercase
            byte[] r2 = r1.name
            int r15 = r12 + r9
            int r15 = r15 + r6
            byte r2 = r2[r15]
            r2 = r2 & 255(0xff, float:3.57E-43)
            byte r2 = r14[r2]
            int r13 = r13 - r2
            if (r13 == 0) goto L_0x0052
            return r13
        L_0x0052:
            int r12 = r12 + 1
            r2 = 0
            goto L_0x0030
        L_0x0056:
            if (r10 == r11) goto L_0x005b
            int r2 = r10 - r11
            return r2
        L_0x005b:
            int r7 = r7 + 1
            r2 = 0
            goto L_0x0019
        L_0x005f:
            int r2 = r3 - r4
            return r2
        */
        throw new UnsupportedOperationException("Method not decompiled: org.xbill.DNS.Name.compareTo(java.lang.Object):int");
    }
}
