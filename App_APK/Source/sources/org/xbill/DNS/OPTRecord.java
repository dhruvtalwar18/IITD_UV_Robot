package org.xbill.DNS;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.xbill.DNS.utils.base16;

public class OPTRecord extends Record {
    private static final long serialVersionUID = -6254521894809367938L;
    private List options;

    public static class Option {
        public final int code;
        public final byte[] data;

        public Option(int code2, byte[] data2) {
            this.code = Record.checkU8("option code", code2);
            this.data = data2;
        }

        public String toString() {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("{");
            stringBuffer.append(this.code);
            stringBuffer.append(" <");
            stringBuffer.append(base16.toString(this.data));
            stringBuffer.append(">}");
            return stringBuffer.toString();
        }
    }

    OPTRecord() {
    }

    /* access modifiers changed from: package-private */
    public Record getObject() {
        return new OPTRecord();
    }

    public OPTRecord(int payloadSize, int xrcode, int version, int flags, List options2) {
        super(Name.root, 41, payloadSize, 0);
        checkU16("payloadSize", payloadSize);
        checkU8("xrcode", xrcode);
        checkU8("version", version);
        checkU16("flags", flags);
        this.ttl = (((long) xrcode) << 24) + (((long) version) << 16) + ((long) flags);
        if (options2 != null) {
            this.options = new ArrayList(options2);
        }
    }

    public OPTRecord(int payloadSize, int xrcode, int version, int flags) {
        this(payloadSize, xrcode, version, flags, (List) null);
    }

    public OPTRecord(int payloadSize, int xrcode, int version) {
        this(payloadSize, xrcode, version, 0, (List) null);
    }

    /* access modifiers changed from: package-private */
    public void rrFromWire(DNSInput in) throws IOException {
        if (in.remaining() > 0) {
            this.options = new ArrayList();
        }
        while (in.remaining() > 0) {
            this.options.add(new Option(in.readU16(), in.readByteArray(in.readU16())));
        }
    }

    /* access modifiers changed from: package-private */
    public void rdataFromString(Tokenizer st, Name origin) throws IOException {
        throw st.exception("no text format defined for OPT");
    }

    /* access modifiers changed from: package-private */
    public String rrToString() {
        StringBuffer sb = new StringBuffer();
        if (this.options != null) {
            sb.append(this.options);
            sb.append(" ");
        }
        sb.append(" ; payload ");
        sb.append(getPayloadSize());
        sb.append(", xrcode ");
        sb.append(getExtendedRcode());
        sb.append(", version ");
        sb.append(getVersion());
        sb.append(", flags ");
        sb.append(getFlags());
        return sb.toString();
    }

    public int getPayloadSize() {
        return this.dclass;
    }

    public int getExtendedRcode() {
        return (int) (this.ttl >>> 24);
    }

    public int getVersion() {
        return (int) ((this.ttl >>> 16) & 255);
    }

    public int getFlags() {
        return (int) (this.ttl & 65535);
    }

    /* access modifiers changed from: package-private */
    public void rrToWire(DNSOutput out, Compression c, boolean canonical) {
        if (this.options != null) {
            for (Option opt : this.options) {
                out.writeU16(opt.code);
                out.writeU16(opt.data.length);
                out.writeByteArray(opt.data);
            }
        }
    }

    public List getOptions() {
        if (this.options == null) {
            return Collections.EMPTY_LIST;
        }
        return Collections.unmodifiableList(this.options);
    }

    public List getOptions(int code) {
        if (this.options == null) {
            return Collections.EMPTY_LIST;
        }
        List list = null;
        for (Option opt : this.options) {
            if (opt.code == code) {
                if (list == null) {
                    list = new ArrayList();
                }
                list.add(opt.data);
            }
        }
        if (list == null) {
            return Collections.EMPTY_LIST;
        }
        return list;
    }
}
