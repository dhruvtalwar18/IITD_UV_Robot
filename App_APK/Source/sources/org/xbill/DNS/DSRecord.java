package org.xbill.DNS;

import java.io.IOException;
import org.xbill.DNS.utils.base16;

public class DSRecord extends Record {
    public static final int SHA1_DIGEST_ID = 1;
    public static final int SHA256_DIGEST_ID = 2;
    private static final long serialVersionUID = -9001819329700081493L;
    private int alg;
    private byte[] digest;
    private int digestid;
    private int footprint;

    public static class Digest {
        public static final int SHA1 = 1;
        public static final int SHA256 = 2;

        private Digest() {
        }
    }

    DSRecord() {
    }

    /* access modifiers changed from: package-private */
    public Record getObject() {
        return new DSRecord();
    }

    public DSRecord(Name name, int dclass, long ttl, int footprint2, int alg2, int digestid2, byte[] digest2) {
        super(name, 43, dclass, ttl);
        this.footprint = checkU16("footprint", footprint2);
        this.alg = checkU8("alg", alg2);
        this.digestid = checkU8("digestid", digestid2);
        this.digest = digest2;
    }

    /* JADX WARNING: Illegal instructions before constructor call */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public DSRecord(org.xbill.DNS.Name r12, int r13, long r14, int r16, int r17, org.xbill.DNS.DNSKEYRecord r18) {
        /*
            r11 = this;
            int r6 = r18.getAlgorithm()
            r9 = r17
            r10 = r18
            byte[] r8 = org.xbill.DNS.DNSSEC.generateDS(r10, r9)
            r0 = r11
            r1 = r12
            r2 = r13
            r3 = r14
            r5 = r16
            r7 = r17
            r0.<init>(r1, r2, r3, r5, r6, r7, r8)
            return
        */
        throw new UnsupportedOperationException("Method not decompiled: org.xbill.DNS.DSRecord.<init>(org.xbill.DNS.Name, int, long, int, int, org.xbill.DNS.DNSKEYRecord):void");
    }

    /* access modifiers changed from: package-private */
    public void rrFromWire(DNSInput in) throws IOException {
        this.footprint = in.readU16();
        this.alg = in.readU8();
        this.digestid = in.readU8();
        this.digest = in.readByteArray();
    }

    /* access modifiers changed from: package-private */
    public void rdataFromString(Tokenizer st, Name origin) throws IOException {
        this.footprint = st.getUInt16();
        this.alg = st.getUInt8();
        this.digestid = st.getUInt8();
        this.digest = st.getHex();
    }

    /* access modifiers changed from: package-private */
    public String rrToString() {
        StringBuffer sb = new StringBuffer();
        sb.append(this.footprint);
        sb.append(" ");
        sb.append(this.alg);
        sb.append(" ");
        sb.append(this.digestid);
        if (this.digest != null) {
            sb.append(" ");
            sb.append(base16.toString(this.digest));
        }
        return sb.toString();
    }

    public int getAlgorithm() {
        return this.alg;
    }

    public int getDigestID() {
        return this.digestid;
    }

    public byte[] getDigest() {
        return this.digest;
    }

    public int getFootprint() {
        return this.footprint;
    }

    /* access modifiers changed from: package-private */
    public void rrToWire(DNSOutput out, Compression c, boolean canonical) {
        out.writeU16(this.footprint);
        out.writeU8(this.alg);
        out.writeU8(this.digestid);
        if (this.digest != null) {
            out.writeByteArray(this.digest);
        }
    }
}
