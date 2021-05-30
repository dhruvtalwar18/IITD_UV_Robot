package org.xbill.DNS;

import java.io.IOException;
import java.net.InetAddress;

public class AAAARecord extends Record {
    private static final long serialVersionUID = -4588601512069748050L;
    private InetAddress address;

    AAAARecord() {
    }

    /* access modifiers changed from: package-private */
    public Record getObject() {
        return new AAAARecord();
    }

    public AAAARecord(Name name, int dclass, long ttl, InetAddress address2) {
        super(name, 28, dclass, ttl);
        if (Address.familyOf(address2) == 2) {
            this.address = address2;
            return;
        }
        throw new IllegalArgumentException("invalid IPv6 address");
    }

    /* access modifiers changed from: package-private */
    public void rrFromWire(DNSInput in) throws IOException {
        this.address = InetAddress.getByAddress(in.readByteArray(16));
    }

    /* access modifiers changed from: package-private */
    public void rdataFromString(Tokenizer st, Name origin) throws IOException {
        this.address = st.getAddress(2);
    }

    /* access modifiers changed from: package-private */
    public String rrToString() {
        return this.address.getHostAddress();
    }

    public InetAddress getAddress() {
        return this.address;
    }

    /* access modifiers changed from: package-private */
    public void rrToWire(DNSOutput out, Compression c, boolean canonical) {
        out.writeByteArray(this.address.getAddress());
    }
}
