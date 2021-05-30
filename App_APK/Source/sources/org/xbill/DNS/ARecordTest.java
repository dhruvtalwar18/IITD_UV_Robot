package org.xbill.DNS;

import java.io.IOException;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.Arrays;
import junit.framework.TestCase;

public class ARecordTest extends TestCase {
    InetAddress m_addr;
    byte[] m_addr_bytes;
    String m_addr_string;
    Name m_an;
    Name m_rn;
    long m_ttl;

    /* access modifiers changed from: protected */
    public void setUp() throws TextParseException, UnknownHostException {
        this.m_an = Name.fromString("My.Absolute.Name.");
        this.m_rn = Name.fromString("My.Relative.Name");
        this.m_addr_string = "193.160.232.5";
        this.m_addr = InetAddress.getByName(this.m_addr_string);
        this.m_addr_bytes = this.m_addr.getAddress();
        this.m_ttl = 79225;
    }

    public void test_ctor_0arg() throws UnknownHostException {
        ARecord ar = new ARecord();
        assertNull(ar.getName());
        assertEquals(0, ar.getType());
        assertEquals(0, ar.getDClass());
        assertEquals(0, ar.getTTL());
        assertEquals(InetAddress.getByName("0.0.0.0"), ar.getAddress());
    }

    public void test_getObject() {
        assertTrue(new ARecord().getObject() instanceof ARecord);
    }

    public void test_ctor_4arg() {
        ARecord ar = new ARecord(this.m_an, 1, this.m_ttl, this.m_addr);
        assertEquals(this.m_an, ar.getName());
        assertEquals(1, ar.getType());
        assertEquals(1, ar.getDClass());
        assertEquals(this.m_ttl, ar.getTTL());
        assertEquals(this.m_addr, ar.getAddress());
        try {
            new ARecord(this.m_rn, 1, this.m_ttl, this.m_addr);
            fail("RelativeNameException not thrown");
        } catch (RelativeNameException e) {
        }
        try {
            new ARecord(this.m_an, 1, this.m_ttl, InetAddress.getByName("2001:0db8:85a3:08d3:1319:8a2e:0370:7334"));
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e2) {
        } catch (UnknownHostException e3) {
            fail(e3.getMessage());
        }
    }

    public void test_rrFromWire() throws IOException {
        DNSInput di = new DNSInput(this.m_addr_bytes);
        ARecord ar = new ARecord();
        ar.rrFromWire(di);
        assertEquals(this.m_addr, ar.getAddress());
    }

    public void test_rdataFromString() throws IOException {
        Tokenizer t = new Tokenizer(this.m_addr_string);
        ARecord ar = new ARecord();
        ar.rdataFromString(t, (Name) null);
        assertEquals(this.m_addr, ar.getAddress());
        try {
            new ARecord().rdataFromString(new Tokenizer("193.160.232"), (Name) null);
            fail("TextParseException not thrown");
        } catch (TextParseException e) {
        }
    }

    public void test_rrToString() {
        assertEquals(this.m_addr_string, new ARecord(this.m_an, 1, this.m_ttl, this.m_addr).rrToString());
    }

    public void test_rrToWire() {
        ARecord ar = new ARecord(this.m_an, 1, this.m_ttl, this.m_addr);
        DNSOutput dout = new DNSOutput();
        ar.rrToWire(dout, (Compression) null, true);
        assertTrue(Arrays.equals(this.m_addr_bytes, dout.toByteArray()));
        DNSOutput dout2 = new DNSOutput();
        ar.rrToWire(dout2, (Compression) null, false);
        assertTrue(Arrays.equals(this.m_addr_bytes, dout2.toByteArray()));
    }
}
