package org.xbill.DNS;

import java.io.IOException;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.Arrays;
import junit.framework.TestCase;

public class A6RecordTest extends TestCase {
    InetAddress m_addr;
    byte[] m_addr_bytes;
    String m_addr_string;
    String m_addr_string_canonical;
    Name m_an;
    Name m_an2;
    int m_prefix_bits;
    Name m_rn;
    long m_ttl;

    /* access modifiers changed from: protected */
    public void setUp() throws TextParseException, UnknownHostException {
        this.m_an = Name.fromString("My.Absolute.Name.");
        this.m_an2 = Name.fromString("My.Second.Absolute.Name.");
        this.m_rn = Name.fromString("My.Relative.Name");
        this.m_addr_string = "2001:0db8:85a3:08d3:1319:8a2e:0370:7334";
        this.m_addr_string_canonical = "2001:db8:85a3:8d3:1319:8a2e:370:7334";
        this.m_addr = InetAddress.getByName(this.m_addr_string);
        this.m_addr_bytes = this.m_addr.getAddress();
        this.m_ttl = 79225;
        this.m_prefix_bits = 9;
    }

    public void test_ctor_0arg() {
        A6Record ar = new A6Record();
        assertNull(ar.getName());
        assertEquals(0, ar.getType());
        assertEquals(0, ar.getDClass());
        assertEquals(0, ar.getTTL());
    }

    public void test_getObject() {
        assertTrue(new A6Record().getObject() instanceof A6Record);
    }

    public void test_ctor_6arg() {
        A6Record ar = new A6Record(this.m_an, 1, this.m_ttl, this.m_prefix_bits, this.m_addr, (Name) null);
        assertEquals(this.m_an, ar.getName());
        assertEquals(38, ar.getType());
        assertEquals(1, ar.getDClass());
        assertEquals(this.m_ttl, ar.getTTL());
        assertEquals(this.m_prefix_bits, ar.getPrefixBits());
        assertEquals(this.m_addr, ar.getSuffix());
        assertNull(ar.getPrefix());
        A6Record ar2 = new A6Record(this.m_an, 1, this.m_ttl, this.m_prefix_bits, this.m_addr, this.m_an2);
        assertEquals(this.m_an, ar2.getName());
        assertEquals(38, ar2.getType());
        assertEquals(1, ar2.getDClass());
        assertEquals(this.m_ttl, ar2.getTTL());
        assertEquals(this.m_prefix_bits, ar2.getPrefixBits());
        assertEquals(this.m_addr, ar2.getSuffix());
        assertEquals(this.m_an2, ar2.getPrefix());
        try {
            new A6Record(this.m_rn, 1, this.m_ttl, this.m_prefix_bits, this.m_addr, (Name) null);
            fail("RelativeNameException not thrown");
        } catch (RelativeNameException e) {
        }
        try {
            new A6Record(this.m_an, 1, this.m_ttl, this.m_prefix_bits, this.m_addr, this.m_rn);
            fail("RelativeNameException not thrown");
        } catch (RelativeNameException e2) {
        }
        try {
            new A6Record(this.m_rn, 1, this.m_ttl, 256, this.m_addr, (Name) null);
            fail("IllegalArgumentException not thrown");
        } catch (RelativeNameException e3) {
        }
        try {
            new A6Record(this.m_an, 1, this.m_ttl, this.m_prefix_bits, InetAddress.getByName("192.168.0.1"), (Name) null);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e4) {
        } catch (UnknownHostException e5) {
            fail(e5.getMessage());
        }
    }

    public void test_rrFromWire() throws CloneNotSupportedException, IOException, UnknownHostException {
        DNSOutput dout = new DNSOutput();
        dout.writeU8(0);
        dout.writeByteArray(this.m_addr_bytes);
        DNSInput din = new DNSInput(dout.toByteArray());
        A6Record ar = new A6Record();
        ar.rrFromWire(din);
        assertEquals(0, ar.getPrefixBits());
        assertEquals(this.m_addr, ar.getSuffix());
        assertNull(ar.getPrefix());
        DNSOutput dout2 = new DNSOutput();
        dout2.writeU8(9);
        dout2.writeByteArray(this.m_addr_bytes, 1, 15);
        dout2.writeByteArray(this.m_an2.toWire());
        DNSInput din2 = new DNSInput(dout2.toByteArray());
        A6Record ar2 = new A6Record();
        ar2.rrFromWire(din2);
        assertEquals(9, ar2.getPrefixBits());
        byte[] addr_bytes = (byte[]) this.m_addr_bytes.clone();
        addr_bytes[0] = 0;
        assertEquals(InetAddress.getByAddress(addr_bytes), ar2.getSuffix());
        assertEquals(this.m_an2, ar2.getPrefix());
    }

    public void test_rdataFromString() throws CloneNotSupportedException, IOException, UnknownHostException {
        StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append("0 ");
        stringBuffer.append(this.m_addr_string);
        Tokenizer t = new Tokenizer(stringBuffer.toString());
        A6Record ar = new A6Record();
        ar.rdataFromString(t, (Name) null);
        assertEquals(0, ar.getPrefixBits());
        assertEquals(this.m_addr, ar.getSuffix());
        assertNull(ar.getPrefix());
        StringBuffer stringBuffer2 = new StringBuffer();
        stringBuffer2.append("9 ");
        stringBuffer2.append(this.m_addr_string);
        stringBuffer2.append(" ");
        stringBuffer2.append(this.m_an2);
        Tokenizer t2 = new Tokenizer(stringBuffer2.toString());
        A6Record ar2 = new A6Record();
        ar2.rdataFromString(t2, (Name) null);
        assertEquals(9, ar2.getPrefixBits());
        assertEquals(this.m_addr, ar2.getSuffix());
        assertEquals(this.m_an2, ar2.getPrefix());
        try {
            new A6Record().rdataFromString(new Tokenizer("129"), (Name) null);
            fail("TextParseException not thrown");
        } catch (TextParseException e) {
        }
        StringBuffer stringBuffer3 = new StringBuffer();
        stringBuffer3.append("0 ");
        stringBuffer3.append(this.m_addr_string.substring(4));
        try {
            new A6Record().rdataFromString(new Tokenizer(stringBuffer3.toString()), (Name) null);
            fail("TextParseException not thrown");
        } catch (TextParseException e2) {
        }
    }

    public void test_rrToString() {
        A6Record ar = new A6Record(this.m_an, 1, this.m_ttl, this.m_prefix_bits, this.m_addr, this.m_an2);
        StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append("");
        stringBuffer.append(this.m_prefix_bits);
        stringBuffer.append(" ");
        stringBuffer.append(this.m_addr_string_canonical);
        stringBuffer.append(" ");
        stringBuffer.append(this.m_an2);
        assertEquals(stringBuffer.toString(), ar.rrToString());
    }

    public void test_rrToWire() {
        A6Record ar = new A6Record(this.m_an, 1, this.m_ttl, this.m_prefix_bits, this.m_addr, this.m_an2);
        DNSOutput dout = new DNSOutput();
        dout.writeU8(this.m_prefix_bits);
        dout.writeByteArray(this.m_addr_bytes, 1, 15);
        dout.writeByteArray(this.m_an2.toWireCanonical());
        byte[] exp = dout.toByteArray();
        DNSOutput dout2 = new DNSOutput();
        ar.rrToWire(dout2, (Compression) null, true);
        assertTrue(Arrays.equals(exp, dout2.toByteArray()));
        DNSOutput dout3 = new DNSOutput();
        dout3.writeU8(this.m_prefix_bits);
        dout3.writeByteArray(this.m_addr_bytes, 1, 15);
        dout3.writeByteArray(this.m_an2.toWire());
        byte[] exp2 = dout3.toByteArray();
        DNSOutput dout4 = new DNSOutput();
        ar.rrToWire(dout4, (Compression) null, false);
        assertTrue(Arrays.equals(exp2, dout4.toByteArray()));
    }
}
