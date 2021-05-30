package org.xbill.DNS;

import com.google.common.base.Ascii;
import com.google.common.primitives.UnsignedBytes;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.Arrays;
import junit.framework.TestCase;

public class AddressTest extends TestCase {
    private void assertEquals(byte[] exp, byte[] act) {
        assertTrue(Arrays.equals(exp, act));
    }

    private void assertEquals(int[] exp, int[] act) {
        assertEquals(exp.length, act.length);
        for (int i = 0; i < exp.length; i++) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("i=");
            stringBuffer.append(i);
            assertEquals(stringBuffer.toString(), exp[i], act[i]);
        }
    }

    public void test_toByteArray_invalid() {
        try {
            Address.toByteArray("doesn't matter", 3);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e) {
        }
    }

    public void test_toByteArray_IPv4() {
        assertEquals(new byte[]{-58, 121, 10, -22}, Address.toByteArray("198.121.10.234", 1));
        assertEquals(new byte[]{0, 0, 0, 0}, Address.toByteArray("0.0.0.0", 1));
        byte[] bArr = {-1, -1, -1, -1};
        byte[] byteArray = Address.toByteArray("255.255.255.255", 1);
    }

    public void test_toByteArray_IPv4_invalid() {
        assertNull(Address.toByteArray("A.B.C.D", 1));
        assertNull(Address.toByteArray("128...", 1));
        assertNull(Address.toByteArray("128.121", 1));
        assertNull(Address.toByteArray("128.111.8", 1));
        assertNull(Address.toByteArray("128.198.10.", 1));
        assertNull(Address.toByteArray("128.121.90..10", 1));
        assertNull(Address.toByteArray("128.121..90.10", 1));
        assertNull(Address.toByteArray("128..121.90.10", 1));
        assertNull(Address.toByteArray(".128.121.90.10", 1));
        assertNull(Address.toByteArray("128.121.90.256", 1));
        assertNull(Address.toByteArray("128.121.256.10", 1));
        assertNull(Address.toByteArray("128.256.90.10", 1));
        assertNull(Address.toByteArray("256.121.90.10", 1));
        assertNull(Address.toByteArray("128.121.90.-1", 1));
        assertNull(Address.toByteArray("128.121.-1.10", 1));
        assertNull(Address.toByteArray("128.-1.90.10", 1));
        assertNull(Address.toByteArray("-1.121.90.10", 1));
        assertNull(Address.toByteArray("120.121.90.10.10", 1));
        assertNull(Address.toByteArray("120.121.90.010", 1));
        assertNull(Address.toByteArray("120.121.090.10", 1));
        assertNull(Address.toByteArray("120.021.90.10", 1));
        assertNull(Address.toByteArray("020.121.90.10", 1));
        assertNull(Address.toByteArray("1120.121.90.10", 1));
        assertNull(Address.toByteArray("120.2121.90.10", 1));
        assertNull(Address.toByteArray("120.121.4190.10", 1));
        assertNull(Address.toByteArray("120.121.190.1000", 1));
        assertNull(Address.toByteArray("", 1));
    }

    public void test_toByteArray_IPv6() {
        byte[] exp = {32, 1, 13, -72, -123, -93, 8, -45, 19, Ascii.EM, -118, 46, 3, 112, 115, 52};
        assertEquals(exp, Address.toByteArray("2001:0db8:85a3:08d3:1319:8a2e:0370:7334", 2));
        assertEquals(exp, Address.toByteArray("2001:db8:85a3:8d3:1319:8a2e:370:7334", 2));
        assertEquals(exp, Address.toByteArray("2001:DB8:85A3:8D3:1319:8A2E:370:7334", 2));
        assertEquals(new byte[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, Address.toByteArray("0:0:0:0:0:0:0:0", 2));
        assertEquals(new byte[]{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}, Address.toByteArray("FFFF:FFFF:FFFF:FFFF:FFFF:FFFF:FFFF:FFFF", 2));
        byte[] exp2 = {32, 1, 13, -72, 0, 0, 8, -45, 19, Ascii.EM, -118, 46, 3, 112, 115, 52};
        assertEquals(exp2, Address.toByteArray("2001:0db8:0000:08d3:1319:8a2e:0370:7334", 2));
        assertEquals(exp2, Address.toByteArray("2001:0db8::08d3:1319:8a2e:0370:7334", 2));
        byte[] exp3 = {0, 0, 0, 0, -123, -93, 8, -45, 19, Ascii.EM, -118, 46, 3, 112, 115, 52};
        assertEquals(exp3, Address.toByteArray("0000:0000:85a3:08d3:1319:8a2e:0370:7334", 2));
        assertEquals(exp3, Address.toByteArray("::85a3:08d3:1319:8a2e:0370:7334", 2));
        byte[] exp4 = {32, 1, 13, -72, -123, -93, 8, -45, 19, Ascii.EM, -118, 46, 0, 0, 0, 0};
        assertEquals(exp4, Address.toByteArray("2001:0db8:85a3:08d3:1319:8a2e:0:0", 2));
        assertEquals(exp4, Address.toByteArray("2001:0db8:85a3:08d3:1319:8a2e::", 2));
        byte[] exp5 = {32, 1, 13, -72, 0, 0, 0, 0, 0, 0, 0, 0, 3, 112, 115, 52};
        assertEquals(exp5, Address.toByteArray("2001:0db8:0000:0000:0000:0000:0370:7334", 2));
        assertEquals(exp5, Address.toByteArray("2001:0db8:0:0:0:0:0370:7334", 2));
        assertEquals(exp5, Address.toByteArray("2001:0db8::0:0370:7334", 2));
        assertEquals(exp5, Address.toByteArray("2001:db8::370:7334", 2));
        assertEquals(new byte[]{32, 1, 13, -72, -123, -93, 8, -45, 19, Ascii.EM, -118, 46, -64, -88, 89, 9}, Address.toByteArray("2001:0db8:85a3:08d3:1319:8a2e:192.168.89.9", 2));
        assertEquals(new byte[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -64, -88, 89, 9}, Address.toByteArray("::192.168.89.9", 2));
    }

    public void test_toByteArray_IPv6_invalid() {
        assertNull(Address.toByteArray("2001:0db8:85a3:08d3:1319:8a2e:0370", 2));
        assertNull(Address.toByteArray("2001:0db8:85a3:08d3:1319:8a2e:0370:193A:BCdE", 2));
        assertNull(Address.toByteArray("2001:0gb8:85a3:08d3:1319:8a2e:0370:9819", 2));
        assertNull(Address.toByteArray("lmno:0bb8:85a3:08d3:1319:8a2e:0370:9819", 2));
        assertNull(Address.toByteArray("11ab:0ab8:85a3:08d3:1319:8a2e:0370:qrst", 2));
        assertNull(Address.toByteArray("11ab:0ab8:85a3:08d3:::", 2));
        assertNull(Address.toByteArray("2001:0ab8:192.168.0.1:1319:8a2e:0370:9819", 2));
        assertNull(Address.toByteArray("2001:0ab8:1212:AbAb:8a2e:345.12.22.1", 2));
        assertNull(Address.toByteArray("2001:0ab8:85a3:128d3:1319:8a2e:0370:9819", 2));
    }

    public void test_toArray() {
        assertEquals(new int[]{1, 2, 3, 4}, Address.toArray("1.2.3.4", 1));
        assertEquals(new int[]{0, 0, 0, 0}, Address.toArray("0.0.0.0", 1));
        assertEquals(new int[]{255, 255, 255, 255}, Address.toArray("255.255.255.255", 1));
    }

    public void test_toArray_invalid() {
        assertNull(Address.toArray("128.121.1", 1));
        assertNull(Address.toArray(""));
    }

    public void test_isDottedQuad() {
        assertTrue(Address.isDottedQuad("1.2.3.4"));
        assertFalse(Address.isDottedQuad("256.2.3.4"));
    }

    public void test_toDottedQuad() {
        assertEquals("128.176.201.1", Address.toDottedQuad(new byte[]{UnsignedBytes.MAX_POWER_OF_TWO, -80, -55, 1}));
        assertEquals("200.1.255.128", Address.toDottedQuad(new int[]{200, 1, 255, 128}));
    }

    public void test_addressLength() {
        assertEquals(4, Address.addressLength(1));
        assertEquals(16, Address.addressLength(2));
        try {
            Address.addressLength(3);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e) {
        }
    }

    public void test_getByName() throws UnknownHostException {
        assertEquals("128.145.198.231", Address.getByName("128.145.198.231").getHostAddress());
        InetAddress out = Address.getByName("serl.cs.colorado.edu");
        assertEquals("serl.cs.colorado.edu", out.getCanonicalHostName());
        assertEquals("128.138.207.163", out.getHostAddress());
    }

    public void test_getByName_invalid() throws UnknownHostException {
        try {
            Address.getByName("bogushost.com");
            fail("UnknownHostException not thrown");
        } catch (UnknownHostException e) {
        }
        try {
            Address.getByName("");
            fail("UnknownHostException not thrown");
        } catch (UnknownHostException e2) {
        }
    }

    public void test_getAllByName() throws UnknownHostException {
        InetAddress[] out = Address.getAllByName("128.145.198.231");
        boolean z = true;
        assertEquals(1, out.length);
        int i = 0;
        assertEquals("128.145.198.231", out[0].getHostAddress());
        InetAddress[] out2 = Address.getAllByName("serl.cs.colorado.edu");
        assertEquals(1, out2.length);
        assertEquals("serl.cs.colorado.edu", out2[0].getCanonicalHostName());
        assertEquals("128.138.207.163", out2[0].getHostAddress());
        InetAddress[] out3 = Address.getAllByName("cnn.com");
        if (out3.length <= 1) {
            z = false;
        }
        assertTrue(z);
        while (true) {
            int i2 = i;
            if (i2 < out3.length) {
                assertTrue(out3[i2].getHostName().endsWith("cnn.com"));
                i = i2 + 1;
            } else {
                return;
            }
        }
    }

    public void test_getAllByName_invalid() throws UnknownHostException {
        try {
            Address.getAllByName("bogushost.com");
            fail("UnknownHostException not thrown");
        } catch (UnknownHostException e) {
        }
        try {
            Address.getAllByName("");
            fail("UnknownHostException not thrown");
        } catch (UnknownHostException e2) {
        }
    }

    public void test_familyOf() throws UnknownHostException {
        assertEquals(1, Address.familyOf(InetAddress.getByName("192.168.0.1")));
        assertEquals(2, Address.familyOf(InetAddress.getByName("1:2:3:4:5:6:7:8")));
        try {
            Address.familyOf((InetAddress) null);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e) {
        }
    }

    public void test_getHostName() throws UnknownHostException {
        assertEquals("serl.cs.colorado.edu.", Address.getHostName(InetAddress.getByName("128.138.207.163")));
        try {
            Address.getHostName(InetAddress.getByName("192.168.1.1"));
            fail("UnknownHostException not thrown");
        } catch (UnknownHostException e) {
        }
    }
}
