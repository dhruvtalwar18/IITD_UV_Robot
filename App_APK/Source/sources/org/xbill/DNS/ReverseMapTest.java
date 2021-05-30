package org.xbill.DNS;

import com.google.common.base.Ascii;
import java.net.InetAddress;
import java.net.UnknownHostException;
import junit.framework.TestCase;

public class ReverseMapTest extends TestCase {
    public void test_fromAddress_ipv4() throws UnknownHostException, TextParseException {
        Name exp = Name.fromString("1.0.168.192.in-addr.arpa.");
        assertEquals(exp, ReverseMap.fromAddress("192.168.0.1"));
        assertEquals(exp, ReverseMap.fromAddress("192.168.0.1", 1));
        assertEquals(exp, ReverseMap.fromAddress(InetAddress.getByName("192.168.0.1")));
        assertEquals(exp, ReverseMap.fromAddress(new byte[]{-64, -88, 0, 1}));
        assertEquals(exp, ReverseMap.fromAddress(new int[]{192, 168, 0, 1}));
    }

    public void test_fromAddress_ipv6() throws UnknownHostException, TextParseException {
        Name exp = Name.fromString("4.3.3.7.0.7.3.0.E.2.A.8.9.1.3.1.3.D.8.0.3.A.5.8.8.B.D.0.1.0.0.2.ip6.arpa.");
        byte[] dat = {32, 1, 13, -72, -123, -93, 8, -45, 19, Ascii.EM, -118, 46, 3, 112, 115, 52};
        assertEquals(exp, ReverseMap.fromAddress("2001:0db8:85a3:08d3:1319:8a2e:0370:7334", 2));
        assertEquals(exp, ReverseMap.fromAddress(InetAddress.getByName("2001:0db8:85a3:08d3:1319:8a2e:0370:7334")));
        assertEquals(exp, ReverseMap.fromAddress(dat));
        assertEquals(exp, ReverseMap.fromAddress(new int[]{32, 1, 13, 184, 133, 163, 8, 211, 19, 25, 138, 46, 3, 112, 115, 52}));
    }

    public void test_fromAddress_invalid() {
        try {
            ReverseMap.fromAddress("A.B.C.D", 1);
            fail("UnknownHostException not thrown");
        } catch (UnknownHostException e) {
        }
        try {
            ReverseMap.fromAddress(new byte[0]);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e2) {
        }
        try {
            ReverseMap.fromAddress(new byte[3]);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e3) {
        }
        try {
            ReverseMap.fromAddress(new byte[5]);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e4) {
        }
        try {
            ReverseMap.fromAddress(new byte[15]);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e5) {
        }
        try {
            ReverseMap.fromAddress(new byte[17]);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e6) {
        }
        try {
            ReverseMap.fromAddress(new int[]{0, 1, 2, 256});
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e7) {
        }
    }
}
