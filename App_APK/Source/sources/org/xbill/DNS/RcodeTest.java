package org.xbill.DNS;

import junit.framework.TestCase;

public class RcodeTest extends TestCase {
    public void test_string() {
        assertEquals("NXDOMAIN", Rcode.string(3));
        assertEquals("NOTIMP", Rcode.string(4));
        assertTrue(Rcode.string(20).startsWith("RESERVED"));
        try {
            Rcode.string(-1);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e) {
        }
        try {
            Rcode.string(4096);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e2) {
        }
    }

    public void test_TSIGstring() {
        assertEquals("BADSIG", Rcode.TSIGstring(16));
        assertTrue(Rcode.TSIGstring(20).startsWith("RESERVED"));
        try {
            Rcode.TSIGstring(-1);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e) {
        }
        try {
            Rcode.string(65536);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e2) {
        }
    }

    public void test_value() {
        assertEquals(1, Rcode.value("FORMERR"));
        assertEquals(4, Rcode.value("NOTIMP"));
        assertEquals(4, Rcode.value("NOTIMPL"));
        assertEquals(35, Rcode.value("RESERVED35"));
        assertEquals(-1, Rcode.value("RESERVED4096"));
        assertEquals(-1, Rcode.value("THIS IS DEFINITELY UNKNOWN"));
        assertEquals(-1, Rcode.value(""));
    }
}
