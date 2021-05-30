package org.xbill.DNS;

import junit.framework.TestCase;

public class ExtendedFlagsTest extends TestCase {
    public void test_string() {
        assertEquals("do", ExtendedFlags.string(32768));
        assertTrue(ExtendedFlags.string(1).startsWith("flag"));
        try {
            ExtendedFlags.string(-1);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e) {
        }
        try {
            ExtendedFlags.string(65536);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e2) {
        }
    }

    public void test_value() {
        assertEquals(32768, ExtendedFlags.value("do"));
        assertEquals(16, ExtendedFlags.value("FLAG16"));
        assertEquals(-1, ExtendedFlags.value("FLAG65536"));
        assertEquals(-1, ExtendedFlags.value("THIS IS DEFINITELY UNKNOWN"));
        assertEquals(-1, ExtendedFlags.value(""));
    }
}
