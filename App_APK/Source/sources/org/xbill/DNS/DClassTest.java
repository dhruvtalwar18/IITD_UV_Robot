package org.xbill.DNS;

import junit.framework.TestCase;

public class DClassTest extends TestCase {
    public void test_string() {
        assertEquals("IN", DClass.string(1));
        assertEquals("CH", DClass.string(3));
        assertTrue(DClass.string(20).startsWith("CLASS"));
        try {
            DClass.string(-1);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e) {
        }
        try {
            DClass.string(65536);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e2) {
        }
    }

    public void test_value() {
        assertEquals(254, DClass.value("NONE"));
        assertEquals(4, DClass.value("HS"));
        assertEquals(4, DClass.value("HESIOD"));
        assertEquals(21, DClass.value("CLASS21"));
        assertEquals(-1, DClass.value("THIS IS DEFINITELY UNKNOWN"));
        assertEquals(-1, DClass.value(""));
    }
}
