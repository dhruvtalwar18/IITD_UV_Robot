package org.xbill.DNS;

import junit.framework.TestCase;

public class TypeTest extends TestCase {
    public void test_string() {
        assertEquals("CNAME", Type.string(5));
        assertTrue(Type.string(256).startsWith("TYPE"));
        try {
            Type.string(-1);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e) {
        }
    }

    public void test_value() {
        assertEquals(253, Type.value("MAILB"));
        assertEquals(300, Type.value("TYPE300"));
        assertEquals(-1, Type.value("THIS IS DEFINITELY UNKNOWN"));
        assertEquals(-1, Type.value(""));
    }

    public void test_value_2arg() {
        assertEquals(301, Type.value("301", true));
    }

    public void test_isRR() {
        assertTrue(Type.isRR(5));
        assertFalse(Type.isRR(251));
    }
}
