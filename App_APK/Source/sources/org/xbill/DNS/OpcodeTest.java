package org.xbill.DNS;

import junit.framework.TestCase;

public class OpcodeTest extends TestCase {
    public void test_string() {
        assertEquals("IQUERY", Opcode.string(1));
        assertTrue(Opcode.string(6).startsWith("RESERVED"));
        try {
            Opcode.string(-1);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e) {
        }
        try {
            Opcode.string(16);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e2) {
        }
    }

    public void test_value() {
        assertEquals(2, Opcode.value("STATUS"));
        assertEquals(6, Opcode.value("RESERVED6"));
        assertEquals(-1, Opcode.value("RESERVED16"));
        assertEquals(-1, Opcode.value("THIS IS DEFINITELY UNKNOWN"));
        assertEquals(-1, Opcode.value(""));
    }
}
