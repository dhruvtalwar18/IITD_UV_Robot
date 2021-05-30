package org.xbill.DNS.utils;

import com.google.common.base.Ascii;
import junit.framework.TestCase;

public class base16Test extends TestCase {
    public base16Test(String name) {
        super(name);
    }

    public void test_toString_emptyArray() {
        assertEquals("", base16.toString(new byte[0]));
    }

    public void test_toString_singleByte1() {
        assertEquals("01", base16.toString(new byte[]{1}));
    }

    public void test_toString_singleByte2() {
        assertEquals("10", base16.toString(new byte[]{16}));
    }

    public void test_toString_singleByte3() {
        assertEquals("FF", base16.toString(new byte[]{-1}));
    }

    public void test_toString_array1() {
        assertEquals("0102030405060708090A0B0C0D0E0F", base16.toString(new byte[]{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, Ascii.FF, 13, Ascii.SO, Ascii.SI}));
    }

    public void test_fromString_emptyString() {
        assertEquals(0, base16.fromString("").length);
    }

    public void test_fromString_invalidStringLength() {
        assertNull(base16.fromString("1"));
    }

    public void test_fromString_nonHexChars() {
        byte[] fromString = base16.fromString("GG");
    }

    public void test_fromString_normal() {
        byte[] out = base16.fromString("0102030405060708090A0B0C0D0E0F");
        byte[] exp = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, Ascii.FF, 13, Ascii.SO, Ascii.SI};
        assertEquals(exp.length, out.length);
        for (int i = 0; i < exp.length; i++) {
            assertEquals(exp[i], out[i]);
        }
    }
}
