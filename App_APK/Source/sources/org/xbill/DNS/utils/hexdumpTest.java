package org.xbill.DNS.utils;

import com.google.common.base.Ascii;
import junit.framework.TestCase;

public class hexdumpTest extends TestCase {
    public hexdumpTest(String name) {
        super(name);
    }

    public void test_shortform() {
        byte[] data = {1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3};
        assertEquals(hexdump.dump("This Is My Description", data, 0, data.length), hexdump.dump("This Is My Description", data));
    }

    public void test_0() {
        assertEquals("1b:\t00 \n", hexdump.dump((String) null, new byte[]{1, 0, 2}, 1, 1));
    }

    public void test_1() {
        assertEquals("1b:\t01 \n", hexdump.dump((String) null, new byte[]{2, 1, 3}, 1, 1));
    }

    public void test_2() {
        assertEquals("1b:\t02 \n", hexdump.dump((String) null, new byte[]{1, 2, 3}, 1, 1));
    }

    public void test_3() {
        assertEquals("1b:\t03 \n", hexdump.dump((String) null, new byte[]{1, 3, 2}, 1, 1));
    }

    public void test_4() {
        assertEquals("1b:\t04 \n", hexdump.dump((String) null, new byte[]{1, 4, 2}, 1, 1));
    }

    public void test_5() {
        assertEquals("1b:\t05 \n", hexdump.dump((String) null, new byte[]{1, 5, 2}, 1, 1));
    }

    public void test_6() {
        assertEquals("1b:\t06 \n", hexdump.dump((String) null, new byte[]{1, 6, 2}, 1, 1));
    }

    public void test_7() {
        assertEquals("1b:\t07 \n", hexdump.dump((String) null, new byte[]{1, 7, 2}, 1, 1));
    }

    public void test_8() {
        assertEquals("1b:\t08 \n", hexdump.dump((String) null, new byte[]{1, 8, 2}, 1, 1));
    }

    public void test_9() {
        assertEquals("1b:\t09 \n", hexdump.dump((String) null, new byte[]{1, 9, 2}, 1, 1));
    }

    public void test_10() {
        assertEquals("1b:\t0A \n", hexdump.dump((String) null, new byte[]{1, 10, 2}, 1, 1));
    }

    public void test_11() {
        assertEquals("1b:\t0B \n", hexdump.dump((String) null, new byte[]{1, 11, 2}, 1, 1));
    }

    public void test_12() {
        assertEquals("1b:\t0C \n", hexdump.dump((String) null, new byte[]{1, Ascii.FF, 2}, 1, 1));
    }

    public void test_13() {
        assertEquals("1b:\t0D \n", hexdump.dump((String) null, new byte[]{1, 13, 2}, 1, 1));
    }

    public void test_14() {
        assertEquals("1b:\t0E \n", hexdump.dump((String) null, new byte[]{1, Ascii.SO, 2}, 1, 1));
    }

    public void test_15() {
        assertEquals("1b:\t0F \n", hexdump.dump((String) null, new byte[]{1, Ascii.SI, 2}, 1, 1));
    }

    public void test_default_constructor() {
        new hexdump();
    }
}
