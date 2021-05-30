package org.xbill.DNS.utils;

import junit.framework.TestCase;

public class base64Test extends TestCase {
    public base64Test(String name) {
        super(name);
    }

    public void test_toString_empty() {
        assertEquals("", base64.toString(new byte[0]));
    }

    public void test_toString_basic1() {
        assertEquals("AA==", base64.toString(new byte[]{0}));
    }

    public void test_toString_basic2() {
        assertEquals("AAA=", base64.toString(new byte[]{0, 0}));
    }

    public void test_toString_basic3() {
        assertEquals("AAAB", base64.toString(new byte[]{0, 0, 1}));
    }

    public void test_toString_basic4() {
        assertEquals("/AAA", base64.toString(new byte[]{-4, 0, 0}));
    }

    public void test_toString_basic5() {
        assertEquals("////", base64.toString(new byte[]{-1, -1, -1}));
    }

    public void test_toString_basic6() {
        assertEquals("AQIDBAUGBwgJ", base64.toString(new byte[]{1, 2, 3, 4, 5, 6, 7, 8, 9}));
    }

    public void test_formatString_empty1() {
        assertEquals("", base64.formatString(new byte[0], 5, "", false));
    }

    public void test_formatString_shorter() {
        assertEquals("AQIDBAUGBwgJ", base64.formatString(new byte[]{1, 2, 3, 4, 5, 6, 7, 8, 9}, 13, "", false));
    }

    public void test_formatString_sameLength() {
        assertEquals("AQIDBAUGBwgJ", base64.formatString(new byte[]{1, 2, 3, 4, 5, 6, 7, 8, 9}, 12, "", false));
    }

    public void test_formatString_oneBreak() {
        assertEquals("AQIDBAUGBw\ngJ", base64.formatString(new byte[]{1, 2, 3, 4, 5, 6, 7, 8, 9}, 10, "", false));
    }

    public void test_formatString_twoBreaks1() {
        assertEquals("AQIDB\nAUGBw\ngJ", base64.formatString(new byte[]{1, 2, 3, 4, 5, 6, 7, 8, 9}, 5, "", false));
    }

    public void test_formatString_twoBreaks2() {
        assertEquals("AQID\nBAUG\nBwgJ", base64.formatString(new byte[]{1, 2, 3, 4, 5, 6, 7, 8, 9}, 4, "", false));
    }

    public void test_formatString_shorterWithPrefix() {
        assertEquals("!_AQIDBAUGBwgJ", base64.formatString(new byte[]{1, 2, 3, 4, 5, 6, 7, 8, 9}, 13, "!_", false));
    }

    public void test_formatString_sameLengthWithPrefix() {
        assertEquals("!_AQIDBAUGBwgJ", base64.formatString(new byte[]{1, 2, 3, 4, 5, 6, 7, 8, 9}, 12, "!_", false));
    }

    public void test_formatString_oneBreakWithPrefix() {
        assertEquals("!_AQIDBAUGBw\n!_gJ", base64.formatString(new byte[]{1, 2, 3, 4, 5, 6, 7, 8, 9}, 10, "!_", false));
    }

    public void test_formatString_twoBreaks1WithPrefix() {
        assertEquals("!_AQIDB\n!_AUGBw\n!_gJ", base64.formatString(new byte[]{1, 2, 3, 4, 5, 6, 7, 8, 9}, 5, "!_", false));
    }

    public void test_formatString_twoBreaks2WithPrefix() {
        assertEquals("!_AQID\n!_BAUG\n!_BwgJ", base64.formatString(new byte[]{1, 2, 3, 4, 5, 6, 7, 8, 9}, 4, "!_", false));
    }

    public void test_formatString_shorterWithPrefixAndClose() {
        assertEquals("!_AQIDBAUGBwgJ )", base64.formatString(new byte[]{1, 2, 3, 4, 5, 6, 7, 8, 9}, 13, "!_", true));
    }

    public void test_formatString_sameLengthWithPrefixAndClose() {
        assertEquals("!_AQIDBAUGBwgJ )", base64.formatString(new byte[]{1, 2, 3, 4, 5, 6, 7, 8, 9}, 12, "!_", true));
    }

    public void test_formatString_oneBreakWithPrefixAndClose() {
        assertEquals("!_AQIDBAUGBw\n!_gJ )", base64.formatString(new byte[]{1, 2, 3, 4, 5, 6, 7, 8, 9}, 10, "!_", true));
    }

    public void test_formatString_twoBreaks1WithPrefixAndClose() {
        assertEquals("!_AQIDB\n!_AUGBw\n!_gJ )", base64.formatString(new byte[]{1, 2, 3, 4, 5, 6, 7, 8, 9}, 5, "!_", true));
    }

    public void test_formatString_twoBreaks2WithPrefixAndClose() {
        assertEquals("!_AQID\n!_BAUG\n!_BwgJ )", base64.formatString(new byte[]{1, 2, 3, 4, 5, 6, 7, 8, 9}, 4, "!_", true));
    }

    private void assertEquals(byte[] exp, byte[] act) {
        assertEquals(exp.length, act.length);
        for (int i = 0; i < exp.length; i++) {
            assertEquals(exp[i], act[i]);
        }
    }

    public void test_fromString_empty1() {
        byte[] bArr = new byte[0];
        assertEquals(new byte[0], base64.fromString(""));
    }

    public void test_fromString_basic1() {
        assertEquals(new byte[]{0}, base64.fromString("AA=="));
    }

    public void test_fromString_basic2() {
        assertEquals(new byte[]{0, 0}, base64.fromString("AAA="));
    }

    public void test_fromString_basic3() {
        assertEquals(new byte[]{0, 0, 1}, base64.fromString("AAAB"));
    }

    public void test_fromString_basic4() {
        assertEquals(new byte[]{-4, 0, 0}, base64.fromString("/AAA"));
    }

    public void test_fromString_basic5() {
        assertEquals(new byte[]{-1, -1, -1}, base64.fromString("////"));
    }

    public void test_fromString_basic6() {
        assertEquals(new byte[]{1, 2, 3, 4, 5, 6, 7, 8, 9}, base64.fromString("AQIDBAUGBwgJ"));
    }

    public void test_fromString_invalid1() {
        assertNull(base64.fromString("AAA"));
    }

    public void test_fromString_invalid2() {
        assertNull(base64.fromString("AA"));
    }

    public void test_fromString_invalid3() {
        assertNull(base64.fromString("A"));
    }

    public void test_fromString_invalid4() {
        assertNull(base64.fromString("BB=="));
    }

    public void test_fromString_invalid5() {
        assertNull(base64.fromString("BBB="));
    }
}
