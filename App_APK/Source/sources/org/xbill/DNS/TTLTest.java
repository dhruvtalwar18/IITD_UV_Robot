package org.xbill.DNS;

import junit.framework.TestCase;

public class TTLTest extends TestCase {
    private final long D = 86400;
    private final long H = 3600;
    private final long M = 60;
    private final long S = 1;
    private final long W = 604800;

    public void test_parseTTL() {
        assertEquals(9876, TTL.parseTTL("9876"));
        assertEquals(0, TTL.parseTTL("0S"));
        assertEquals(0, TTL.parseTTL("0M"));
        assertEquals(0, TTL.parseTTL("0H"));
        assertEquals(0, TTL.parseTTL("0D"));
        assertEquals(0, TTL.parseTTL("0W"));
        assertEquals(1, TTL.parseTTL("1s"));
        assertEquals(60, TTL.parseTTL("1m"));
        assertEquals(3600, TTL.parseTTL("1h"));
        assertEquals(86400, TTL.parseTTL("1d"));
        assertEquals(604800, TTL.parseTTL("1w"));
        assertEquals(98, TTL.parseTTL("98S"));
        assertEquals(4560, TTL.parseTTL("76M"));
        assertEquals(194400, TTL.parseTTL("54H"));
        assertEquals(2764800, TTL.parseTTL("32D"));
        assertEquals(6048000, TTL.parseTTL("10W"));
        assertEquals(5220758, TTL.parseTTL("98S11M1234H2D01W"));
    }

    public void test_parseTTL_invalid() {
        try {
            TTL.parseTTL((String) null);
            fail("NumberFormatException not throw");
        } catch (NumberFormatException e) {
        }
        try {
            TTL.parseTTL("");
            fail("NumberFormatException not throw");
        } catch (NumberFormatException e2) {
        }
        try {
            TTL.parseTTL("S");
            fail("NumberFormatException not throw");
        } catch (NumberFormatException e3) {
        }
        try {
            TTL.parseTTL("10S4B");
            fail("NumberFormatException not throw");
        } catch (NumberFormatException e4) {
        }
        try {
            TTL.parseTTL("1S4294967295S");
            fail("NumberFormatException not throw");
        } catch (NumberFormatException e5) {
        }
        try {
            TTL.parseTTL("4294967296");
            fail("NumberFormatException not throw");
        } catch (NumberFormatException e6) {
        }
    }

    public void test_format() {
        assertEquals("0S", TTL.format(0));
        assertEquals("1S", TTL.format(1));
        assertEquals("59S", TTL.format(59));
        assertEquals("1M", TTL.format(60));
        assertEquals("59M", TTL.format(3540));
        assertEquals("1M33S", TTL.format(93));
        assertEquals("59M59S", TTL.format(3599));
        assertEquals("1H", TTL.format(3600));
        assertEquals("10H1M21S", TTL.format(36081));
        assertEquals("23H59M59S", TTL.format(86399));
        assertEquals("1D", TTL.format(86400));
        assertEquals("4D18H45M30S", TTL.format(413130));
        assertEquals("6D23H59M59S", TTL.format(604799));
        assertEquals("1W", TTL.format(604800));
        assertEquals("10W4D1H21M29S", TTL.format(6398489));
        assertEquals("3550W5D3H14M7S", TTL.format(TTL.MAX_VALUE));
    }

    public void test_format_invalid() {
        try {
            TTL.format(-1);
            fail("InvalidTTLException not thrown");
        } catch (InvalidTTLException e) {
        }
        try {
            TTL.format(4294967296L);
            fail("InvalidTTLException not thrown");
        } catch (InvalidTTLException e2) {
        }
    }
}
