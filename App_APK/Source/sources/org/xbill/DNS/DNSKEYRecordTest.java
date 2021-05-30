package org.xbill.DNS;

import java.io.IOException;
import java.net.UnknownHostException;
import java.util.Arrays;
import junit.framework.TestCase;

public class DNSKEYRecordTest extends TestCase {
    public void test_ctor_0arg() throws UnknownHostException {
        DNSKEYRecord ar = new DNSKEYRecord();
        assertNull(ar.getName());
        assertEquals(0, ar.getType());
        assertEquals(0, ar.getDClass());
        assertEquals(0, ar.getTTL());
        assertEquals(0, ar.getAlgorithm());
        assertEquals(0, ar.getFlags());
        assertEquals(0, ar.getFootprint());
        assertEquals(0, ar.getProtocol());
        assertNull(ar.getKey());
    }

    public void test_getObject() {
        assertTrue(new DNSKEYRecord().getObject() instanceof DNSKEYRecord);
    }

    public void test_ctor_7arg() throws TextParseException {
        Name n = Name.fromString("My.Absolute.Name.");
        Name r = Name.fromString("My.Relative.Name");
        byte[] key = {0, 1, 3, 5, 7, 9};
        DNSKEYRecord kr = new DNSKEYRecord(n, 1, 9388, 38962, 18, 103, key);
        assertEquals(n, kr.getName());
        assertEquals(48, kr.getType());
        assertEquals(1, kr.getDClass());
        assertEquals(9388, kr.getTTL());
        assertEquals(38962, kr.getFlags());
        assertEquals(18, kr.getProtocol());
        assertEquals(103, kr.getAlgorithm());
        assertTrue(Arrays.equals(key, kr.getKey()));
        try {
            new DNSKEYRecord(r, 1, 9388, 38962, 18, 103, key);
            fail("RelativeNameException not thrown");
        } catch (RelativeNameException e) {
        }
    }

    public void test_rdataFromString() throws IOException, TextParseException {
        DNSKEYRecord kr = new DNSKEYRecord();
        kr.rdataFromString(new Tokenizer("43981 129 RSASHA1 AQIDBAUGBwgJ"), (Name) null);
        assertEquals(43981, kr.getFlags());
        assertEquals(129, kr.getProtocol());
        assertEquals(5, kr.getAlgorithm());
        assertTrue(Arrays.equals(new byte[]{1, 2, 3, 4, 5, 6, 7, 8, 9}, kr.getKey()));
        try {
            new DNSKEYRecord().rdataFromString(new Tokenizer("4626 170 ZONE AQIDBAUGBwgJ"), (Name) null);
            fail("TextParseException not thrown");
        } catch (TextParseException e) {
        }
    }
}
