package org.xbill.DNS;

import java.io.IOException;
import java.net.UnknownHostException;
import java.util.Arrays;
import junit.framework.TestCase;
import org.xbill.DNS.KEYRecord;

public class KEYRecordTest extends TestCase {
    public void test_ctor_0arg() throws UnknownHostException {
        KEYRecord ar = new KEYRecord();
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
        assertTrue(new KEYRecord().getObject() instanceof KEYRecord);
    }

    public void test_ctor_7arg() throws TextParseException {
        Name n = Name.fromString("My.Absolute.Name.");
        Name r = Name.fromString("My.Relative.Name");
        byte[] key = {0, 1, 3, 5, 7, 9};
        KEYRecord kr = new KEYRecord(n, 1, 9388, 38962, 18, 103, key);
        assertEquals(n, kr.getName());
        assertEquals(25, kr.getType());
        assertEquals(1, kr.getDClass());
        assertEquals(9388, kr.getTTL());
        assertEquals(38962, kr.getFlags());
        assertEquals(18, kr.getProtocol());
        assertEquals(103, kr.getAlgorithm());
        assertTrue(Arrays.equals(key, kr.getKey()));
        try {
            new KEYRecord(r, 1, 9388, 38962, 18, 103, key);
            fail("RelativeNameException not thrown");
        } catch (RelativeNameException e) {
        }
    }

    public void test_Protocol_string() {
        assertEquals("DNSSEC", KEYRecord.Protocol.string(3));
        assertEquals("254", KEYRecord.Protocol.string(254));
        try {
            KEYRecord.Protocol.string(-1);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e) {
        }
        try {
            KEYRecord.Protocol.string(256);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e2) {
        }
    }

    public void test_Protocol_value() {
        assertEquals(4, KEYRecord.Protocol.value("IPSEC"));
        assertEquals(254, KEYRecord.Protocol.value("254"));
        assertEquals(-1, KEYRecord.Protocol.value("-2"));
        assertEquals(-1, KEYRecord.Protocol.value("256"));
    }

    public void test_Flags_value() {
        assertEquals(-1, KEYRecord.Flags.value("-2"));
        assertEquals(0, KEYRecord.Flags.value("0"));
        assertEquals(43829, KEYRecord.Flags.value("43829"));
        assertEquals(65535, KEYRecord.Flags.value("65535"));
        assertEquals(-1, KEYRecord.Flags.value("65536"));
        assertEquals(4096, KEYRecord.Flags.value("EXTEND"));
        assertEquals(-1, KEYRecord.Flags.value("NOT_A_VALID_NAME"));
        assertEquals(33056, KEYRecord.Flags.value("NOAUTH|ZONE|FLAG10"));
        assertEquals(-1, KEYRecord.Flags.value("NOAUTH|INVALID_NAME|FLAG10"));
        assertEquals(0, KEYRecord.Flags.value("|"));
    }

    public void test_rdataFromString() throws IOException, TextParseException {
        KEYRecord kr = new KEYRecord();
        kr.rdataFromString(new Tokenizer("NOAUTH|ZONE|FLAG10 EMAIL RSASHA1 AQIDBAUGBwgJ"), (Name) null);
        assertEquals(33056, kr.getFlags());
        assertEquals(2, kr.getProtocol());
        assertEquals(5, kr.getAlgorithm());
        assertTrue(Arrays.equals(new byte[]{1, 2, 3, 4, 5, 6, 7, 8, 9}, kr.getKey()));
        KEYRecord kr2 = new KEYRecord();
        kr2.rdataFromString(new Tokenizer("NOAUTH|NOKEY|FLAG10 TLS ECC"), (Name) null);
        assertEquals(49184, kr2.getFlags());
        assertEquals(1, kr2.getProtocol());
        assertEquals(4, kr2.getAlgorithm());
        assertNull(kr2.getKey());
        try {
            new KEYRecord().rdataFromString(new Tokenizer("NOAUTH|ZONE|JUNK EMAIL RSASHA1 AQIDBAUGBwgJ"), (Name) null);
            fail("TextParseException not thrown");
        } catch (TextParseException e) {
        }
        try {
            new KEYRecord().rdataFromString(new Tokenizer("NOAUTH|ZONE RSASHA1 ECC AQIDBAUGBwgJ"), (Name) null);
            fail("TextParseException not thrown");
        } catch (TextParseException e2) {
        }
        try {
            new KEYRecord().rdataFromString(new Tokenizer("NOAUTH|ZONE EMAIL ZONE AQIDBAUGBwgJ"), (Name) null);
            fail("TextParseException not thrown");
        } catch (TextParseException e3) {
        }
    }
}
