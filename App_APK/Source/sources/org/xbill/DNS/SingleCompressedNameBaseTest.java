package org.xbill.DNS;

import java.io.IOException;
import java.util.Arrays;
import junit.framework.TestCase;

public class SingleCompressedNameBaseTest extends TestCase {
    private void assertEquals(byte[] exp, byte[] act) {
        assertTrue(Arrays.equals(exp, act));
    }

    private static class TestClass extends SingleCompressedNameBase {
        public TestClass() {
        }

        public TestClass(Name name, int type, int dclass, long ttl, Name singleName, String desc) {
            super(name, type, dclass, ttl, singleName, desc);
        }

        public Name getSingleName() {
            return super.getSingleName();
        }

        public Record getObject() {
            return null;
        }
    }

    public void test_ctor() throws TextParseException {
        assertNull(new TestClass().getSingleName());
        Name n = Name.fromString("my.name.");
        Name sn = Name.fromString("my.single.name.");
        TestClass tc = new TestClass(n, 1, 1, 100, sn, "The Description");
        assertSame(n, tc.getName());
        assertEquals(1, tc.getType());
        assertEquals(1, tc.getDClass());
        assertEquals(100, tc.getTTL());
        assertSame(sn, tc.getSingleName());
    }

    public void test_rrToWire() throws IOException, TextParseException {
        Name fromString = Name.fromString("my.name.");
        Name fromString2 = Name.fromString("My.Single.Name.");
        TestClass testClass = new TestClass(fromString, 1, 1, 100, fromString2, "The Description");
        DNSOutput dout = new DNSOutput();
        testClass.rrToWire(dout, (Compression) null, false);
        byte[] out = dout.toByteArray();
        assertEquals(new byte[]{2, 77, 121, 6, 83, 105, 110, 103, 108, 101, 4, 78, 97, 109, 101, 0}, out);
        byte[] bArr = out;
        TestClass tc = new TestClass(fromString, 1, 1, 100, fromString2, "The Description");
        DNSOutput dout2 = new DNSOutput();
        tc.rrToWire(dout2, (Compression) null, true);
        assertEquals(new byte[]{2, 109, 121, 6, 115, 105, 110, 103, 108, 101, 4, 110, 97, 109, 101, 0}, dout2.toByteArray());
    }
}
