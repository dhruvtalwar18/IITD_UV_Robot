package org.xbill.DNS;

import java.io.IOException;
import java.util.Arrays;
import junit.framework.TestCase;

public class SingleNameBaseTest extends TestCase {
    private void assertEquals(byte[] exp, byte[] act) {
        assertTrue(Arrays.equals(exp, act));
    }

    private static class TestClass extends SingleNameBase {
        public TestClass() {
        }

        public TestClass(Name name, int type, int dclass, long ttl) {
            super(name, type, dclass, ttl);
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
        TestClass tc = new TestClass(n, 1, 1, 100);
        assertSame(n, tc.getName());
        assertEquals(1, tc.getType());
        assertEquals(1, tc.getDClass());
        assertEquals(100, tc.getTTL());
        TestClass tc2 = new TestClass(n, 1, 1, 100, sn, "The Description");
        assertSame(n, tc2.getName());
        assertEquals(1, tc2.getType());
        assertEquals(1, tc2.getDClass());
        assertEquals(100, tc2.getTTL());
        assertSame(sn, tc2.getSingleName());
    }

    public void test_rrFromWire() throws IOException {
        DNSInput in = new DNSInput(new byte[]{2, 109, 121, 6, 115, 105, 110, 103, 108, 101, 4, 110, 97, 109, 101, 0});
        TestClass tc = new TestClass();
        tc.rrFromWire(in);
        assertEquals(Name.fromString("my.single.name."), tc.getSingleName());
    }

    public void test_rdataFromString() throws IOException {
        Name exp = Name.fromString("my.single.name.");
        Tokenizer t = new Tokenizer("my.single.name.");
        TestClass tc = new TestClass();
        tc.rdataFromString(t, (Name) null);
        assertEquals(exp, tc.getSingleName());
        try {
            new TestClass().rdataFromString(new Tokenizer("my.relative.name"), (Name) null);
            fail("RelativeNameException not thrown");
        } catch (RelativeNameException e) {
        }
    }

    public void test_rrToString() throws IOException, TextParseException {
        Name exp = Name.fromString("my.single.name.");
        Tokenizer t = new Tokenizer("my.single.name.");
        TestClass tc = new TestClass();
        tc.rdataFromString(t, (Name) null);
        assertEquals(exp, tc.getSingleName());
        assertEquals(tc.rrToString(), exp.toString());
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
