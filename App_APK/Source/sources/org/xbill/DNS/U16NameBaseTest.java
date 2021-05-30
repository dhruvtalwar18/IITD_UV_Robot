package org.xbill.DNS;

import java.io.IOException;
import java.util.Arrays;
import junit.framework.TestCase;

public class U16NameBaseTest extends TestCase {
    private void assertEquals(byte[] exp, byte[] act) {
        assertTrue(Arrays.equals(exp, act));
    }

    private static class TestClass extends U16NameBase {
        public TestClass() {
        }

        public TestClass(Name name, int type, int dclass, long ttl) {
            super(name, type, dclass, ttl);
        }

        public TestClass(Name name, int type, int dclass, long ttl, int u16Field, String u16Description, Name nameField, String nameDescription) {
            super(name, type, dclass, ttl, u16Field, u16Description, nameField, nameDescription);
        }

        public int getU16Field() {
            return super.getU16Field();
        }

        public Name getNameField() {
            return super.getNameField();
        }

        public Record getObject() {
            return null;
        }
    }

    public void test_ctor_0arg() {
        TestClass tc = new TestClass();
        assertNull(tc.getName());
        assertEquals(0, tc.getType());
        assertEquals(0, tc.getDClass());
        assertEquals(0, tc.getTTL());
        assertEquals(0, tc.getU16Field());
        assertNull(tc.getNameField());
    }

    public void test_ctor_4arg() throws TextParseException {
        Name n = Name.fromString("My.Name.");
        TestClass tc = new TestClass(n, 15, 1, 48346);
        assertSame(n, tc.getName());
        assertEquals(15, tc.getType());
        assertEquals(1, tc.getDClass());
        assertEquals(48346, tc.getTTL());
        assertEquals(0, tc.getU16Field());
        assertNull(tc.getNameField());
    }

    public void test_ctor_8arg() throws TextParseException {
        Name n = Name.fromString("My.Name.");
        Name m = Name.fromString("My.Other.Name.");
        TestClass testClass = new TestClass(n, 15, 1, 45359, 7979, "u16 description", m, "name description");
        assertSame(n, testClass.getName());
        assertEquals(15, testClass.getType());
        assertEquals(1, testClass.getDClass());
        assertEquals(45359, testClass.getTTL());
        assertEquals(7979, testClass.getU16Field());
        assertEquals(m, testClass.getNameField());
        try {
            new TestClass(n, 15, 1, 45359, 65536, "u16 description", m, "name description");
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e) {
        }
        try {
            new TestClass(n, 15, 1, 45359, 7979, "u16 description", Name.fromString("My.relative.Name"), "name description");
            fail("RelativeNameException not thrown");
        } catch (RelativeNameException e2) {
        }
    }

    public void test_rrFromWire() throws IOException {
        DNSInput in = new DNSInput(new byte[]{-68, 31, 2, 77, 121, 6, 115, 105, 78, 103, 108, 69, 4, 110, 65, 109, 69, 0});
        TestClass tc = new TestClass();
        tc.rrFromWire(in);
        Name exp = Name.fromString("My.single.name.");
        assertEquals(48159, (long) tc.getU16Field());
        assertEquals(exp, tc.getNameField());
    }

    public void test_rdataFromString() throws IOException {
        Name exp = Name.fromString("My.Single.Name.");
        Tokenizer t = new Tokenizer("6562 My.Single.Name.");
        TestClass tc = new TestClass();
        tc.rdataFromString(t, (Name) null);
        assertEquals(6562, tc.getU16Field());
        assertEquals(exp, tc.getNameField());
        try {
            new TestClass().rdataFromString(new Tokenizer("10 My.Relative.Name"), (Name) null);
            fail("RelativeNameException not thrown");
        } catch (RelativeNameException e) {
        }
    }

    public void test_rrToString() throws IOException, TextParseException {
        assertEquals("7979 My.Other.Name.", new TestClass(Name.fromString("My.Name."), 15, 1, 45359, 7979, "u16 description", Name.fromString("My.Other.Name."), "name description").rrToString());
    }

    public void test_rrToWire() throws IOException, TextParseException {
        TestClass tc = new TestClass(Name.fromString("My.Name."), 15, 1, 45359, 7979, "u16 description", Name.fromString("M.O.n."), "name description");
        DNSOutput dout = new DNSOutput();
        tc.rrToWire(dout, (Compression) null, true);
        assertTrue(Arrays.equals(new byte[]{31, 43, 1, 109, 1, 111, 1, 110, 0}, dout.toByteArray()));
        DNSOutput dout2 = new DNSOutput();
        tc.rrToWire(dout2, (Compression) null, false);
        assertTrue(Arrays.equals(new byte[]{31, 43, 1, 77, 1, 79, 1, 110, 0}, dout2.toByteArray()));
    }
}
