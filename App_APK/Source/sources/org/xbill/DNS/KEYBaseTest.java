package org.xbill.DNS;

import com.google.common.base.Ascii;
import java.io.IOException;
import java.util.Arrays;
import junit.framework.TestCase;
import org.apache.commons.net.telnet.TelnetCommand;
import org.xbill.DNS.utils.base64;

public class KEYBaseTest extends TestCase {

    private static class TestClass extends KEYBase {
        public TestClass() {
        }

        public TestClass(Name name, int type, int dclass, long ttl, int flags, int proto, int alg, byte[] key) {
            super(name, type, dclass, ttl, flags, proto, alg, key);
        }

        public Record getObject() {
            return null;
        }

        /* access modifiers changed from: package-private */
        public void rdataFromString(Tokenizer st, Name origin) throws IOException {
        }
    }

    public void test_ctor() throws TextParseException {
        TestClass tc = new TestClass();
        assertEquals(0, tc.getFlags());
        assertEquals(0, tc.getProtocol());
        assertEquals(0, tc.getAlgorithm());
        assertNull(tc.getKey());
        Name n = Name.fromString("my.name.");
        byte[] key = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, Ascii.FF, 13, Ascii.SO, Ascii.SI};
        TestClass tc2 = new TestClass(n, 25, 1, 100, 255, 15, 14, key);
        assertSame(n, tc2.getName());
        assertEquals(25, tc2.getType());
        assertEquals(1, tc2.getDClass());
        assertEquals(100, tc2.getTTL());
        assertEquals(255, tc2.getFlags());
        assertEquals(15, tc2.getProtocol());
        assertEquals(14, tc2.getAlgorithm());
        assertTrue(Arrays.equals(key, tc2.getKey()));
    }

    public void test_rrFromWire() throws IOException {
        DNSInput in = new DNSInput(new byte[]{-85, -51, -17, Ascii.EM, 1, 2, 3, 4, 5});
        TestClass tc = new TestClass();
        tc.rrFromWire(in);
        assertEquals(43981, tc.getFlags());
        assertEquals(TelnetCommand.EOR, tc.getProtocol());
        assertEquals(25, tc.getAlgorithm());
        assertTrue(Arrays.equals(new byte[]{1, 2, 3, 4, 5}, tc.getKey()));
        DNSInput in2 = new DNSInput(new byte[]{-70, -38, -1, 40});
        TestClass tc2 = new TestClass();
        tc2.rrFromWire(in2);
        assertEquals(47834, tc2.getFlags());
        assertEquals(255, tc2.getProtocol());
        assertEquals(40, tc2.getAlgorithm());
        assertNull(tc2.getKey());
    }

    public void test_rrToString() throws IOException, TextParseException {
        Name n = Name.fromString("my.name.");
        byte[] key = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, Ascii.FF, 13, Ascii.SO, Ascii.SI};
        assertEquals("255 15 14", new TestClass(n, 25, 1, 100, 255, 15, 14, (byte[]) null).rrToString());
        TestClass tc = new TestClass(n, 25, 1, 100, 255, 15, 14, key);
        String out = tc.rrToString();
        StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append("255 15 14 ");
        stringBuffer.append(base64.toString(key));
        assertEquals(stringBuffer.toString(), out);
        Options.set("multiline");
        String out2 = tc.rrToString();
        StringBuffer stringBuffer2 = new StringBuffer();
        stringBuffer2.append("255 15 14 (\n\t");
        stringBuffer2.append(base64.toString(key));
        stringBuffer2.append(" ) ; key_tag = 18509");
        assertEquals(stringBuffer2.toString(), out2);
        Options.unset("multiline");
    }

    public void test_getFootprint() throws TextParseException {
        Name n = Name.fromString("my.name.");
        TestClass tc = new TestClass(n, 25, 1, 100, 255, 15, 1, new byte[]{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, Ascii.FF, 13, Ascii.SO, Ascii.SI});
        int foot = tc.getFootprint();
        assertEquals(3342, foot);
        assertEquals(foot, tc.getFootprint());
        TestClass tc2 = new TestClass(n, 25, 1, 100, 35243, 205, TelnetCommand.EOR, new byte[]{Ascii.DC2, 52, 86});
        int foot2 = tc2.getFootprint();
        assertEquals(49103, foot2);
        assertEquals(foot2, tc2.getFootprint());
        assertEquals(0, new TestClass().getFootprint());
    }

    public void test_rrToWire() throws IOException, TextParseException {
        TestClass tc = new TestClass(Name.fromString("my.name."), 25, 1, 100, 30345, 171, 205, new byte[]{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, Ascii.FF, 13, Ascii.SO, Ascii.SI});
        byte[] exp = {118, -119, -85, -51, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, Ascii.FF, 13, Ascii.SO, Ascii.SI};
        DNSOutput o = new DNSOutput();
        tc.rrToWire(o, (Compression) null, true);
        assertTrue(Arrays.equals(exp, o.toByteArray()));
        DNSOutput o2 = new DNSOutput();
        tc.rrToWire(o2, (Compression) null, false);
        assertTrue(Arrays.equals(exp, o2.toByteArray()));
    }
}
