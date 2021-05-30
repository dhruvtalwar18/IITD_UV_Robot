package org.xbill.DNS;

import com.google.common.primitives.SignedBytes;
import java.io.IOException;
import java.util.Arrays;
import junit.framework.Test;
import junit.framework.TestCase;
import junit.framework.TestSuite;
import org.bytedeco.javacpp.opencv_cudaimgproc;
import org.jboss.netty.handler.codec.http.HttpConstants;
import rocon_app_manager_msgs.ErrorCodes;

public class NameTest extends TestCase {
    static /* synthetic */ Class class$org$xbill$DNS$NameTest;
    static /* synthetic */ Class class$org$xbill$DNS$NameTest$Test_DNSInput_init;
    static /* synthetic */ Class class$org$xbill$DNS$NameTest$Test_String_init;
    static /* synthetic */ Class class$org$xbill$DNS$NameTest$Test_compareTo;
    static /* synthetic */ Class class$org$xbill$DNS$NameTest$Test_equals;
    static /* synthetic */ Class class$org$xbill$DNS$NameTest$Test_toWire;
    static /* synthetic */ Class class$org$xbill$DNS$NameTest$Test_toWireCanonical;

    public static class Test_String_init extends TestCase {
        private final String m_abs = "WWW.DnsJava.org.";
        private Name m_abs_origin;
        private final String m_rel = "WWW.DnsJava";
        private Name m_rel_origin;

        /* access modifiers changed from: protected */
        public void setUp() throws TextParseException {
            this.m_abs_origin = Name.fromString("Orig.");
            this.m_rel_origin = Name.fromString("Orig");
        }

        public void test_ctor_empty() {
            try {
                new Name("");
                fail("TextParseException not thrown");
            } catch (TextParseException e) {
            }
        }

        public void test_ctor_at_null_origin() throws TextParseException {
            Name n = new Name("@");
            assertFalse(n.isAbsolute());
            assertFalse(n.isWild());
            assertEquals(0, n.labels());
            assertEquals(0, n.length());
        }

        public void test_ctor_at_abs_origin() throws TextParseException {
            assertEquals(this.m_abs_origin, new Name("@", this.m_abs_origin));
        }

        public void test_ctor_at_rel_origin() throws TextParseException {
            assertEquals(this.m_rel_origin, new Name("@", this.m_rel_origin));
        }

        public void test_ctor_dot() throws TextParseException {
            Name n = new Name(".");
            assertEquals(Name.root, n);
            assertNotSame(Name.root, n);
            assertEquals(1, n.labels());
            assertEquals(1, n.length());
        }

        public void test_ctor_wildcard() throws TextParseException {
            Name n = new Name("*");
            assertFalse(n.isAbsolute());
            assertTrue(n.isWild());
            assertEquals(1, n.labels());
            assertEquals(2, n.length());
            assertTrue(Arrays.equals(new byte[]{1, 42}, n.getLabel(0)));
            assertEquals("*", n.getLabelString(0));
        }

        public void test_ctor_abs() throws TextParseException {
            Name n = new Name("WWW.DnsJava.org.");
            assertTrue(n.isAbsolute());
            assertFalse(n.isWild());
            assertEquals(4, n.labels());
            assertEquals(17, n.length());
            assertTrue(Arrays.equals(new byte[]{3, 87, 87, 87}, n.getLabel(0)));
            assertEquals("WWW", n.getLabelString(0));
            assertTrue(Arrays.equals(new byte[]{7, 68, 110, 115, 74, 97, 118, 97}, n.getLabel(1)));
            assertEquals("DnsJava", n.getLabelString(1));
            assertTrue(Arrays.equals(new byte[]{3, 111, 114, 103}, n.getLabel(2)));
            assertEquals("org", n.getLabelString(2));
            assertTrue(Arrays.equals(new byte[]{0}, n.getLabel(3)));
            assertEquals("", n.getLabelString(3));
        }

        public void test_ctor_rel() throws TextParseException {
            Name n = new Name("WWW.DnsJava");
            assertFalse(n.isAbsolute());
            assertFalse(n.isWild());
            assertEquals(2, n.labels());
            assertEquals(12, n.length());
            assertTrue(Arrays.equals(new byte[]{3, 87, 87, 87}, n.getLabel(0)));
            assertEquals("WWW", n.getLabelString(0));
            assertTrue(Arrays.equals(new byte[]{7, 68, 110, 115, 74, 97, 118, 97}, n.getLabel(1)));
            assertEquals("DnsJava", n.getLabelString(1));
        }

        public void test_ctor_7label() throws TextParseException {
            Name n = new Name("a.b.c.d.e.f.");
            assertTrue(n.isAbsolute());
            assertFalse(n.isWild());
            assertEquals(7, n.labels());
            assertEquals(13, n.length());
            assertTrue(Arrays.equals(new byte[]{1, 97}, n.getLabel(0)));
            assertEquals("a", n.getLabelString(0));
            assertTrue(Arrays.equals(new byte[]{1, 98}, n.getLabel(1)));
            assertEquals("b", n.getLabelString(1));
            assertTrue(Arrays.equals(new byte[]{1, 99}, n.getLabel(2)));
            assertEquals("c", n.getLabelString(2));
            assertTrue(Arrays.equals(new byte[]{1, 100}, n.getLabel(3)));
            assertEquals("d", n.getLabelString(3));
            assertTrue(Arrays.equals(new byte[]{1, 101}, n.getLabel(4)));
            assertEquals("e", n.getLabelString(4));
            assertTrue(Arrays.equals(new byte[]{1, 102}, n.getLabel(5)));
            assertEquals("f", n.getLabelString(5));
            assertTrue(Arrays.equals(new byte[]{0}, n.getLabel(6)));
            assertEquals("", n.getLabelString(6));
        }

        public void test_ctor_8label() throws TextParseException {
            Name n = new Name("a.b.c.d.e.f.g.");
            assertTrue(n.isAbsolute());
            assertFalse(n.isWild());
            assertEquals(8, n.labels());
            assertEquals(15, n.length());
            assertTrue(Arrays.equals(new byte[]{1, 97}, n.getLabel(0)));
            assertEquals("a", n.getLabelString(0));
            assertTrue(Arrays.equals(new byte[]{1, 98}, n.getLabel(1)));
            assertEquals("b", n.getLabelString(1));
            assertTrue(Arrays.equals(new byte[]{1, 99}, n.getLabel(2)));
            assertEquals("c", n.getLabelString(2));
            assertTrue(Arrays.equals(new byte[]{1, 100}, n.getLabel(3)));
            assertEquals("d", n.getLabelString(3));
            assertTrue(Arrays.equals(new byte[]{1, 101}, n.getLabel(4)));
            assertEquals("e", n.getLabelString(4));
            assertTrue(Arrays.equals(new byte[]{1, 102}, n.getLabel(5)));
            assertEquals("f", n.getLabelString(5));
            assertTrue(Arrays.equals(new byte[]{1, 103}, n.getLabel(6)));
            assertEquals("g", n.getLabelString(6));
            assertTrue(Arrays.equals(new byte[]{0}, n.getLabel(7)));
            assertEquals("", n.getLabelString(7));
        }

        public void test_ctor_abs_abs_origin() throws TextParseException {
            Name n = new Name("WWW.DnsJava.org.", this.m_abs_origin);
            assertTrue(n.isAbsolute());
            assertFalse(n.isWild());
            assertEquals(4, n.labels());
            assertEquals(17, n.length());
            assertTrue(Arrays.equals(new byte[]{3, 87, 87, 87}, n.getLabel(0)));
            assertEquals("WWW", n.getLabelString(0));
            assertTrue(Arrays.equals(new byte[]{7, 68, 110, 115, 74, 97, 118, 97}, n.getLabel(1)));
            assertEquals("DnsJava", n.getLabelString(1));
            assertTrue(Arrays.equals(new byte[]{3, 111, 114, 103}, n.getLabel(2)));
            assertEquals("org", n.getLabelString(2));
            assertTrue(Arrays.equals(new byte[]{0}, n.getLabel(3)));
            assertEquals("", n.getLabelString(3));
        }

        public void test_ctor_abs_rel_origin() throws TextParseException {
            Name n = new Name("WWW.DnsJava.org.", this.m_rel_origin);
            assertTrue(n.isAbsolute());
            assertFalse(n.isWild());
            assertEquals(4, n.labels());
            assertEquals(17, n.length());
            assertTrue(Arrays.equals(new byte[]{3, 87, 87, 87}, n.getLabel(0)));
            assertEquals("WWW", n.getLabelString(0));
            assertTrue(Arrays.equals(new byte[]{7, 68, 110, 115, 74, 97, 118, 97}, n.getLabel(1)));
            assertEquals("DnsJava", n.getLabelString(1));
            assertTrue(Arrays.equals(new byte[]{3, 111, 114, 103}, n.getLabel(2)));
            assertEquals("org", n.getLabelString(2));
            assertTrue(Arrays.equals(new byte[]{0}, n.getLabel(3)));
            assertEquals("", n.getLabelString(3));
        }

        public void test_ctor_rel_abs_origin() throws TextParseException {
            Name n = new Name("WWW.DnsJava", this.m_abs_origin);
            assertTrue(n.isAbsolute());
            assertFalse(n.isWild());
            assertEquals(4, n.labels());
            assertEquals(18, n.length());
            assertTrue(Arrays.equals(new byte[]{3, 87, 87, 87}, n.getLabel(0)));
            assertEquals("WWW", n.getLabelString(0));
            assertTrue(Arrays.equals(new byte[]{7, 68, 110, 115, 74, 97, 118, 97}, n.getLabel(1)));
            assertEquals("DnsJava", n.getLabelString(1));
            assertTrue(Arrays.equals(new byte[]{4, 79, 114, 105, 103}, n.getLabel(2)));
            assertEquals("Orig", n.getLabelString(2));
            assertTrue(Arrays.equals(new byte[]{0}, n.getLabel(3)));
            assertEquals("", n.getLabelString(3));
        }

        public void test_ctor_invalid_label() {
            try {
                new Name("junk..junk.");
                fail("TextParseException not thrown");
            } catch (TextParseException e) {
            }
        }

        public void test_ctor_max_label() throws TextParseException {
            Name n = new Name("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa.b.");
            assertTrue(n.isAbsolute());
            assertFalse(n.isWild());
            assertEquals(3, n.labels());
            assertEquals(67, n.length());
            assertTrue(Arrays.equals(new byte[]{63, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97}, n.getLabel(0)));
            assertEquals("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa", n.getLabelString(0));
            assertTrue(Arrays.equals(new byte[]{1, 98}, n.getLabel(1)));
            assertEquals("b", n.getLabelString(1));
            assertTrue(Arrays.equals(new byte[]{0}, n.getLabel(2)));
            assertEquals("", n.getLabelString(2));
        }

        public void test_ctor_toobig_label() {
            try {
                new Name("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa.b.");
                fail("TextParseException not thrown");
            } catch (TextParseException e) {
            }
        }

        public void test_ctor_max_length_rel() throws TextParseException {
            Name n = new Name("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa.bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb.ccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc.dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd");
            assertFalse(n.isAbsolute());
            assertFalse(n.isWild());
            assertEquals(4, n.labels());
            assertEquals(255, n.length());
        }

        public void test_ctor_max_length_abs() throws TextParseException {
            Name n = new Name("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa.bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb.ccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc.ddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd.");
            assertTrue(n.isAbsolute());
            assertFalse(n.isWild());
            assertEquals(5, n.labels());
            assertEquals(255, n.length());
        }

        public void test_ctor_escaped() throws TextParseException {
            Name n = new Name("ab\\123cd");
            assertFalse(n.isAbsolute());
            assertFalse(n.isWild());
            assertEquals(1, n.labels());
            assertEquals(6, n.length());
            assertTrue(Arrays.equals(new byte[]{5, 97, 98, 123, 99, 100}, n.getLabel(0)));
        }

        public void test_ctor_escaped_end() throws TextParseException {
            Name n = new Name("abcd\\123");
            assertFalse(n.isAbsolute());
            assertFalse(n.isWild());
            assertEquals(1, n.labels());
            assertEquals(6, n.length());
            assertTrue(Arrays.equals(new byte[]{5, 97, 98, 99, 100, 123}, n.getLabel(0)));
        }

        public void test_ctor_short_escaped() throws TextParseException {
            try {
                new Name("ab\\12cd");
                fail("TextParseException not throw");
            } catch (TextParseException e) {
            }
        }

        public void test_ctor_short_escaped_end() throws TextParseException {
            try {
                new Name("ab\\12");
                fail("TextParseException not throw");
            } catch (TextParseException e) {
            }
        }

        public void test_ctor_empty_escaped_end() throws TextParseException {
            try {
                new Name("ab\\");
                fail("TextParseException not throw");
            } catch (TextParseException e) {
            }
        }

        public void test_ctor_toobig_escaped() throws TextParseException {
            try {
                new Name("ab\\256cd");
                fail("TextParseException not throw");
            } catch (TextParseException e) {
            }
        }

        public void test_ctor_toobig_escaped_end() throws TextParseException {
            try {
                new Name("ab\\256");
                fail("TextParseException not throw");
            } catch (TextParseException e) {
            }
        }

        public void test_ctor_max_label_escaped() throws TextParseException {
            Name n = new Name("aaaa\\100aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa.b.");
            assertTrue(n.isAbsolute());
            assertFalse(n.isWild());
            assertEquals(3, n.labels());
            assertEquals(67, n.length());
            assertTrue(Arrays.equals(new byte[]{63, 97, 97, 97, 97, 100, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97}, n.getLabel(0)));
            assertTrue(Arrays.equals(new byte[]{1, 98}, n.getLabel(1)));
            assertEquals("b", n.getLabelString(1));
            assertTrue(Arrays.equals(new byte[]{0}, n.getLabel(2)));
            assertEquals("", n.getLabelString(2));
        }

        public void test_ctor_max_labels() throws TextParseException {
            StringBuffer sb = new StringBuffer();
            for (int i = 0; i < 127; i++) {
                sb.append("a.");
            }
            Name n = new Name(sb.toString());
            assertTrue(n.isAbsolute());
            assertFalse(n.isWild());
            assertEquals(128, n.labels());
            assertEquals(255, n.length());
            for (int i2 = 0; i2 < 127; i2++) {
                assertTrue(Arrays.equals(new byte[]{1, 97}, n.getLabel(i2)));
                assertEquals("a", n.getLabelString(i2));
            }
            assertTrue(Arrays.equals(new byte[]{0}, n.getLabel(127)));
            assertEquals("", n.getLabelString(127));
        }

        public void test_ctor_toobig_label_escaped_end() throws TextParseException {
            try {
                new Name("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\\090.b.");
                fail("TextParseException not thrown");
            } catch (TextParseException e) {
            }
        }

        public void test_ctor_toobig_label_escaped() throws TextParseException {
            try {
                new Name("aaaaaaaaaaaaaaaaaaaaaaaaaaaa\\001aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa.b.");
                fail("TextParseException not thrown");
            } catch (TextParseException e) {
            }
        }

        public void test_fromString() throws TextParseException {
            assertEquals(new Name("WWW.DnsJava", this.m_abs_origin), Name.fromString("WWW.DnsJava", this.m_abs_origin));
        }

        public void test_fromString_at() throws TextParseException {
            assertSame(this.m_rel_origin, Name.fromString("@", this.m_rel_origin));
        }

        public void test_fromString_dot() throws TextParseException {
            assertSame(Name.root, Name.fromString("."));
        }

        public void test_fromConstantString() throws TextParseException {
            assertEquals(new Name("WWW.DnsJava.org."), Name.fromConstantString("WWW.DnsJava.org."));
        }

        public void test_fromConstantString_invalid() {
            try {
                Name.fromConstantString("junk..junk");
                fail("IllegalArgumentException not thrown");
            } catch (IllegalArgumentException e) {
            }
        }
    }

    public static class Test_DNSInput_init extends TestCase {
        public void test_basic() throws IOException, TextParseException, WireParseException {
            assertEquals(Name.fromString("Www.DnsJava.org."), new Name(new byte[]{3, 87, 119, 119, 7, 68, 110, 115, 74, 97, 118, 97, 3, 111, 114, 103, 0}));
        }

        public void test_incomplete() throws IOException {
            try {
                new Name(new byte[]{3, 87, 119, 119});
                fail("WireParseException not thrown");
            } catch (WireParseException e) {
            }
        }

        public void test_root() throws WireParseException {
            assertEquals(Name.root, new Name(new DNSInput(new byte[]{0})));
        }

        public void test_invalid_length() throws IOException {
            try {
                new Name(new byte[]{4, 87, 119, 119});
                fail("WireParseException not thrown");
            } catch (WireParseException e) {
            }
        }

        public void test_max_label_length() throws TextParseException, WireParseException {
            assertEquals(Name.fromString("bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb."), new Name(new DNSInput(new byte[]{63, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 0})));
        }

        public void test_max_name() throws TextParseException, WireParseException {
            assertEquals(new Name("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa.bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb.ccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc.ddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd."), new Name(new DNSInput(new byte[]{63, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 63, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 63, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, HttpConstants.EQUALS, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 0})));
        }

        public void test_toolong_name() throws TextParseException, WireParseException {
            try {
                new Name(new DNSInput(new byte[]{63, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 63, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 63, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 62, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 0}));
                fail("WireParseException not thrown");
            } catch (WireParseException e) {
            }
        }

        public void test_max_labels() throws TextParseException, WireParseException {
            Name e = Name.fromString("a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.a.");
            Name n = new Name(new DNSInput(new byte[]{1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 0}));
            assertEquals(128, n.labels());
            assertEquals(e, n);
        }

        public void test_toomany_labels() throws TextParseException, WireParseException {
            try {
                new Name(new DNSInput(new byte[]{1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 1, 97, 0}));
                fail("WireParseException not thrown");
            } catch (WireParseException e) {
            }
        }

        public void test_basic_compression() throws TextParseException, WireParseException {
            Name e = Name.fromString("abc.");
            DNSInput in = new DNSInput(new byte[]{10, 3, 97, 98, 99, 0, -64, 1});
            in.jump(6);
            Options.set("verbosecompression");
            Name n = new Name(in);
            Options.unset("verbosecompression");
            assertEquals(e, n);
        }

        public void test_two_pointer_compression() throws TextParseException, WireParseException {
            Name e = Name.fromString("abc.");
            DNSInput in = new DNSInput(new byte[]{10, 3, 97, 98, 99, 0, -64, 1, -64, 6});
            in.jump(8);
            assertEquals(e, new Name(in));
        }

        public void test_two_part_compression() throws TextParseException, WireParseException {
            Name e = Name.fromString("B.abc.");
            DNSInput in = new DNSInput(new byte[]{10, 3, 97, 98, 99, 0, 1, 66, -64, 1});
            in.jump(6);
            assertEquals(e, new Name(in));
        }

        public void test_long_jump_compression() throws TextParseException, WireParseException {
            byte[] raw = new byte[opencv_cudaimgproc.COLOR_BayerGR2GRAY_MHT];
            // fill-array-data instruction
            raw[0] = 12;
            raw[1] = 12;
            raw[2] = 12;
            raw[3] = 12;
            raw[4] = 12;
            raw[5] = 12;
            raw[6] = 12;
            raw[7] = 12;
            raw[8] = 12;
            raw[9] = 12;
            raw[10] = 12;
            raw[11] = 12;
            raw[12] = 12;
            raw[13] = 12;
            raw[14] = 12;
            raw[15] = 12;
            raw[16] = 12;
            raw[17] = 12;
            raw[18] = 12;
            raw[19] = 12;
            raw[20] = 12;
            raw[21] = 12;
            raw[22] = 12;
            raw[23] = 12;
            raw[24] = 12;
            raw[25] = 12;
            raw[26] = 12;
            raw[27] = 12;
            raw[28] = 12;
            raw[29] = 12;
            raw[30] = 12;
            raw[31] = 12;
            raw[32] = 12;
            raw[33] = 12;
            raw[34] = 12;
            raw[35] = 12;
            raw[36] = 12;
            raw[37] = 12;
            raw[38] = 12;
            raw[39] = 12;
            raw[40] = 12;
            raw[41] = 12;
            raw[42] = 12;
            raw[43] = 12;
            raw[44] = 12;
            raw[45] = 12;
            raw[46] = 12;
            raw[47] = 12;
            raw[48] = 12;
            raw[49] = 12;
            raw[50] = 12;
            raw[51] = 12;
            raw[52] = 12;
            raw[53] = 12;
            raw[54] = 12;
            raw[55] = 12;
            raw[56] = 12;
            raw[57] = 12;
            raw[58] = 12;
            raw[59] = 12;
            raw[60] = 12;
            raw[61] = 12;
            raw[62] = 12;
            raw[63] = 12;
            raw[64] = 12;
            raw[65] = 12;
            raw[66] = 12;
            raw[67] = 12;
            raw[68] = 12;
            raw[69] = 12;
            raw[70] = 12;
            raw[71] = 12;
            raw[72] = 12;
            raw[73] = 12;
            raw[74] = 12;
            raw[75] = 12;
            raw[76] = 12;
            raw[77] = 12;
            raw[78] = 12;
            raw[79] = 12;
            raw[80] = 12;
            raw[81] = 12;
            raw[82] = 12;
            raw[83] = 12;
            raw[84] = 12;
            raw[85] = 12;
            raw[86] = 12;
            raw[87] = 12;
            raw[88] = 12;
            raw[89] = 12;
            raw[90] = 12;
            raw[91] = 12;
            raw[92] = 12;
            raw[93] = 12;
            raw[94] = 12;
            raw[95] = 12;
            raw[96] = 12;
            raw[97] = 12;
            raw[98] = 12;
            raw[99] = 12;
            raw[100] = 12;
            raw[101] = 12;
            raw[102] = 12;
            raw[103] = 12;
            raw[104] = 12;
            raw[105] = 12;
            raw[106] = 12;
            raw[107] = 12;
            raw[108] = 12;
            raw[109] = 12;
            raw[110] = 12;
            raw[111] = 12;
            raw[112] = 12;
            raw[113] = 12;
            raw[114] = 12;
            raw[115] = 12;
            raw[116] = 12;
            raw[117] = 12;
            raw[118] = 12;
            raw[119] = 12;
            raw[120] = 12;
            raw[121] = 12;
            raw[122] = 12;
            raw[123] = 12;
            raw[124] = 12;
            raw[125] = 12;
            raw[126] = 12;
            raw[127] = 12;
            raw[128] = 12;
            raw[129] = 12;
            raw[130] = 12;
            raw[131] = 12;
            raw[132] = 12;
            raw[133] = 12;
            raw[134] = 12;
            raw[135] = 12;
            raw[136] = 12;
            raw[137] = 12;
            raw[138] = 12;
            raw[139] = 12;
            raw[140] = 12;
            raw[141] = 12;
            raw[142] = 12;
            raw[143] = 12;
            raw[144] = 12;
            raw[145] = 12;
            raw[146] = 12;
            raw[147] = 12;
            raw[148] = 12;
            raw[149] = 12;
            raw[150] = 12;
            raw[151] = 12;
            raw[152] = 12;
            raw[153] = 12;
            raw[154] = 12;
            raw[155] = 12;
            raw[156] = 12;
            raw[157] = 12;
            raw[158] = 12;
            raw[159] = 12;
            raw[160] = 12;
            raw[161] = 12;
            raw[162] = 12;
            raw[163] = 12;
            raw[164] = 12;
            raw[165] = 12;
            raw[166] = 12;
            raw[167] = 12;
            raw[168] = 12;
            raw[169] = 12;
            raw[170] = 12;
            raw[171] = 12;
            raw[172] = 12;
            raw[173] = 12;
            raw[174] = 12;
            raw[175] = 12;
            raw[176] = 12;
            raw[177] = 12;
            raw[178] = 12;
            raw[179] = 12;
            raw[180] = 12;
            raw[181] = 12;
            raw[182] = 12;
            raw[183] = 12;
            raw[184] = 12;
            raw[185] = 12;
            raw[186] = 12;
            raw[187] = 12;
            raw[188] = 12;
            raw[189] = 12;
            raw[190] = 12;
            raw[191] = 12;
            raw[192] = 12;
            raw[193] = 12;
            raw[194] = 12;
            raw[195] = 12;
            raw[196] = 12;
            raw[197] = 12;
            raw[198] = 12;
            raw[199] = 12;
            raw[200] = 12;
            raw[201] = 12;
            raw[202] = 12;
            raw[203] = 12;
            raw[204] = 12;
            raw[205] = 12;
            raw[206] = 12;
            raw[207] = 12;
            raw[208] = 12;
            raw[209] = 12;
            raw[210] = 12;
            raw[211] = 12;
            raw[212] = 12;
            raw[213] = 12;
            raw[214] = 12;
            raw[215] = 12;
            raw[216] = 12;
            raw[217] = 12;
            raw[218] = 12;
            raw[219] = 12;
            raw[220] = 12;
            raw[221] = 12;
            raw[222] = 12;
            raw[223] = 12;
            raw[224] = 12;
            raw[225] = 12;
            raw[226] = 12;
            raw[227] = 12;
            raw[228] = 12;
            raw[229] = 12;
            raw[230] = 12;
            raw[231] = 12;
            raw[232] = 12;
            raw[233] = 12;
            raw[234] = 12;
            raw[235] = 12;
            raw[236] = 12;
            raw[237] = 12;
            raw[238] = 12;
            raw[239] = 12;
            raw[240] = 12;
            raw[241] = 12;
            raw[242] = 12;
            raw[243] = 12;
            raw[244] = 12;
            raw[245] = 12;
            raw[246] = 12;
            raw[247] = 12;
            raw[248] = 12;
            raw[249] = 12;
            raw[250] = 12;
            raw[251] = 12;
            raw[252] = 12;
            raw[253] = 12;
            raw[254] = 12;
            raw[255] = 12;
            raw[256] = 3;
            raw[257] = 97;
            raw[258] = 98;
            raw[259] = 99;
            raw[260] = 0;
            raw[261] = -63;
            raw[262] = 0;
            Name e = Name.fromString("abc.");
            DNSInput in = new DNSInput(raw);
            in.jump(opencv_cudaimgproc.COLOR_BayerGB2GRAY_MHT);
            assertEquals(e, new Name(in));
        }

        public void test_bad_compression() throws TextParseException, WireParseException {
            try {
                new Name(new DNSInput(new byte[]{-64, 2, 0}));
                fail("WireParseException not thrown");
            } catch (WireParseException e) {
            }
        }

        public void test_basic_compression_state_restore() throws TextParseException, WireParseException {
            Name e = Name.fromString("abc.");
            Name e2 = Name.fromString("def.");
            DNSInput in = new DNSInput(new byte[]{10, 3, 97, 98, 99, 0, -64, 1, 3, 100, 101, 102, 0});
            in.jump(6);
            assertEquals(e, new Name(in));
            assertEquals(e2, new Name(in));
        }

        public void test_two_part_compression_state_restore() throws TextParseException, WireParseException {
            Name e = Name.fromString("B.abc.");
            Name e2 = Name.fromString("def.");
            DNSInput in = new DNSInput(new byte[]{10, 3, 97, 98, 99, 0, 1, 66, -64, 1, 3, 100, 101, 102, 0});
            in.jump(6);
            assertEquals(e, new Name(in));
            assertEquals(e2, new Name(in));
        }
    }

    public void test_init_from_name() throws TextParseException {
        assertEquals(new Name("B.c.d."), new Name(new Name("A.B.c.d."), 1));
    }

    public void test_init_from_name_root() throws TextParseException {
        assertEquals(Name.root, new Name(new Name("A.B.c.d."), 4));
    }

    public void test_init_from_name_empty() throws TextParseException {
        Name n2 = new Name(new Name("A.B.c.d."), 5);
        assertFalse(n2.isAbsolute());
        assertFalse(n2.isWild());
        assertEquals(0, n2.labels());
        assertEquals(0, n2.length());
    }

    public void test_concatenate_basic() throws NameTooLongException, TextParseException {
        assertEquals(Name.fromString("A.B.c.d."), Name.concatenate(Name.fromString("A.B"), Name.fromString("c.d.")));
    }

    public void test_concatenate_abs_prefix() throws NameTooLongException, TextParseException {
        assertEquals(Name.fromString("A.B."), Name.concatenate(Name.fromString("A.B."), Name.fromString("c.d.")));
    }

    public void test_concatenate_too_long() throws TextParseException {
        try {
            Name.concatenate(Name.fromString("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa.bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb"), Name.fromString("ccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc.ddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd."));
            fail("NameTooLongException not thrown");
        } catch (NameTooLongException e) {
        }
    }

    public void test_relativize() throws TextParseException {
        assertEquals(Name.fromString("a.b"), Name.fromString("a.b.c.").relativize(Name.fromString("c.")));
    }

    public void test_relativize_null_origin() throws TextParseException {
        Name sub = Name.fromString("a.b.c.");
        assertEquals(sub, sub.relativize((Name) null));
    }

    public void test_relativize_disjoint() throws TextParseException {
        Name sub = Name.fromString("a.b.c.");
        assertEquals(sub, sub.relativize(Name.fromString("e.f.")));
    }

    public void test_relativize_root() throws TextParseException {
        assertEquals(Name.fromString("a.b.c"), Name.fromString("a.b.c.").relativize(Name.fromString(".")));
    }

    public void test_wild() throws TextParseException {
        assertEquals(Name.fromString("*.b.c."), Name.fromString("a.b.c.").wild(1));
    }

    public void test_wild_abs() throws TextParseException {
        assertEquals(Name.fromString("*."), Name.fromString("a.b.c.").wild(3));
    }

    public void test_wild_toobig() throws TextParseException {
        try {
            Name.fromString("a.b.c.").wild(4);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e) {
        }
    }

    public void test_wild_toosmall() throws TextParseException {
        try {
            Name.fromString("a.b.c.").wild(0);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e) {
        }
    }

    public void test_fromDNAME() throws NameTooLongException, TextParseException {
        DNAMERecord dnr = new DNAMERecord(new Name("the.owner."), 1, 43981, new Name("the.alias."));
        assertEquals(new Name("sub.the.alias."), new Name("sub.the.owner.").fromDNAME(dnr));
    }

    public void test_fromDNAME_toobig() throws NameTooLongException, TextParseException {
        try {
            new Name("ddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd.the.owner.").fromDNAME(new DNAMERecord(new Name("the.owner."), 1, 43981, new Name("the.aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa.bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb.ccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc.")));
            fail("NameTooLongException not thrown");
        } catch (NameTooLongException e) {
        }
    }

    public void test_fromDNAME_disjoint() throws NameTooLongException, TextParseException {
        assertNull(new Name("sub.the.other").fromDNAME(new DNAMERecord(new Name("the.owner."), 1, 43981, new Name("the.alias."))));
    }

    public void test_subdomain_abs() throws TextParseException {
        Name dom = new Name("the.domain.");
        Name sub = new Name("sub.of.the.domain.");
        assertTrue(sub.subdomain(dom));
        assertFalse(dom.subdomain(sub));
    }

    public void test_subdomain_rel() throws TextParseException {
        Name dom = new Name("the.domain");
        Name sub = new Name("sub.of.the.domain");
        assertTrue(sub.subdomain(dom));
        assertFalse(dom.subdomain(sub));
    }

    public void test_subdomain_equal() throws TextParseException {
        Name dom = new Name("the.domain");
        Name sub = new Name("the.domain");
        assertTrue(sub.subdomain(dom));
        assertTrue(dom.subdomain(sub));
    }

    public void test_toString_abs() throws TextParseException {
        assertEquals("This.Is.My.Absolute.Name.", new Name("This.Is.My.Absolute.Name.").toString());
    }

    public void test_toString_rel() throws TextParseException {
        assertEquals("This.Is.My.Relative.Name", new Name("This.Is.My.Relative.Name").toString());
    }

    public void test_toString_at() throws TextParseException {
        assertEquals("@", new Name("@", (Name) null).toString());
    }

    public void test_toString_root() throws TextParseException {
        assertEquals(".", Name.root.toString());
    }

    public void test_toString_wild() throws TextParseException {
        assertEquals("*.A.b.c.e", new Name("*.A.b.c.e").toString());
    }

    public void test_toString_escaped() throws TextParseException {
        assertEquals("my.escaped.junk\\128.label.", new Name("my.escaped.junk\\128.label.").toString());
    }

    public void test_toString_special_char() throws TextParseException, WireParseException {
        assertEquals("\\\".\\(.\\).\\..\\;.\\\\.\\@.\\$.", new Name(new DNSInput(new byte[]{1, 34, 1, 40, 1, 41, 1, 46, 1, HttpConstants.SEMICOLON, 1, 92, 1, SignedBytes.MAX_POWER_OF_TWO, 1, ErrorCodes.CLIENT_CONNECTION_DISRUPTED, 0})).toString());
    }

    public static class Test_toWire extends TestCase {
        public void test_rel() throws TextParseException {
            try {
                new Name("A.Relative.Name").toWire(new DNSOutput(), (Compression) null);
                fail("IllegalArgumentException not thrown");
            } catch (IllegalArgumentException e) {
            }
        }

        public void test_null_Compression() throws TextParseException {
            Name n = new Name("A.Basic.Name.");
            DNSOutput o = new DNSOutput();
            n.toWire(o, (Compression) null);
            assertTrue(Arrays.equals(new byte[]{1, 65, 5, 66, 97, 115, 105, 99, 4, 78, 97, 109, 101, 0}, o.toByteArray()));
        }

        public void test_empty_Compression() throws TextParseException {
            Name n = new Name("A.Basic.Name.");
            Compression c = new Compression();
            DNSOutput o = new DNSOutput();
            n.toWire(o, c);
            assertTrue(Arrays.equals(new byte[]{1, 65, 5, 66, 97, 115, 105, 99, 4, 78, 97, 109, 101, 0}, o.toByteArray()));
            assertEquals(0, c.get(n));
        }

        public void test_with_exact_Compression() throws TextParseException {
            Name n = new Name("A.Basic.Name.");
            Compression c = new Compression();
            c.add(256, n);
            DNSOutput o = new DNSOutput();
            n.toWire(o, c);
            assertTrue(Arrays.equals(new byte[]{-63, 0}, o.toByteArray()));
            assertEquals(256, c.get(n));
        }

        public void test_with_partial_Compression() throws TextParseException {
            Name d = new Name("Basic.Name.");
            Name n = new Name("A.Basic.Name.");
            Compression c = new Compression();
            c.add(257, d);
            DNSOutput o = new DNSOutput();
            n.toWire(o, c);
            assertTrue(Arrays.equals(new byte[]{1, 65, -63, 1}, o.toByteArray()));
            assertEquals(257, c.get(d));
            assertEquals(0, c.get(n));
        }

        public void test_0arg_rel() throws TextParseException {
            try {
                new Name("A.Relative.Name").toWire();
                fail("IllegalArgumentException not thrown");
            } catch (IllegalArgumentException e) {
            }
        }

        public void test_0arg() throws TextParseException {
            assertTrue(Arrays.equals(new byte[]{1, 65, 5, 66, 97, 115, 105, 99, 4, 78, 97, 109, 101, 0}, new Name("A.Basic.Name.").toWire()));
        }

        public void test_root() {
            byte[] bArr = {0};
            assertTrue(Arrays.equals(bArr, Name.root.toWire()));
        }

        public void test_3arg() throws TextParseException {
            Name d = new Name("Basic.Name.");
            Name n = new Name("A.Basic.Name.");
            Compression c = new Compression();
            c.add(257, d);
            DNSOutput o = new DNSOutput();
            n.toWire(o, c, false);
            assertTrue(Arrays.equals(new byte[]{1, 65, -63, 1}, o.toByteArray()));
            assertEquals(257, c.get(d));
            assertEquals(0, c.get(n));
        }
    }

    public static class Test_toWireCanonical extends TestCase {
        public void test_basic() throws TextParseException {
            Name n = new Name("A.Basic.Name.");
            DNSOutput o = new DNSOutput();
            n.toWireCanonical(o);
            assertTrue(Arrays.equals(new byte[]{1, 97, 5, 98, 97, 115, 105, 99, 4, 110, 97, 109, 101, 0}, o.toByteArray()));
        }

        public void test_0arg() throws TextParseException {
            assertTrue(Arrays.equals(new byte[]{1, 97, 5, 98, 97, 115, 105, 99, 4, 110, 97, 109, 101, 0}, new Name("A.Basic.Name.").toWireCanonical()));
        }

        public void test_root() {
            byte[] bArr = {0};
            assertTrue(Arrays.equals(bArr, Name.root.toWireCanonical()));
        }

        public void test_empty() throws TextParseException {
            assertTrue(Arrays.equals(new byte[0], new Name("@", (Name) null).toWireCanonical()));
        }

        public void test_3arg() throws TextParseException {
            Name d = new Name("Basic.Name.");
            Name n = new Name("A.Basic.Name.");
            Compression c = new Compression();
            c.add(257, d);
            DNSOutput o = new DNSOutput();
            n.toWire(o, c, true);
            assertTrue(Arrays.equals(new byte[]{1, 97, 5, 98, 97, 115, 105, 99, 4, 110, 97, 109, 101, 0}, o.toByteArray()));
            assertEquals(257, c.get(d));
            assertEquals(-1, c.get(n));
        }
    }

    public static class Test_equals extends TestCase {
        public void test_same() throws TextParseException {
            Name n = new Name("A.Name.");
            assertTrue(n.equals(n));
        }

        public void test_null() throws TextParseException {
            assertFalse(new Name("A.Name.").equals((Object) null));
        }

        public void test_notName() throws TextParseException {
            assertFalse(new Name("A.Name.").equals(new Object()));
        }

        public void test_abs() throws TextParseException {
            Name n = new Name("A.Name.");
            Name n2 = new Name("a.name.");
            assertTrue(n.equals(n2));
            assertTrue(n2.equals(n));
        }

        public void test_rel() throws TextParseException {
            Name n1 = new Name("A.Relative.Name");
            Name n2 = new Name("a.relative.name");
            assertTrue(n1.equals(n2));
            assertTrue(n2.equals(n1));
        }

        public void test_mixed() throws TextParseException {
            Name n1 = new Name("A.Name");
            Name n2 = new Name("a.name.");
            assertFalse(n1.equals(n2));
            assertFalse(n2.equals(n1));
        }

        public void test_weird() throws TextParseException {
            Name n1 = new Name("ab.c");
            Name n2 = new Name("abc.");
            assertFalse(n1.equals(n2));
            assertFalse(n2.equals(n1));
        }
    }

    public static class Test_compareTo extends TestCase {
        public void test_notName() throws TextParseException {
            try {
                new Name("A.Name").compareTo(new Object());
                fail("ClassCastException not thrown");
            } catch (ClassCastException e) {
            }
        }

        public void test_same() throws TextParseException {
            Name n = new Name("A.Name");
            assertEquals(0, n.compareTo(n));
        }

        public void test_equal() throws TextParseException {
            Name n1 = new Name("A.Name.");
            Name n2 = new Name("a.name.");
            assertEquals(0, n1.compareTo(n2));
            assertEquals(0, n2.compareTo(n1));
        }

        public void test_close() throws TextParseException {
            Name n1 = new Name("a.name");
            Name n2 = new Name("a.name.");
            boolean z = false;
            assertTrue(n1.compareTo(n2) > 0);
            if (n2.compareTo(n1) < 0) {
                z = true;
            }
            assertTrue(z);
        }

        public void test_disjoint() throws TextParseException {
            Name n1 = new Name("b");
            Name n2 = new Name("c");
            boolean z = false;
            assertTrue(n1.compareTo(n2) < 0);
            if (n2.compareTo(n1) > 0) {
                z = true;
            }
            assertTrue(z);
        }

        public void test_label_prefix() throws TextParseException {
            Name n1 = new Name("thisIs.a.");
            Name n2 = new Name("thisIsGreater.a.");
            boolean z = false;
            assertTrue(n1.compareTo(n2) < 0);
            if (n2.compareTo(n1) > 0) {
                z = true;
            }
            assertTrue(z);
        }

        public void test_more_labels() throws TextParseException {
            Name n1 = new Name("c.b.a.");
            Name n2 = new Name("d.c.b.a.");
            boolean z = false;
            assertTrue(n1.compareTo(n2) < 0);
            if (n2.compareTo(n1) > 0) {
                z = true;
            }
            assertTrue(z);
        }
    }

    public static Test suite() {
        Class cls;
        Class cls2;
        Class cls3;
        Class cls4;
        Class cls5;
        Class cls6;
        Class cls7;
        TestSuite s = new TestSuite();
        if (class$org$xbill$DNS$NameTest$Test_String_init == null) {
            cls = class$("org.xbill.DNS.NameTest$Test_String_init");
            class$org$xbill$DNS$NameTest$Test_String_init = cls;
        } else {
            cls = class$org$xbill$DNS$NameTest$Test_String_init;
        }
        s.addTestSuite(cls);
        if (class$org$xbill$DNS$NameTest$Test_DNSInput_init == null) {
            cls2 = class$("org.xbill.DNS.NameTest$Test_DNSInput_init");
            class$org$xbill$DNS$NameTest$Test_DNSInput_init = cls2;
        } else {
            cls2 = class$org$xbill$DNS$NameTest$Test_DNSInput_init;
        }
        s.addTestSuite(cls2);
        if (class$org$xbill$DNS$NameTest == null) {
            cls3 = class$("org.xbill.DNS.NameTest");
            class$org$xbill$DNS$NameTest = cls3;
        } else {
            cls3 = class$org$xbill$DNS$NameTest;
        }
        s.addTestSuite(cls3);
        if (class$org$xbill$DNS$NameTest$Test_toWire == null) {
            cls4 = class$("org.xbill.DNS.NameTest$Test_toWire");
            class$org$xbill$DNS$NameTest$Test_toWire = cls4;
        } else {
            cls4 = class$org$xbill$DNS$NameTest$Test_toWire;
        }
        s.addTestSuite(cls4);
        if (class$org$xbill$DNS$NameTest$Test_toWireCanonical == null) {
            cls5 = class$("org.xbill.DNS.NameTest$Test_toWireCanonical");
            class$org$xbill$DNS$NameTest$Test_toWireCanonical = cls5;
        } else {
            cls5 = class$org$xbill$DNS$NameTest$Test_toWireCanonical;
        }
        s.addTestSuite(cls5);
        if (class$org$xbill$DNS$NameTest$Test_equals == null) {
            cls6 = class$("org.xbill.DNS.NameTest$Test_equals");
            class$org$xbill$DNS$NameTest$Test_equals = cls6;
        } else {
            cls6 = class$org$xbill$DNS$NameTest$Test_equals;
        }
        s.addTestSuite(cls6);
        if (class$org$xbill$DNS$NameTest$Test_compareTo == null) {
            cls7 = class$("org.xbill.DNS.NameTest$Test_compareTo");
            class$org$xbill$DNS$NameTest$Test_compareTo = cls7;
        } else {
            cls7 = class$org$xbill$DNS$NameTest$Test_compareTo;
        }
        s.addTestSuite(cls7);
        return s;
    }

    static /* synthetic */ Class class$(String x0) {
        try {
            return Class.forName(x0);
        } catch (ClassNotFoundException x1) {
            throw new NoClassDefFoundError().initCause(x1);
        }
    }
}
