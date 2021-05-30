package org.xbill.DNS;

import com.google.common.base.Ascii;
import java.io.BufferedInputStream;
import java.io.ByteArrayInputStream;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.util.Arrays;
import junit.framework.TestCase;
import org.xbill.DNS.Tokenizer;

public class TokenizerTest extends TestCase {
    private Tokenizer m_t;

    /* access modifiers changed from: protected */
    public void setUp() {
        this.m_t = null;
    }

    private void assertEquals(byte[] exp, byte[] act) {
        assertTrue(Arrays.equals(exp, act));
    }

    public void test_get() throws IOException {
        this.m_t = new Tokenizer((InputStream) new BufferedInputStream(new ByteArrayInputStream("AnIdentifier \"a quoted \\\" string\"\r\n; this is \"my\"\t(comment)\nanotherIdentifier (\ramultilineIdentifier\n)".getBytes())));
        Tokenizer.Token tt = this.m_t.get(true, true);
        assertEquals(3, tt.type);
        assertTrue(tt.isString());
        assertFalse(tt.isEOL());
        assertEquals("AnIdentifier", tt.value);
        Tokenizer.Token tt2 = this.m_t.get(true, true);
        assertEquals(2, tt2.type);
        assertFalse(tt2.isString());
        assertFalse(tt2.isEOL());
        assertNull(tt2.value);
        Tokenizer.Token tt3 = this.m_t.get(true, true);
        assertEquals(4, tt3.type);
        assertTrue(tt3.isString());
        assertFalse(tt3.isEOL());
        assertEquals("a quoted \\\" string", tt3.value);
        Tokenizer.Token tt4 = this.m_t.get(true, true);
        assertEquals(1, tt4.type);
        assertFalse(tt4.isString());
        assertTrue(tt4.isEOL());
        assertNull(tt4.value);
        Tokenizer.Token tt5 = this.m_t.get(true, true);
        assertEquals(5, tt5.type);
        assertFalse(tt5.isString());
        assertFalse(tt5.isEOL());
        assertEquals(" this is \"my\"\t(comment)", tt5.value);
        Tokenizer.Token tt6 = this.m_t.get(true, true);
        assertEquals(1, tt6.type);
        assertFalse(tt6.isString());
        assertTrue(tt6.isEOL());
        assertNull(tt6.value);
        Tokenizer.Token tt7 = this.m_t.get(true, true);
        assertEquals(3, tt7.type);
        assertTrue(tt7.isString());
        assertFalse(tt7.isEOL());
        assertEquals("anotherIdentifier", tt7.value);
        assertEquals(2, this.m_t.get(true, true).type);
        Tokenizer.Token tt8 = this.m_t.get(true, true);
        assertEquals(3, tt8.type);
        assertTrue(tt8.isString());
        assertFalse(tt8.isEOL());
        assertEquals("amultilineIdentifier", tt8.value);
        assertEquals(2, this.m_t.get(true, true).type);
        Tokenizer.Token tt9 = this.m_t.get(true, true);
        assertEquals(0, tt9.type);
        assertFalse(tt9.isString());
        assertTrue(tt9.isEOL());
        assertNull(tt9.value);
        Tokenizer.Token tt10 = this.m_t.get(true, true);
        assertEquals(0, tt10.type);
        assertFalse(tt10.isString());
        assertTrue(tt10.isEOL());
        assertNull(tt10.value);
        this.m_t = new Tokenizer("onlyOneIdentifier");
        Tokenizer.Token tt11 = this.m_t.get();
        assertEquals(3, tt11.type);
        assertEquals("onlyOneIdentifier", tt11.value);
        this.m_t = new Tokenizer("identifier ;");
        assertEquals("identifier", this.m_t.get().value);
        assertEquals(0, this.m_t.get().type);
        this.m_t = new Tokenizer("identifier \nidentifier2; junk comment");
        Tokenizer.Token tt12 = this.m_t.get(true, true);
        assertEquals(3, tt12.type);
        assertEquals("identifier", tt12.value);
        this.m_t.unget();
        Tokenizer.Token tt13 = this.m_t.get(true, true);
        assertEquals(3, tt13.type);
        assertEquals("identifier", tt13.value);
        assertEquals(2, this.m_t.get(true, true).type);
        this.m_t.unget();
        assertEquals(2, this.m_t.get(true, true).type);
        assertEquals(1, this.m_t.get(true, true).type);
        this.m_t.unget();
        assertEquals(1, this.m_t.get(true, true).type);
        Tokenizer.Token tt14 = this.m_t.get(true, true);
        assertEquals(3, tt14.type);
        assertEquals("identifier2", tt14.value);
        Tokenizer.Token tt15 = this.m_t.get(true, true);
        assertEquals(5, tt15.type);
        assertEquals(" junk comment", tt15.value);
        this.m_t.unget();
        Tokenizer.Token tt16 = this.m_t.get(true, true);
        assertEquals(5, tt16.type);
        assertEquals(" junk comment", tt16.value);
        assertEquals(0, this.m_t.get(true, true).type);
        this.m_t = new Tokenizer("identifier ( junk ; comment\n )");
        assertEquals(3, this.m_t.get().type);
        assertEquals(3, this.m_t.get().type);
        assertEquals(0, this.m_t.get().type);
    }

    public void test_get_invalid() throws IOException {
        this.m_t = new Tokenizer("(this ;");
        this.m_t.get();
        try {
            this.m_t.get();
            fail("TextParseException not thrown");
        } catch (TextParseException e) {
        }
        this.m_t = new Tokenizer("\"bad");
        try {
            this.m_t.get();
            fail("TextParseException not thrown");
        } catch (TextParseException e2) {
        }
        this.m_t = new Tokenizer(")");
        try {
            this.m_t.get();
            fail("TextParseException not thrown");
        } catch (TextParseException e3) {
        }
        this.m_t = new Tokenizer("\\");
        try {
            this.m_t.get();
            fail("TextParseException not thrown");
        } catch (TextParseException e4) {
        }
        this.m_t = new Tokenizer("\"\n");
        try {
            this.m_t.get();
            fail("TextParseException not thrown");
        } catch (TextParseException e5) {
        }
    }

    public void test_File_input() throws IOException {
        File tmp = File.createTempFile("dnsjava", "tmp");
        try {
            FileWriter fw = new FileWriter(tmp);
            fw.write("file\ninput; test");
            fw.close();
            this.m_t = new Tokenizer(tmp);
            Tokenizer.Token tt = this.m_t.get();
            assertEquals(3, tt.type);
            assertEquals(HttpPostBodyUtil.FILE, tt.value);
            assertEquals(1, this.m_t.get().type);
            Tokenizer.Token tt2 = this.m_t.get();
            assertEquals(3, tt2.type);
            assertEquals("input", tt2.value);
            Tokenizer.Token tt3 = this.m_t.get(false, true);
            assertEquals(5, tt3.type);
            assertEquals(" test", tt3.value);
            this.m_t.close();
        } finally {
            tmp.delete();
        }
    }

    public void test_unwanted_comment() throws IOException {
        this.m_t = new Tokenizer("; this whole thing is a comment\n");
        assertEquals(1, this.m_t.get().type);
    }

    public void test_unwanted_ungotten_whitespace() throws IOException {
        this.m_t = new Tokenizer(" ");
        Tokenizer.Token token = this.m_t.get(true, true);
        this.m_t.unget();
        assertEquals(0, this.m_t.get().type);
    }

    public void test_unwanted_ungotten_comment() throws IOException {
        this.m_t = new Tokenizer("; this whole thing is a comment");
        Tokenizer.Token token = this.m_t.get(true, true);
        this.m_t.unget();
        assertEquals(0, this.m_t.get().type);
    }

    public void test_empty_string() throws IOException {
        this.m_t = new Tokenizer("");
        assertEquals(0, this.m_t.get().type);
        this.m_t = new Tokenizer(" ");
        assertEquals(0, this.m_t.get().type);
    }

    public void test_multiple_ungets() throws IOException {
        this.m_t = new Tokenizer("a simple one");
        Tokenizer.Token token = this.m_t.get();
        this.m_t.unget();
        try {
            this.m_t.unget();
            fail("IllegalStateException not thrown");
        } catch (IllegalStateException e) {
        }
    }

    public void test_getString() throws IOException {
        this.m_t = new Tokenizer("just_an_identifier");
        assertEquals("just_an_identifier", this.m_t.getString());
        this.m_t = new Tokenizer("\"just a string\"");
        assertEquals("just a string", this.m_t.getString());
        this.m_t = new Tokenizer("; just a comment");
        try {
            String out = this.m_t.getString();
            fail("TextParseException not thrown");
        } catch (TextParseException e) {
        }
    }

    public void test_getIdentifier() throws IOException {
        this.m_t = new Tokenizer("just_an_identifier");
        assertEquals("just_an_identifier", this.m_t.getIdentifier());
        this.m_t = new Tokenizer("\"just a string\"");
        try {
            this.m_t.getIdentifier();
            fail("TextParseException not thrown");
        } catch (TextParseException e) {
        }
    }

    public void test_getLong() throws IOException {
        this.m_t = new Tokenizer("2147483648");
        assertEquals(2147483648L, this.m_t.getLong());
        this.m_t = new Tokenizer("-10");
        try {
            this.m_t.getLong();
            fail("TextParseException not thrown");
        } catch (TextParseException e) {
        }
        this.m_t = new Tokenizer("19_identifier");
        try {
            this.m_t.getLong();
            fail("TextParseException not thrown");
        } catch (TextParseException e2) {
        }
    }

    public void test_getUInt32() throws IOException {
        this.m_t = new Tokenizer("2882400018");
        assertEquals(2882400018L, this.m_t.getUInt32());
        this.m_t = new Tokenizer("4294967296");
        try {
            this.m_t.getUInt32();
            fail("TextParseException not thrown");
        } catch (TextParseException e) {
        }
        this.m_t = new Tokenizer("-12345");
        try {
            this.m_t.getUInt32();
            fail("TextParseException not thrown");
        } catch (TextParseException e2) {
        }
    }

    public void test_getUInt16() throws IOException {
        this.m_t = new Tokenizer("43981");
        assertEquals(43981, (long) this.m_t.getUInt16());
        this.m_t = new Tokenizer("65536");
        try {
            this.m_t.getUInt16();
            fail("TextParseException not thrown");
        } catch (TextParseException e) {
        }
        this.m_t = new Tokenizer("-125");
        try {
            this.m_t.getUInt16();
            fail("TextParseException not thrown");
        } catch (TextParseException e2) {
        }
    }

    public void test_getUInt8() throws IOException {
        this.m_t = new Tokenizer("205");
        assertEquals(205, (long) this.m_t.getUInt8());
        this.m_t = new Tokenizer("256");
        try {
            this.m_t.getUInt8();
            fail("TextParseException not thrown");
        } catch (TextParseException e) {
        }
        this.m_t = new Tokenizer("-12");
        try {
            this.m_t.getUInt8();
            fail("TextParseException not thrown");
        } catch (TextParseException e2) {
        }
    }

    public void test_getTTL() throws IOException {
        this.m_t = new Tokenizer("59S");
        assertEquals(59, this.m_t.getTTL());
        this.m_t = new Tokenizer("2147483647");
        assertEquals(TTL.MAX_VALUE, this.m_t.getTTL());
        this.m_t = new Tokenizer("2147483648");
        assertEquals(TTL.MAX_VALUE, this.m_t.getTTL());
        this.m_t = new Tokenizer("Junk");
        try {
            this.m_t.getTTL();
            fail("TextParseException not thrown");
        } catch (TextParseException e) {
        }
    }

    public void test_getTTLLike() throws IOException {
        this.m_t = new Tokenizer("59S");
        assertEquals(59, this.m_t.getTTLLike());
        this.m_t = new Tokenizer("2147483647");
        assertEquals(TTL.MAX_VALUE, this.m_t.getTTLLike());
        this.m_t = new Tokenizer("2147483648");
        assertEquals(2147483648L, this.m_t.getTTLLike());
        this.m_t = new Tokenizer("Junk");
        try {
            this.m_t.getTTLLike();
            fail("TextParseException not thrown");
        } catch (TextParseException e) {
        }
    }

    public void test_getName() throws IOException, TextParseException {
        Name root = Name.fromString(".");
        this.m_t = new Tokenizer("junk");
        assertEquals(Name.fromString("junk."), this.m_t.getName(root));
        Name rel = Name.fromString("you.dig");
        this.m_t = new Tokenizer("junk");
        try {
            this.m_t.getName(rel);
            fail("RelativeNameException not thrown");
        } catch (RelativeNameException e) {
        }
        this.m_t = new Tokenizer("");
        try {
            this.m_t.getName(root);
            fail("TextParseException not thrown");
        } catch (TextParseException e2) {
        }
    }

    public void test_getEOL() throws IOException {
        this.m_t = new Tokenizer("id");
        this.m_t.getIdentifier();
        try {
            this.m_t.getEOL();
        } catch (TextParseException e) {
            fail(e.getMessage());
        }
        this.m_t = new Tokenizer("\n");
        try {
            this.m_t.getEOL();
            this.m_t.getEOL();
        } catch (TextParseException e2) {
            fail(e2.getMessage());
        }
        this.m_t = new Tokenizer("id");
        try {
            this.m_t.getEOL();
            fail("TextParseException not thrown");
        } catch (TextParseException e3) {
        }
    }

    public void test_getBase64() throws IOException {
        byte[] exp = {1, 2, 3, 4, 5, 6, 7, 8, 9};
        this.m_t = new Tokenizer("AQIDBAUGBwgJ");
        assertEquals(exp, this.m_t.getBase64());
        this.m_t = new Tokenizer("AQIDB AUGB   wgJ");
        assertEquals(exp, this.m_t.getBase64());
        this.m_t = new Tokenizer("AQIDBAUGBwgJ\nAB23DK");
        assertEquals(exp, this.m_t.getBase64());
        this.m_t = new Tokenizer("\n");
        assertNull(this.m_t.getBase64());
        this.m_t = new Tokenizer("\n");
        try {
            this.m_t.getBase64(true);
            fail("TextParseException not thrown");
        } catch (TextParseException e) {
        }
        this.m_t = new Tokenizer("not_base64");
        try {
            this.m_t.getBase64(false);
            fail("TextParseException not thrown");
        } catch (TextParseException e2) {
        }
        this.m_t = new Tokenizer("not_base64");
        try {
            this.m_t.getBase64(true);
            fail("TextParseException not thrown");
        } catch (TextParseException e3) {
        }
    }

    public void test_getHex() throws IOException {
        byte[] exp = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, Ascii.FF, 13, Ascii.SO, Ascii.SI};
        this.m_t = new Tokenizer("0102030405060708090A0B0C0D0E0F");
        assertEquals(exp, this.m_t.getHex());
        this.m_t = new Tokenizer("0102030 405 060708090A0B0C      0D0E0F");
        assertEquals(exp, this.m_t.getHex());
        this.m_t = new Tokenizer("0102030405060708090A0B0C0D0E0F\n01AB3FE");
        assertEquals(exp, this.m_t.getHex());
        this.m_t = new Tokenizer("\n");
        assertNull(this.m_t.getHex());
        this.m_t = new Tokenizer("\n");
        try {
            this.m_t.getHex(true);
            fail("TextParseException not thrown");
        } catch (TextParseException e) {
        }
        this.m_t = new Tokenizer("not_hex");
        try {
            this.m_t.getHex(false);
            fail("TextParseException not thrown");
        } catch (TextParseException e2) {
        }
        this.m_t = new Tokenizer("not_hex");
        try {
            this.m_t.getHex(true);
            fail("TextParseException not thrown");
        } catch (TextParseException e3) {
        }
    }
}
