package org.xbill.DNS;

import com.google.common.base.Ascii;
import java.io.IOException;
import java.util.Arrays;
import junit.framework.TestCase;

public class HINFORecordTest extends TestCase {
    public void test_ctor_0arg() {
        HINFORecord dr = new HINFORecord();
        assertNull(dr.getName());
        assertEquals(0, dr.getType());
        assertEquals(0, dr.getDClass());
        assertEquals(0, dr.getTTL());
    }

    public void test_getObject() {
        assertTrue(new HINFORecord().getObject() instanceof HINFORecord);
    }

    public void test_ctor_5arg() throws TextParseException {
        Name n = Name.fromString("The.Name.");
        HINFORecord dr = new HINFORecord(n, 1, 43981, "i686 Intel(R) Pentium(R) M processor 1.70GHz GenuineIntel GNU/Linux", "Linux troy 2.6.10-gentoo-r6 #8 Wed Apr 6 21:25:04 MDT 2005");
        assertEquals(n, dr.getName());
        assertEquals(1, dr.getDClass());
        assertEquals(13, dr.getType());
        assertEquals(43981, dr.getTTL());
        assertEquals("i686 Intel(R) Pentium(R) M processor 1.70GHz GenuineIntel GNU/Linux", dr.getCPU());
        assertEquals("Linux troy 2.6.10-gentoo-r6 #8 Wed Apr 6 21:25:04 MDT 2005", dr.getOS());
    }

    public void test_ctor_5arg_invalid_CPU() throws TextParseException {
        try {
            new HINFORecord(Name.fromString("The.Name."), 1, 43981, "i686 Intel(R) Pentium(R) M \\256 processor 1.70GHz GenuineIntel GNU/Linux", "Linux troy 2.6.10-gentoo-r6 #8 Wed Apr 6 21:25:04 MDT 2005");
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e) {
        }
    }

    public void test_ctor_5arg_invalid_OS() throws TextParseException {
        try {
            new HINFORecord(Name.fromString("The.Name."), 1, 43981, "i686 Intel(R) Pentium(R) M processor 1.70GHz GenuineIntel GNU/Linux", "Linux troy 2.6.10-gentoo-r6 \\1 #8 Wed Apr 6 21:25:04 MDT 2005");
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e) {
        }
    }

    public void test_rrFromWire() throws IOException {
        DNSInput in = new DNSInput(new byte[]{39, 73, 110, 116, 101, 108, 40, 82, 41, 32, 80, 101, 110, 116, 105, 117, 109, 40, 82, 41, 32, 77, 32, 112, 114, 111, 99, 101, 115, 115, 111, 114, 32, 49, 46, 55, 48, 71, 72, 122, Ascii.ESC, 76, 105, 110, 117, 120, 32, 116, 114, 111, 121, 32, 50, 46, 54, 46, 49, 48, 45, 103, 101, 110, 116, 111, 111, 45, 114, 54});
        HINFORecord dr = new HINFORecord();
        dr.rrFromWire(in);
        assertEquals("Intel(R) Pentium(R) M processor 1.70GHz", dr.getCPU());
        assertEquals("Linux troy 2.6.10-gentoo-r6", dr.getOS());
    }

    public void test_rdataFromString() throws IOException {
        StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append("\"");
        stringBuffer.append("Intel(R) Pentium(R) M processor 1.70GHz");
        stringBuffer.append("\" \"");
        stringBuffer.append("Linux troy 2.6.10-gentoo-r6");
        stringBuffer.append("\"");
        Tokenizer t = new Tokenizer(stringBuffer.toString());
        HINFORecord dr = new HINFORecord();
        dr.rdataFromString(t, (Name) null);
        assertEquals("Intel(R) Pentium(R) M processor 1.70GHz", dr.getCPU());
        assertEquals("Linux troy 2.6.10-gentoo-r6", dr.getOS());
    }

    public void test_rdataFromString_invalid_CPU() throws IOException {
        StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append("\"");
        stringBuffer.append("Intel(R) Pentium(R) \\388 M processor 1.70GHz");
        stringBuffer.append("\" \"");
        stringBuffer.append("Linux troy 2.6.10-gentoo-r6");
        stringBuffer.append("\"");
        try {
            new HINFORecord().rdataFromString(new Tokenizer(stringBuffer.toString()), (Name) null);
            fail("TextParseException not thrown");
        } catch (TextParseException e) {
        }
    }

    public void test_rdataFromString_invalid_OS() throws IOException {
        StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append("\"");
        stringBuffer.append("Intel(R) Pentium(R) M processor 1.70GHz");
        stringBuffer.append("\"");
        try {
            new HINFORecord().rdataFromString(new Tokenizer(stringBuffer.toString()), (Name) null);
            fail("TextParseException not thrown");
        } catch (TextParseException e) {
        }
    }

    public void test_rrToString() throws TextParseException {
        StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append("\"");
        stringBuffer.append("Intel(R) Pentium(R) M processor 1.70GHz");
        stringBuffer.append("\" \"");
        stringBuffer.append("Linux troy 2.6.10-gentoo-r6");
        stringBuffer.append("\"");
        assertEquals(stringBuffer.toString(), new HINFORecord(Name.fromString("The.Name."), 1, 291, "Intel(R) Pentium(R) M processor 1.70GHz", "Linux troy 2.6.10-gentoo-r6").rrToString());
    }

    public void test_rrToWire() throws TextParseException {
        byte[] raw = {39, 73, 110, 116, 101, 108, 40, 82, 41, 32, 80, 101, 110, 116, 105, 117, 109, 40, 82, 41, 32, 77, 32, 112, 114, 111, 99, 101, 115, 115, 111, 114, 32, 49, 46, 55, 48, 71, 72, 122, Ascii.ESC, 76, 105, 110, 117, 120, 32, 116, 114, 111, 121, 32, 50, 46, 54, 46, 49, 48, 45, 103, 101, 110, 116, 111, 111, 45, 114, 54};
        HINFORecord dr = new HINFORecord(Name.fromString("The.Name."), 1, 291, "Intel(R) Pentium(R) M processor 1.70GHz", "Linux troy 2.6.10-gentoo-r6");
        DNSOutput out = new DNSOutput();
        dr.rrToWire(out, (Compression) null, true);
        assertTrue(Arrays.equals(raw, out.toByteArray()));
    }
}
