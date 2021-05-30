package org.xbill.DNS;

import com.google.common.base.Ascii;
import java.io.IOException;
import junit.framework.TestCase;

public class HeaderTest extends TestCase {
    private Header m_h;

    public void setUp() {
        this.m_h = new Header(43981);
    }

    public void test_fixture_state() {
        assertEquals(43981, this.m_h.getID());
        boolean[] flags = this.m_h.getFlags();
        for (boolean assertFalse : flags) {
            assertFalse(assertFalse);
        }
        assertEquals(0, this.m_h.getRcode());
        assertEquals(0, this.m_h.getOpcode());
        assertEquals(0, this.m_h.getCount(0));
        assertEquals(0, this.m_h.getCount(1));
        assertEquals(0, this.m_h.getCount(2));
        assertEquals(0, this.m_h.getCount(3));
    }

    public void test_ctor_0arg() {
        this.m_h = new Header();
        assertTrue(this.m_h.getID() >= 0 && this.m_h.getID() < 65535);
        boolean[] flags = this.m_h.getFlags();
        for (boolean assertFalse : flags) {
            assertFalse(assertFalse);
        }
        assertEquals(0, this.m_h.getRcode());
        assertEquals(0, this.m_h.getOpcode());
        assertEquals(0, this.m_h.getCount(0));
        assertEquals(0, this.m_h.getCount(1));
        assertEquals(0, this.m_h.getCount(2));
        assertEquals(0, this.m_h.getCount(3));
    }

    public void test_ctor_DNSInput() throws IOException {
        this.m_h = new Header(new DNSInput(new byte[]{Ascii.DC2, -85, -113, -67, 101, Ascii.FS, 16, -16, -104, -70, 113, -112}));
        assertEquals(4779, this.m_h.getID());
        boolean[] flags = this.m_h.getFlags();
        assertTrue(flags[0]);
        assertEquals(1, this.m_h.getOpcode());
        assertTrue(flags[5]);
        assertTrue(flags[6]);
        assertTrue(flags[7]);
        assertTrue(flags[8]);
        assertFalse(flags[9]);
        assertTrue(flags[10]);
        assertTrue(flags[11]);
        assertEquals(13, this.m_h.getRcode());
        assertEquals(25884, this.m_h.getCount(0));
        assertEquals(4336, this.m_h.getCount(1));
        assertEquals(39098, this.m_h.getCount(2));
        assertEquals(29072, this.m_h.getCount(3));
    }

    public void test_toWire() throws IOException {
        byte[] raw = {Ascii.DC2, -85, -113, -67, 101, Ascii.FS, 16, -16, -104, -70, 113, -112};
        this.m_h = new Header(raw);
        DNSOutput dout = new DNSOutput();
        this.m_h.toWire(dout);
        byte[] out = dout.toByteArray();
        assertEquals(12, out.length);
        int i = 0;
        for (int i2 = 0; i2 < out.length; i2++) {
            assertEquals(raw[i2], out[i2]);
        }
        this.m_h.setOpcode(10);
        assertEquals(10, this.m_h.getOpcode());
        this.m_h.setRcode(7);
        raw[2] = -41;
        raw[3] = -73;
        byte[] out2 = this.m_h.toWire();
        assertEquals(12, out2.length);
        while (true) {
            int i3 = i;
            if (i3 < out2.length) {
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append("i=");
                stringBuffer.append(i3);
                assertEquals(stringBuffer.toString(), raw[i3], out2[i3]);
                i = i3 + 1;
            } else {
                return;
            }
        }
    }

    public void test_flags() {
        this.m_h.setFlag(0);
        this.m_h.setFlag(5);
        assertTrue(this.m_h.getFlag(0));
        assertTrue(this.m_h.getFlags()[0]);
        assertTrue(this.m_h.getFlag(5));
        assertTrue(this.m_h.getFlags()[5]);
        this.m_h.unsetFlag(0);
        assertFalse(this.m_h.getFlag(0));
        assertFalse(this.m_h.getFlags()[0]);
        assertTrue(this.m_h.getFlag(5));
        assertTrue(this.m_h.getFlags()[5]);
        this.m_h.unsetFlag(5);
        assertFalse(this.m_h.getFlag(0));
        assertFalse(this.m_h.getFlags()[0]);
        assertFalse(this.m_h.getFlag(5));
        assertFalse(this.m_h.getFlags()[5]);
        boolean[] flags = this.m_h.getFlags();
        for (int i = 0; i < flags.length; i++) {
            if ((i <= 0 || i >= 5) && i <= 11) {
                assertFalse(flags[i]);
            }
        }
    }

    public void test_flags_invalid() {
        try {
            this.m_h.setFlag(-1);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e) {
        }
        try {
            this.m_h.setFlag(1);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e2) {
        }
        try {
            this.m_h.setFlag(16);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e3) {
        }
        try {
            this.m_h.unsetFlag(-1);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e4) {
        }
        try {
            this.m_h.unsetFlag(13);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e5) {
        }
        try {
            this.m_h.unsetFlag(16);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e6) {
        }
        try {
            this.m_h.getFlag(-1);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e7) {
        }
        try {
            this.m_h.getFlag(4);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e8) {
        }
        try {
            this.m_h.getFlag(16);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e9) {
        }
    }

    public void test_ID() {
        assertEquals(43981, this.m_h.getID());
        this.m_h = new Header();
        int id = this.m_h.getID();
        assertEquals(id, this.m_h.getID());
        assertTrue(id >= 0 && id < 65535);
        this.m_h.setID(56506);
        assertEquals(56506, this.m_h.getID());
    }

    public void test_setID_invalid() {
        try {
            this.m_h.setID(65536);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e) {
        }
        try {
            this.m_h.setID(-1);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e2) {
        }
    }

    public void test_Rcode() {
        int i = 0;
        assertEquals(0, this.m_h.getRcode());
        this.m_h.setRcode(10);
        assertEquals(10, this.m_h.getRcode());
        while (true) {
            int i2 = i;
            if (i2 < 12) {
                if ((i2 <= 0 || i2 >= 5) && i2 <= 11) {
                    assertFalse(this.m_h.getFlag(i2));
                }
                i = i2 + 1;
            } else {
                return;
            }
        }
    }

    public void test_setRcode_invalid() {
        try {
            this.m_h.setRcode(-1);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e) {
        }
        try {
            this.m_h.setRcode(256);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e2) {
        }
    }

    public void test_Opcode() {
        assertEquals(0, this.m_h.getOpcode());
        this.m_h.setOpcode(14);
        assertEquals(14, this.m_h.getOpcode());
        assertFalse(this.m_h.getFlag(0));
        for (int i = 5; i < 12; i++) {
            assertFalse(this.m_h.getFlag(i));
        }
        assertEquals(0, this.m_h.getRcode());
    }

    public void test_setOpcode_invalid() {
        try {
            this.m_h.setOpcode(-1);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e) {
        }
        try {
            this.m_h.setOpcode(256);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e2) {
        }
    }

    public void test_Count() {
        this.m_h.setCount(2, 30);
        assertEquals(0, this.m_h.getCount(0));
        assertEquals(0, this.m_h.getCount(1));
        assertEquals(30, this.m_h.getCount(2));
        assertEquals(0, this.m_h.getCount(3));
        this.m_h.incCount(0);
        assertEquals(1, this.m_h.getCount(0));
        this.m_h.decCount(2);
        assertEquals(29, this.m_h.getCount(2));
    }

    public void test_setCount_invalid() {
        try {
            this.m_h.setCount(-1, 0);
            fail("ArrayIndexOutOfBoundsException not thrown");
        } catch (ArrayIndexOutOfBoundsException e) {
        }
        try {
            this.m_h.setCount(4, 0);
            fail("ArrayIndexOutOfBoundsException not thrown");
        } catch (ArrayIndexOutOfBoundsException e2) {
        }
        try {
            this.m_h.setCount(0, -1);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e3) {
        }
        try {
            this.m_h.setCount(3, 65536);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e4) {
        }
    }

    public void test_getCount_invalid() {
        try {
            this.m_h.getCount(-1);
            fail("ArrayIndexOutOfBoundsException not thrown");
        } catch (ArrayIndexOutOfBoundsException e) {
        }
        try {
            this.m_h.getCount(4);
            fail("ArrayIndexOutOfBoundsException not thrown");
        } catch (ArrayIndexOutOfBoundsException e2) {
        }
    }

    public void test_incCount_invalid() {
        this.m_h.setCount(1, 65535);
        try {
            this.m_h.incCount(1);
            fail("IllegalStateException not thrown");
        } catch (IllegalStateException e) {
        }
    }

    public void test_decCount_invalid() {
        this.m_h.setCount(2, 0);
        try {
            this.m_h.decCount(2);
            fail("IllegalStateException not thrown");
        } catch (IllegalStateException e) {
        }
    }

    public void test_toString() {
        this.m_h.setOpcode(Opcode.value("STATUS"));
        this.m_h.setRcode(Rcode.value("NXDOMAIN"));
        boolean z = false;
        this.m_h.setFlag(0);
        this.m_h.setFlag(7);
        this.m_h.setFlag(8);
        this.m_h.setFlag(11);
        this.m_h.setCount(1, 255);
        this.m_h.setCount(2, 10);
        String text = this.m_h.toString();
        assertFalse(text.indexOf("id: 43981") == -1);
        assertFalse(text.indexOf("opcode: STATUS") == -1);
        assertFalse(text.indexOf("status: NXDOMAIN") == -1);
        assertFalse(text.indexOf(" qr ") == -1);
        assertFalse(text.indexOf(" rd ") == -1);
        assertFalse(text.indexOf(" ra ") == -1);
        assertFalse(text.indexOf(" cd ") == -1);
        assertFalse(text.indexOf("qd: 0 ") == -1);
        assertFalse(text.indexOf("an: 255 ") == -1);
        assertFalse(text.indexOf("au: 10 ") == -1);
        if (text.indexOf("ad: 0 ") == -1) {
            z = true;
        }
        assertFalse(z);
    }

    public void test_clone() {
        this.m_h.setOpcode(Opcode.value("IQUERY"));
        this.m_h.setRcode(Rcode.value("SERVFAIL"));
        this.m_h.setFlag(0);
        this.m_h.setFlag(7);
        this.m_h.setFlag(8);
        this.m_h.setFlag(11);
        this.m_h.setCount(1, 255);
        this.m_h.setCount(2, 10);
        Header h2 = (Header) this.m_h.clone();
        assertNotSame(this.m_h, h2);
        assertEquals(this.m_h.getID(), h2.getID());
        for (int i = 0; i < 16; i++) {
            if ((i <= 0 || i >= 5) && i <= 11) {
                assertEquals(this.m_h.getFlag(i), h2.getFlag(i));
            }
        }
        for (int i2 = 0; i2 < 4; i2++) {
            assertEquals(this.m_h.getCount(i2), h2.getCount(i2));
        }
    }
}
