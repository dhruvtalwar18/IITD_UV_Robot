package org.xbill.DNS;

import java.util.Arrays;
import junit.framework.TestCase;

public class DNSInputTest extends TestCase {
    private DNSInput m_di;
    private byte[] m_raw;

    private void assertEquals(byte[] exp, byte[] act) {
        assertTrue(Arrays.equals(exp, act));
    }

    public void setUp() {
        this.m_raw = new byte[]{0, 1, 2, 3, 4, 5, -1, -1, -1, -1};
        this.m_di = new DNSInput(this.m_raw);
    }

    public void test_initial_state() {
        assertEquals(0, this.m_di.current());
        assertEquals(10, this.m_di.remaining());
    }

    public void test_jump1() {
        this.m_di.jump(1);
        assertEquals(1, this.m_di.current());
        assertEquals(9, this.m_di.remaining());
    }

    public void test_jump2() {
        this.m_di.jump(9);
        assertEquals(9, this.m_di.current());
        assertEquals(1, this.m_di.remaining());
    }

    public void test_jump_invalid() {
        try {
            this.m_di.jump(10);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e) {
        }
    }

    public void test_setActive() {
        this.m_di.setActive(5);
        assertEquals(0, this.m_di.current());
        assertEquals(5, this.m_di.remaining());
    }

    public void test_setActive_boundary1() {
        this.m_di.setActive(10);
        assertEquals(0, this.m_di.current());
        assertEquals(10, this.m_di.remaining());
    }

    public void test_setActive_boundary2() {
        this.m_di.setActive(0);
        assertEquals(0, this.m_di.current());
        assertEquals(0, this.m_di.remaining());
    }

    public void test_setActive_invalid() {
        try {
            this.m_di.setActive(11);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e) {
        }
    }

    public void test_clearActive() {
        this.m_di.clearActive();
        assertEquals(0, this.m_di.current());
        assertEquals(10, this.m_di.remaining());
        this.m_di.setActive(5);
        this.m_di.clearActive();
        assertEquals(0, this.m_di.current());
        assertEquals(10, this.m_di.remaining());
    }

    public void test_restore_invalid() {
        try {
            this.m_di.restore();
            fail("IllegalStateException not thrown");
        } catch (IllegalStateException e) {
        }
    }

    public void test_save_restore() {
        this.m_di.jump(4);
        assertEquals(4, this.m_di.current());
        assertEquals(6, this.m_di.remaining());
        this.m_di.save();
        this.m_di.jump(0);
        assertEquals(0, this.m_di.current());
        assertEquals(10, this.m_di.remaining());
        this.m_di.restore();
        assertEquals(4, this.m_di.current());
        assertEquals(6, this.m_di.remaining());
    }

    public void test_readU8_basic() throws WireParseException {
        int v1 = this.m_di.readU8();
        assertEquals(1, this.m_di.current());
        assertEquals(9, this.m_di.remaining());
        assertEquals(0, v1);
    }

    public void test_readU8_maxval() throws WireParseException {
        this.m_di.jump(9);
        int v1 = this.m_di.readU8();
        assertEquals(10, this.m_di.current());
        assertEquals(0, this.m_di.remaining());
        assertEquals(255, v1);
        try {
            int v12 = this.m_di.readU8();
            fail("WireParseException not thrown");
        } catch (WireParseException e) {
        }
    }

    public void test_readU16_basic() throws WireParseException {
        int v1 = this.m_di.readU16();
        assertEquals(2, this.m_di.current());
        assertEquals(8, this.m_di.remaining());
        assertEquals(1, v1);
        this.m_di.jump(1);
        assertEquals(258, this.m_di.readU16());
    }

    public void test_readU16_maxval() throws WireParseException {
        this.m_di.jump(8);
        int v = this.m_di.readU16();
        assertEquals(10, this.m_di.current());
        assertEquals(0, this.m_di.remaining());
        assertEquals(65535, v);
        try {
            this.m_di.jump(9);
            this.m_di.readU16();
            fail("WireParseException not thrown");
        } catch (WireParseException e) {
        }
    }

    public void test_readU32_basic() throws WireParseException {
        long v1 = this.m_di.readU32();
        assertEquals(4, this.m_di.current());
        assertEquals(6, this.m_di.remaining());
        assertEquals(66051, v1);
    }

    public void test_readU32_maxval() throws WireParseException {
        this.m_di.jump(6);
        long v = this.m_di.readU32();
        assertEquals(10, this.m_di.current());
        assertEquals(0, this.m_di.remaining());
        assertEquals(4294967295L, v);
        try {
            this.m_di.jump(7);
            this.m_di.readU32();
            fail("WireParseException not thrown");
        } catch (WireParseException e) {
        }
    }

    public void test_readByteArray_0arg() throws WireParseException {
        this.m_di.jump(1);
        byte[] out = this.m_di.readByteArray();
        assertEquals(10, this.m_di.current());
        int i = 0;
        assertEquals(0, this.m_di.remaining());
        assertEquals(9, out.length);
        while (true) {
            int i2 = i;
            if (i2 < 9) {
                assertEquals(this.m_raw[i2 + 1], out[i2]);
                i = i2 + 1;
            } else {
                return;
            }
        }
    }

    public void test_readByteArray_0arg_boundary() throws WireParseException {
        this.m_di.jump(9);
        this.m_di.readU8();
        assertEquals(0, this.m_di.readByteArray().length);
    }

    public void test_readByteArray_1arg() throws WireParseException {
        byte[] out = this.m_di.readByteArray(2);
        assertEquals(2, this.m_di.current());
        assertEquals(8, this.m_di.remaining());
        assertEquals(2, out.length);
        assertEquals(0, out[0]);
        assertEquals(1, out[1]);
    }

    public void test_readByteArray_1arg_boundary() throws WireParseException {
        byte[] out = this.m_di.readByteArray(10);
        assertEquals(10, this.m_di.current());
        assertEquals(0, this.m_di.remaining());
        assertEquals(this.m_raw, out);
    }

    public void test_readByteArray_1arg_invalid() {
        try {
            this.m_di.readByteArray(11);
            fail("WireParseException not thrown");
        } catch (WireParseException e) {
        }
    }

    public void test_readByteArray_3arg() throws WireParseException {
        byte[] data = new byte[5];
        this.m_di.jump(4);
        this.m_di.readByteArray(data, 1, 4);
        assertEquals(8, this.m_di.current());
        assertEquals(0, data[0]);
        for (int i = 0; i < 4; i++) {
            assertEquals(this.m_raw[i + 4], data[i + 1]);
        }
    }

    public void test_readCountedSting() throws WireParseException {
        this.m_di.jump(1);
        byte[] out = this.m_di.readCountedString();
        assertEquals(1, out.length);
        assertEquals(3, this.m_di.current());
        assertEquals(out[0], 2);
    }
}
