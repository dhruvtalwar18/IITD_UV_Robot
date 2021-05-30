package org.xbill.DNS;

import com.google.common.base.Ascii;
import java.util.Arrays;
import junit.framework.TestCase;

public class DNSOutputTest extends TestCase {
    private DNSOutput m_do;

    public void setUp() {
        this.m_do = new DNSOutput(1);
    }

    private void assertEquals(byte[] exp, byte[] act) {
        assertTrue(Arrays.equals(exp, act));
    }

    public void test_default_ctor() {
        this.m_do = new DNSOutput();
        assertEquals(0, this.m_do.current());
    }

    public void test_initial_state() {
        assertEquals(0, this.m_do.current());
        try {
            this.m_do.restore();
            fail("IllegalStateException not thrown");
        } catch (IllegalStateException e) {
        }
        try {
            this.m_do.jump(1);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e2) {
        }
    }

    public void test_writeU8_basic() {
        this.m_do.writeU8(1);
        assertEquals(1, this.m_do.current());
        byte[] curr = this.m_do.toByteArray();
        assertEquals(1, curr.length);
        assertEquals(1, curr[0]);
    }

    public void test_writeU8_expand() {
        this.m_do.writeU8(1);
        this.m_do.writeU8(2);
        assertEquals(2, this.m_do.current());
        byte[] curr = this.m_do.toByteArray();
        assertEquals(2, curr.length);
        assertEquals(1, curr[0]);
        assertEquals(2, curr[1]);
    }

    public void test_writeU8_max() {
        this.m_do.writeU8(255);
        assertEquals((byte) -1, this.m_do.toByteArray()[0]);
    }

    public void test_writeU8_toobig() {
        try {
            this.m_do.writeU8(511);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e) {
        }
    }

    public void test_writeU16_basic() {
        this.m_do.writeU16(256);
        assertEquals(2, this.m_do.current());
        byte[] curr = this.m_do.toByteArray();
        assertEquals(2, curr.length);
        assertEquals(1, curr[0]);
        assertEquals(0, curr[1]);
    }

    public void test_writeU16_max() {
        this.m_do.writeU16(65535);
        byte[] curr = this.m_do.toByteArray();
        assertEquals((byte) -1, curr[0]);
        assertEquals((byte) -1, curr[1]);
    }

    public void test_writeU16_toobig() {
        try {
            this.m_do.writeU16(131071);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e) {
        }
    }

    public void test_writeU32_basic() {
        this.m_do.writeU32(285216785);
        assertEquals(4, this.m_do.current());
        byte[] curr = this.m_do.toByteArray();
        assertEquals(4, curr.length);
        assertEquals(17, curr[0]);
        assertEquals(0, curr[1]);
        assertEquals(16, curr[2]);
        assertEquals(17, curr[3]);
    }

    public void test_writeU32_max() {
        this.m_do.writeU32(4294967295L);
        byte[] curr = this.m_do.toByteArray();
        assertEquals((byte) -1, curr[0]);
        assertEquals((byte) -1, curr[1]);
        assertEquals((byte) -1, curr[2]);
        assertEquals((byte) -1, curr[3]);
    }

    public void test_writeU32_toobig() {
        try {
            this.m_do.writeU32(8589934591L);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e) {
        }
    }

    public void test_jump_basic() {
        this.m_do.writeU32(287454020);
        assertEquals(4, this.m_do.current());
        this.m_do.jump(2);
        assertEquals(2, this.m_do.current());
        this.m_do.writeU8(153);
        byte[] curr = this.m_do.toByteArray();
        assertEquals(3, curr.length);
        assertEquals(17, curr[0]);
        assertEquals(34, curr[1]);
        assertEquals((byte) -103, curr[2]);
    }

    public void test_writeByteArray_1arg() {
        byte[] in = {-85, -51, -17, Ascii.DC2, 52};
        this.m_do.writeByteArray(in);
        assertEquals(5, this.m_do.current());
        assertEquals(in, this.m_do.toByteArray());
    }

    public void test_writeByteArray_3arg() {
        byte[] in = {-85, -51, -17, Ascii.DC2, 52};
        this.m_do.writeByteArray(in, 2, 3);
        assertEquals(3, this.m_do.current());
        assertEquals(new byte[]{in[2], in[3], in[4]}, this.m_do.toByteArray());
    }

    public void test_writeCountedString_basic() {
        byte[] in = {104, 101, 108, 76, 48};
        this.m_do.writeCountedString(in);
        assertEquals(in.length + 1, this.m_do.current());
        byte[] curr = this.m_do.toByteArray();
        assertEquals(new byte[]{(byte) in.length, in[0], in[1], in[2], in[3], in[4]}, curr);
    }

    public void test_writeCountedString_empty() {
        byte[] in = new byte[0];
        this.m_do.writeCountedString(in);
        assertEquals(in.length + 1, this.m_do.current());
        byte[] curr = this.m_do.toByteArray();
        assertEquals(new byte[]{(byte) in.length}, curr);
    }

    public void test_writeCountedString_toobig() {
        try {
            this.m_do.writeCountedString(new byte[256]);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e) {
        }
    }

    public void test_save_restore() {
        this.m_do.writeU32(305419896);
        assertEquals(4, this.m_do.current());
        this.m_do.save();
        this.m_do.writeU16(43981);
        assertEquals(6, this.m_do.current());
        this.m_do.restore();
        assertEquals(4, this.m_do.current());
        try {
            this.m_do.restore();
            fail("IllegalArgumentException not thrown");
        } catch (IllegalStateException e) {
        }
    }
}
