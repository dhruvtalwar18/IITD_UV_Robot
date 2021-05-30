package org.xbill.DNS;

import java.io.IOException;
import java.util.Arrays;
import junit.framework.Test;
import junit.framework.TestCase;
import junit.framework.TestSuite;
import org.apache.commons.net.telnet.TelnetCommand;
import rocon_app_manager_msgs.ErrorCodes;

public class DSRecordTest extends TestCase {
    static /* synthetic */ Class class$org$xbill$DNS$DSRecordTest;
    static /* synthetic */ Class class$org$xbill$DNS$DSRecordTest$Test_Ctor_7arg;

    public void test_ctor_0arg() {
        DSRecord dr = new DSRecord();
        assertNull(dr.getName());
        assertEquals(0, dr.getType());
        assertEquals(0, dr.getDClass());
        assertEquals(0, dr.getTTL());
        assertEquals(0, dr.getAlgorithm());
        assertEquals(0, dr.getDigestID());
        assertNull(dr.getDigest());
        assertEquals(0, dr.getFootprint());
    }

    public void test_getObject() {
        assertTrue(new DSRecord().getObject() instanceof DSRecord);
    }

    public static class Test_Ctor_7arg extends TestCase {
        private int m_algorithm;
        private byte[] m_digest;
        private int m_digestid;
        private int m_footprint;
        private Name m_n;
        private long m_ttl;

        /* access modifiers changed from: protected */
        public void setUp() throws TextParseException {
            this.m_n = Name.fromString("The.Name.");
            this.m_ttl = 43981;
            this.m_footprint = 61185;
            this.m_algorithm = 35;
            this.m_digestid = 69;
            this.m_digest = new byte[]{103, -119, -85, -51, -17};
        }

        public void test_basic() throws TextParseException {
            DSRecord dr = new DSRecord(this.m_n, 1, this.m_ttl, this.m_footprint, this.m_algorithm, this.m_digestid, this.m_digest);
            assertEquals(this.m_n, dr.getName());
            assertEquals(1, dr.getDClass());
            assertEquals(43, dr.getType());
            assertEquals(this.m_ttl, dr.getTTL());
            assertEquals(this.m_footprint, dr.getFootprint());
            assertEquals(this.m_algorithm, dr.getAlgorithm());
            assertEquals(this.m_digestid, dr.getDigestID());
            assertTrue(Arrays.equals(this.m_digest, dr.getDigest()));
        }

        public void test_toosmall_footprint() throws TextParseException {
            try {
                new DSRecord(this.m_n, 1, this.m_ttl, -1, this.m_algorithm, this.m_digestid, this.m_digest);
                fail("IllegalArgumentException not thrown");
            } catch (IllegalArgumentException e) {
            }
        }

        public void test_toobig_footprint() throws TextParseException {
            try {
                new DSRecord(this.m_n, 1, this.m_ttl, 65536, this.m_algorithm, this.m_digestid, this.m_digest);
                fail("IllegalArgumentException not thrown");
            } catch (IllegalArgumentException e) {
            }
        }

        public void test_toosmall_algorithm() throws TextParseException {
            try {
                new DSRecord(this.m_n, 1, this.m_ttl, this.m_footprint, -1, this.m_digestid, this.m_digest);
                fail("IllegalArgumentException not thrown");
            } catch (IllegalArgumentException e) {
            }
        }

        public void test_toobig_algorithm() throws TextParseException {
            try {
                new DSRecord(this.m_n, 1, this.m_ttl, this.m_footprint, 65536, this.m_digestid, this.m_digest);
                fail("IllegalArgumentException not thrown");
            } catch (IllegalArgumentException e) {
            }
        }

        public void test_toosmall_digestid() throws TextParseException {
            try {
                new DSRecord(this.m_n, 1, this.m_ttl, this.m_footprint, this.m_algorithm, -1, this.m_digest);
                fail("IllegalArgumentException not thrown");
            } catch (IllegalArgumentException e) {
            }
        }

        public void test_toobig_digestid() throws TextParseException {
            try {
                new DSRecord(this.m_n, 1, this.m_ttl, this.m_footprint, this.m_algorithm, 65536, this.m_digest);
                fail("IllegalArgumentException not thrown");
            } catch (IllegalArgumentException e) {
            }
        }

        public void test_null_digest() {
            DSRecord dr = new DSRecord(this.m_n, 1, this.m_ttl, this.m_footprint, this.m_algorithm, this.m_digestid, (byte[]) null);
            assertEquals(this.m_n, dr.getName());
            assertEquals(1, dr.getDClass());
            assertEquals(43, dr.getType());
            assertEquals(this.m_ttl, dr.getTTL());
            assertEquals(this.m_footprint, dr.getFootprint());
            assertEquals(this.m_algorithm, dr.getAlgorithm());
            assertEquals(this.m_digestid, dr.getDigestID());
            assertNull(dr.getDigest());
        }
    }

    public void test_rrFromWire() throws IOException {
        DNSInput in = new DNSInput(new byte[]{-85, -51, -17, 1, ErrorCodes.INVITING_CONTROLLER_BLACKLISTED, 69, 103, -119});
        DSRecord dr = new DSRecord();
        dr.rrFromWire(in);
        assertEquals(43981, dr.getFootprint());
        assertEquals(TelnetCommand.EOR, dr.getAlgorithm());
        assertEquals(1, dr.getDigestID());
        assertTrue(Arrays.equals(new byte[]{ErrorCodes.INVITING_CONTROLLER_BLACKLISTED, 69, 103, -119}, dr.getDigest()));
    }

    public void test_rdataFromString() throws IOException {
        byte[] bArr = {-85, -51, -17, 1, ErrorCodes.INVITING_CONTROLLER_BLACKLISTED, 69, 103, -119};
        Tokenizer t = new Tokenizer("43981 239 1 23456789AB");
        DSRecord dr = new DSRecord();
        dr.rdataFromString(t, (Name) null);
        assertEquals(43981, dr.getFootprint());
        assertEquals(TelnetCommand.EOR, dr.getAlgorithm());
        assertEquals(1, dr.getDigestID());
        assertTrue(Arrays.equals(new byte[]{ErrorCodes.INVITING_CONTROLLER_BLACKLISTED, 69, 103, -119, -85}, dr.getDigest()));
    }

    public void test_rrToString() throws TextParseException {
        assertEquals("43981 239 1 23456789AB", new DSRecord(Name.fromString("The.Name."), 1, 291, 43981, TelnetCommand.EOR, 1, new byte[]{ErrorCodes.INVITING_CONTROLLER_BLACKLISTED, 69, 103, -119, -85}).rrToString());
    }

    public void test_rrToWire() throws TextParseException {
        DSRecord dr = new DSRecord(Name.fromString("The.Name."), 1, 291, 43981, TelnetCommand.EOR, 1, new byte[]{ErrorCodes.INVITING_CONTROLLER_BLACKLISTED, 69, 103, -119, -85});
        byte[] exp = {-85, -51, -17, 1, ErrorCodes.INVITING_CONTROLLER_BLACKLISTED, 69, 103, -119, -85};
        DNSOutput out = new DNSOutput();
        dr.rrToWire(out, (Compression) null, true);
        assertTrue(Arrays.equals(exp, out.toByteArray()));
    }

    public static Test suite() {
        Class cls;
        Class cls2;
        TestSuite s = new TestSuite();
        if (class$org$xbill$DNS$DSRecordTest$Test_Ctor_7arg == null) {
            cls = class$("org.xbill.DNS.DSRecordTest$Test_Ctor_7arg");
            class$org$xbill$DNS$DSRecordTest$Test_Ctor_7arg = cls;
        } else {
            cls = class$org$xbill$DNS$DSRecordTest$Test_Ctor_7arg;
        }
        s.addTestSuite(cls);
        if (class$org$xbill$DNS$DSRecordTest == null) {
            cls2 = class$("org.xbill.DNS.DSRecordTest");
            class$org$xbill$DNS$DSRecordTest = cls2;
        } else {
            cls2 = class$org$xbill$DNS$DSRecordTest;
        }
        s.addTestSuite(cls2);
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
