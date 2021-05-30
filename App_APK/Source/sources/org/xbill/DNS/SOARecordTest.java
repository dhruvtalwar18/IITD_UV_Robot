package org.xbill.DNS;

import com.google.common.base.Ascii;
import java.io.IOException;
import java.net.UnknownHostException;
import java.util.Arrays;
import java.util.Random;
import junit.framework.Test;
import junit.framework.TestCase;
import junit.framework.TestSuite;

public class SOARecordTest {
    static /* synthetic */ Class class$org$xbill$DNS$SOARecordTest$Test_init;
    static /* synthetic */ Class class$org$xbill$DNS$SOARecordTest$Test_rdataFromString;
    static /* synthetic */ Class class$org$xbill$DNS$SOARecordTest$Test_rrFromWire;
    static /* synthetic */ Class class$org$xbill$DNS$SOARecordTest$Test_rrToString;
    static /* synthetic */ Class class$org$xbill$DNS$SOARecordTest$Test_rrToWire;
    private static final Random m_random = new Random();

    /* access modifiers changed from: private */
    public static long randomU16() {
        return m_random.nextLong() >>> 48;
    }

    /* access modifiers changed from: private */
    public static long randomU32() {
        return m_random.nextLong() >>> 32;
    }

    public static class Test_init extends TestCase {
        private Name m_admin;
        private Name m_an;
        private long m_expire;
        private Name m_host;
        private long m_minimum;
        private long m_refresh;
        private long m_retry;
        private Name m_rn;
        private long m_serial;
        private long m_ttl;

        /* access modifiers changed from: protected */
        public void setUp() throws TextParseException, UnknownHostException {
            this.m_an = Name.fromString("My.Absolute.Name.");
            this.m_rn = Name.fromString("My.Relative.Name");
            this.m_host = Name.fromString("My.Host.Name.");
            this.m_admin = Name.fromString("My.Administrative.Name.");
            this.m_ttl = SOARecordTest.randomU16();
            this.m_serial = SOARecordTest.randomU32();
            this.m_refresh = SOARecordTest.randomU32();
            this.m_retry = SOARecordTest.randomU32();
            this.m_expire = SOARecordTest.randomU32();
            this.m_minimum = SOARecordTest.randomU32();
        }

        public void test_0arg() throws UnknownHostException {
            SOARecord ar = new SOARecord();
            assertNull(ar.getName());
            assertEquals(0, ar.getType());
            assertEquals(0, ar.getDClass());
            assertEquals(0, ar.getTTL());
            assertNull(ar.getHost());
            assertNull(ar.getAdmin());
            assertEquals(0, ar.getSerial());
            assertEquals(0, ar.getRefresh());
            assertEquals(0, ar.getRetry());
            assertEquals(0, ar.getExpire());
            assertEquals(0, ar.getMinimum());
        }

        public void test_getObject() {
            assertTrue(new SOARecord().getObject() instanceof SOARecord);
        }

        public void test_10arg() {
            SOARecord ar = new SOARecord(this.m_an, 1, this.m_ttl, this.m_host, this.m_admin, this.m_serial, this.m_refresh, this.m_retry, this.m_expire, this.m_minimum);
            assertEquals(this.m_an, ar.getName());
            assertEquals(6, ar.getType());
            assertEquals(1, ar.getDClass());
            assertEquals(this.m_ttl, ar.getTTL());
            assertEquals(this.m_host, ar.getHost());
            assertEquals(this.m_admin, ar.getAdmin());
            assertEquals(this.m_serial, ar.getSerial());
            assertEquals(this.m_refresh, ar.getRefresh());
            assertEquals(this.m_retry, ar.getRetry());
            assertEquals(this.m_expire, ar.getExpire());
            assertEquals(this.m_minimum, ar.getMinimum());
        }

        public void test_10arg_relative_name() {
            try {
                Name name = this.m_rn;
                long j = this.m_ttl;
                long j2 = j;
                new SOARecord(name, 1, j2, this.m_host, this.m_admin, this.m_serial, this.m_refresh, this.m_retry, this.m_expire, this.m_minimum);
                fail("RelativeNameException not thrown");
            } catch (RelativeNameException e) {
            }
        }

        public void test_10arg_relative_host() {
            try {
                Name name = this.m_an;
                long j = this.m_ttl;
                long j2 = j;
                new SOARecord(name, 1, j2, this.m_rn, this.m_admin, this.m_serial, this.m_refresh, this.m_retry, this.m_expire, this.m_minimum);
                fail("RelativeNameException not thrown");
            } catch (RelativeNameException e) {
            }
        }

        public void test_10arg_relative_admin() {
            try {
                Name name = this.m_an;
                long j = this.m_ttl;
                long j2 = j;
                new SOARecord(name, 1, j2, this.m_host, this.m_rn, this.m_serial, this.m_refresh, this.m_retry, this.m_expire, this.m_minimum);
                fail("RelativeNameException not thrown");
            } catch (RelativeNameException e) {
            }
        }

        public void test_10arg_negative_serial() {
            try {
                new SOARecord(this.m_an, 1, this.m_ttl, this.m_host, this.m_admin, -1, this.m_refresh, this.m_retry, this.m_expire, this.m_minimum);
                fail("IllegalArgumentException not thrown");
            } catch (IllegalArgumentException e) {
            }
        }

        public void test_10arg_toobig_serial() {
            try {
                new SOARecord(this.m_an, 1, this.m_ttl, this.m_host, this.m_admin, 4294967296L, this.m_refresh, this.m_retry, this.m_expire, this.m_minimum);
                fail("IllegalArgumentException not thrown");
            } catch (IllegalArgumentException e) {
            }
        }

        public void test_10arg_negative_refresh() {
            try {
                new SOARecord(this.m_an, 1, this.m_ttl, this.m_host, this.m_admin, this.m_serial, -1, this.m_retry, this.m_expire, this.m_minimum);
                fail("IllegalArgumentException not thrown");
            } catch (IllegalArgumentException e) {
            }
        }

        public void test_10arg_toobig_refresh() {
            try {
                new SOARecord(this.m_an, 1, this.m_ttl, this.m_host, this.m_admin, this.m_serial, 4294967296L, this.m_retry, this.m_expire, this.m_minimum);
                fail("IllegalArgumentException not thrown");
            } catch (IllegalArgumentException e) {
            }
        }

        public void test_10arg_negative_retry() {
            try {
                new SOARecord(this.m_an, 1, this.m_ttl, this.m_host, this.m_admin, this.m_serial, this.m_refresh, -1, this.m_expire, this.m_minimum);
                fail("IllegalArgumentException not thrown");
            } catch (IllegalArgumentException e) {
            }
        }

        public void test_10arg_toobig_retry() {
            try {
                new SOARecord(this.m_an, 1, this.m_ttl, this.m_host, this.m_admin, this.m_serial, this.m_refresh, 4294967296L, this.m_expire, this.m_minimum);
                fail("IllegalArgumentException not thrown");
            } catch (IllegalArgumentException e) {
            }
        }

        public void test_10arg_negative_expire() {
            try {
                new SOARecord(this.m_an, 1, this.m_ttl, this.m_host, this.m_admin, this.m_serial, this.m_refresh, this.m_retry, -1, this.m_minimum);
                fail("IllegalArgumentException not thrown");
            } catch (IllegalArgumentException e) {
            }
        }

        public void test_10arg_toobig_expire() {
            try {
                new SOARecord(this.m_an, 1, this.m_ttl, this.m_host, this.m_admin, this.m_serial, this.m_refresh, this.m_retry, 4294967296L, this.m_minimum);
                fail("IllegalArgumentException not thrown");
            } catch (IllegalArgumentException e) {
            }
        }

        public void test_10arg_negative_minimun() {
            try {
                new SOARecord(this.m_an, 1, this.m_ttl, this.m_host, this.m_admin, this.m_serial, this.m_refresh, this.m_retry, this.m_expire, -1);
                fail("IllegalArgumentException not thrown");
            } catch (IllegalArgumentException e) {
            }
        }

        public void test_10arg_toobig_minimum() {
            try {
                new SOARecord(this.m_an, 1, this.m_ttl, this.m_host, this.m_admin, this.m_serial, this.m_refresh, this.m_retry, this.m_expire, 4294967296L);
                fail("IllegalArgumentException not thrown");
            } catch (IllegalArgumentException e) {
            }
        }
    }

    public static class Test_rrFromWire extends TestCase {
        private Name m_admin;
        private long m_expire;
        private Name m_host;
        private long m_minimum;
        private long m_refresh;
        private long m_retry;
        private long m_serial;

        /* access modifiers changed from: protected */
        public void setUp() throws TextParseException, UnknownHostException {
            this.m_host = Name.fromString("M.h.N.");
            this.m_admin = Name.fromString("M.a.n.");
            this.m_serial = 2882400018L;
            this.m_refresh = 3454997044L;
            this.m_retry = 4010947670L;
            this.m_expire = 305419896;
            this.m_minimum = 878082202;
        }

        public void test() throws IOException {
            DNSInput di = new DNSInput(new byte[]{1, 109, 1, 104, 1, 110, 0, 1, 109, 1, 97, 1, 110, 0, -85, -51, -17, Ascii.DC2, -51, -17, Ascii.DC2, 52, -17, Ascii.DC2, 52, 86, Ascii.DC2, 52, 86, 120, 52, 86, 120, -102});
            SOARecord ar = new SOARecord();
            ar.rrFromWire(di);
            assertEquals(this.m_host, ar.getHost());
            assertEquals(this.m_admin, ar.getAdmin());
            assertEquals(this.m_serial, ar.getSerial());
            assertEquals(this.m_refresh, ar.getRefresh());
            assertEquals(this.m_retry, ar.getRetry());
            assertEquals(this.m_expire, ar.getExpire());
            assertEquals(this.m_minimum, ar.getMinimum());
        }
    }

    public static class Test_rdataFromString extends TestCase {
        private Name m_admin;
        private long m_expire;
        private Name m_host;
        private long m_minimum;
        private Name m_origin;
        private long m_refresh;
        private long m_retry;
        private long m_serial;

        /* access modifiers changed from: protected */
        public void setUp() throws TextParseException, UnknownHostException {
            this.m_origin = Name.fromString("O.");
            this.m_host = Name.fromString("M.h", this.m_origin);
            this.m_admin = Name.fromString("M.a.n.");
            this.m_serial = 2882400018L;
            this.m_refresh = 3454997044L;
            this.m_retry = 4010947670L;
            this.m_expire = 305419896;
            this.m_minimum = 878082202;
        }

        public void test_valid() throws IOException {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("M.h ");
            stringBuffer.append(this.m_admin);
            stringBuffer.append(" ");
            stringBuffer.append(this.m_serial);
            stringBuffer.append(" ");
            stringBuffer.append(this.m_refresh);
            stringBuffer.append(" ");
            stringBuffer.append(this.m_retry);
            stringBuffer.append(" ");
            stringBuffer.append(this.m_expire);
            stringBuffer.append(" ");
            stringBuffer.append(this.m_minimum);
            Tokenizer t = new Tokenizer(stringBuffer.toString());
            SOARecord ar = new SOARecord();
            ar.rdataFromString(t, this.m_origin);
            assertEquals(this.m_host, ar.getHost());
            assertEquals(this.m_admin, ar.getAdmin());
            assertEquals(this.m_serial, ar.getSerial());
            assertEquals(this.m_refresh, ar.getRefresh());
            assertEquals(this.m_retry, ar.getRetry());
            assertEquals(this.m_expire, ar.getExpire());
            assertEquals(this.m_minimum, ar.getMinimum());
        }

        public void test_relative_name() throws IOException {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("M.h ");
            stringBuffer.append(this.m_admin);
            stringBuffer.append(" ");
            stringBuffer.append(this.m_serial);
            stringBuffer.append(" ");
            stringBuffer.append(this.m_refresh);
            stringBuffer.append(" ");
            stringBuffer.append(this.m_retry);
            stringBuffer.append(" ");
            stringBuffer.append(this.m_expire);
            stringBuffer.append(" ");
            stringBuffer.append(this.m_minimum);
            try {
                new SOARecord().rdataFromString(new Tokenizer(stringBuffer.toString()), (Name) null);
                fail("RelativeNameException not thrown");
            } catch (RelativeNameException e) {
            }
        }
    }

    public static class Test_rrToString extends TestCase {
        private Name m_admin;
        private Name m_an;
        private long m_expire;
        private Name m_host;
        private long m_minimum;
        private long m_refresh;
        private long m_retry;
        private long m_serial;
        private long m_ttl;

        /* access modifiers changed from: protected */
        public void setUp() throws TextParseException {
            this.m_an = Name.fromString("My.absolute.name.");
            this.m_ttl = 5032;
            this.m_host = Name.fromString("M.h.N.");
            this.m_admin = Name.fromString("M.a.n.");
            this.m_serial = 2882400018L;
            this.m_refresh = 3454997044L;
            this.m_retry = 4010947670L;
            this.m_expire = 305419896;
            this.m_minimum = 878082202;
        }

        public void test_singleLine() {
            Name name = this.m_an;
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append(this.m_host);
            stringBuffer.append(" ");
            stringBuffer.append(this.m_admin);
            stringBuffer.append(" ");
            stringBuffer.append(this.m_serial);
            stringBuffer.append(" ");
            stringBuffer.append(this.m_refresh);
            stringBuffer.append(" ");
            stringBuffer.append(this.m_retry);
            stringBuffer.append(" ");
            stringBuffer.append(this.m_expire);
            stringBuffer.append(" ");
            stringBuffer.append(this.m_minimum);
            assertEquals(stringBuffer.toString(), new SOARecord(name, 1, this.m_ttl, this.m_host, this.m_admin, this.m_serial, this.m_refresh, this.m_retry, this.m_expire, this.m_minimum).rrToString());
        }

        public void test_multiLine() {
            Name name = this.m_an;
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("^.*\\(\\n\\s*");
            stringBuffer.append(this.m_serial);
            stringBuffer.append("\\s*;\\s*serial\\n");
            stringBuffer.append("\\s*");
            stringBuffer.append(this.m_refresh);
            stringBuffer.append("\\s*;\\s*refresh\\n");
            stringBuffer.append("\\s*");
            stringBuffer.append(this.m_retry);
            stringBuffer.append("\\s*;\\s*retry\\n");
            stringBuffer.append("\\s*");
            stringBuffer.append(this.m_expire);
            stringBuffer.append("\\s*;\\s*expire\\n");
            stringBuffer.append("\\s*");
            stringBuffer.append(this.m_minimum);
            stringBuffer.append("\\s*\\)\\s*;\\s*minimum$");
            String re = stringBuffer.toString();
            Options.set("multiline");
            String out = new SOARecord(name, 1, this.m_ttl, this.m_host, this.m_admin, this.m_serial, this.m_refresh, this.m_retry, this.m_expire, this.m_minimum).rrToString();
            Options.unset("multiline");
            assertTrue(out.matches(re));
        }
    }

    public static class Test_rrToWire extends TestCase {
        private Name m_admin;
        private Name m_an;
        private long m_expire;
        private Name m_host;
        private long m_minimum;
        private long m_refresh;
        private long m_retry;
        private long m_serial;
        private long m_ttl;

        /* access modifiers changed from: protected */
        public void setUp() throws TextParseException {
            this.m_an = Name.fromString("My.Abs.Name.");
            this.m_ttl = 5032;
            this.m_host = Name.fromString("M.h.N.");
            this.m_admin = Name.fromString("M.a.n.");
            this.m_serial = 2882400018L;
            this.m_refresh = 3454997044L;
            this.m_retry = 4010947670L;
            this.m_expire = 305419896;
            this.m_minimum = 878082202;
        }

        public void test_canonical() {
            byte[] exp = {1, 109, 1, 104, 1, 110, 0, 1, 109, 1, 97, 1, 110, 0, -85, -51, -17, Ascii.DC2, -51, -17, Ascii.DC2, 52, -17, Ascii.DC2, 52, 86, Ascii.DC2, 52, 86, 120, 52, 86, 120, -102};
            SOARecord ar = new SOARecord(this.m_an, 1, this.m_ttl, this.m_host, this.m_admin, this.m_serial, this.m_refresh, this.m_retry, this.m_expire, this.m_minimum);
            DNSOutput o = new DNSOutput();
            ar.rrToWire(o, (Compression) null, true);
            assertTrue(Arrays.equals(exp, o.toByteArray()));
        }

        public void test_case_sensitive() {
            byte[] exp = {1, 77, 1, 104, 1, 78, 0, 1, 77, 1, 97, 1, 110, 0, -85, -51, -17, Ascii.DC2, -51, -17, Ascii.DC2, 52, -17, Ascii.DC2, 52, 86, Ascii.DC2, 52, 86, 120, 52, 86, 120, -102};
            SOARecord ar = new SOARecord(this.m_an, 1, this.m_ttl, this.m_host, this.m_admin, this.m_serial, this.m_refresh, this.m_retry, this.m_expire, this.m_minimum);
            DNSOutput o = new DNSOutput();
            ar.rrToWire(o, (Compression) null, false);
            assertTrue(Arrays.equals(exp, o.toByteArray()));
        }
    }

    public static Test suite() {
        Class cls;
        Class cls2;
        Class cls3;
        Class cls4;
        Class cls5;
        TestSuite s = new TestSuite();
        if (class$org$xbill$DNS$SOARecordTest$Test_init == null) {
            cls = class$("org.xbill.DNS.SOARecordTest$Test_init");
            class$org$xbill$DNS$SOARecordTest$Test_init = cls;
        } else {
            cls = class$org$xbill$DNS$SOARecordTest$Test_init;
        }
        s.addTestSuite(cls);
        if (class$org$xbill$DNS$SOARecordTest$Test_rrFromWire == null) {
            cls2 = class$("org.xbill.DNS.SOARecordTest$Test_rrFromWire");
            class$org$xbill$DNS$SOARecordTest$Test_rrFromWire = cls2;
        } else {
            cls2 = class$org$xbill$DNS$SOARecordTest$Test_rrFromWire;
        }
        s.addTestSuite(cls2);
        if (class$org$xbill$DNS$SOARecordTest$Test_rdataFromString == null) {
            cls3 = class$("org.xbill.DNS.SOARecordTest$Test_rdataFromString");
            class$org$xbill$DNS$SOARecordTest$Test_rdataFromString = cls3;
        } else {
            cls3 = class$org$xbill$DNS$SOARecordTest$Test_rdataFromString;
        }
        s.addTestSuite(cls3);
        if (class$org$xbill$DNS$SOARecordTest$Test_rrToString == null) {
            cls4 = class$("org.xbill.DNS.SOARecordTest$Test_rrToString");
            class$org$xbill$DNS$SOARecordTest$Test_rrToString = cls4;
        } else {
            cls4 = class$org$xbill$DNS$SOARecordTest$Test_rrToString;
        }
        s.addTestSuite(cls4);
        if (class$org$xbill$DNS$SOARecordTest$Test_rrToWire == null) {
            cls5 = class$("org.xbill.DNS.SOARecordTest$Test_rrToWire");
            class$org$xbill$DNS$SOARecordTest$Test_rrToWire = cls5;
        } else {
            cls5 = class$org$xbill$DNS$SOARecordTest$Test_rrToWire;
        }
        s.addTestSuite(cls5);
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
