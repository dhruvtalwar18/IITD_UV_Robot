package org.xbill.DNS;

import java.io.IOException;
import junit.framework.Test;
import junit.framework.TestCase;
import junit.framework.TestSuite;
import org.bytedeco.javacpp.opencv_stitching;

public class GPOSRecordTest extends TestCase {
    static /* synthetic */ Class class$org$xbill$DNS$GPOSRecordTest;
    static /* synthetic */ Class class$org$xbill$DNS$GPOSRecordTest$Test_Ctor_6arg_Strings;
    static /* synthetic */ Class class$org$xbill$DNS$GPOSRecordTest$Test_Ctor_6arg_doubles;
    static /* synthetic */ Class class$org$xbill$DNS$GPOSRecordTest$Test_rdataFromString;
    static /* synthetic */ Class class$org$xbill$DNS$GPOSRecordTest$Test_rrFromWire;

    public void test_ctor_0arg() {
        GPOSRecord gr = new GPOSRecord();
        assertNull(gr.getName());
        assertEquals(0, gr.getType());
        assertEquals(0, gr.getDClass());
        assertEquals(0, gr.getTTL());
    }

    public void test_getObject() {
        assertTrue(new GPOSRecord().getObject() instanceof GPOSRecord);
    }

    public static class Test_Ctor_6arg_doubles extends TestCase {
        private double m_alt;
        private double m_lat;
        private double m_long;
        private Name m_n;
        private long m_ttl;

        /* access modifiers changed from: protected */
        public void setUp() throws TextParseException {
            this.m_n = Name.fromString("The.Name.");
            this.m_ttl = 43981;
            this.m_lat = -10.43d;
            this.m_long = 76.12d;
            this.m_alt = 100.101d;
        }

        public void test_basic() throws TextParseException {
            GPOSRecord gr = new GPOSRecord(this.m_n, 1, this.m_ttl, this.m_long, this.m_lat, this.m_alt);
            assertEquals(this.m_n, gr.getName());
            assertEquals(1, gr.getDClass());
            assertEquals(27, gr.getType());
            assertEquals(this.m_ttl, gr.getTTL());
            assertEquals(new Double(this.m_long), new Double(gr.getLongitude()));
            assertEquals(new Double(this.m_lat), new Double(gr.getLatitude()));
            assertEquals(new Double(this.m_alt), new Double(gr.getAltitude()));
            assertEquals(new Double(this.m_long).toString(), gr.getLongitudeString());
            assertEquals(new Double(this.m_lat).toString(), gr.getLatitudeString());
            assertEquals(new Double(this.m_alt).toString(), gr.getAltitudeString());
        }

        public void test_toosmall_longitude() throws TextParseException {
            try {
                new GPOSRecord(this.m_n, 1, this.m_ttl, -90.001d, this.m_lat, this.m_alt);
                fail("IllegalArgumentException not thrown");
            } catch (IllegalArgumentException e) {
            }
        }

        public void test_toobig_longitude() throws TextParseException {
            try {
                new GPOSRecord(this.m_n, 1, this.m_ttl, 90.001d, this.m_lat, this.m_alt);
                fail("IllegalArgumentException not thrown");
            } catch (IllegalArgumentException e) {
            }
        }

        public void test_toosmall_latitude() throws TextParseException {
            try {
                new GPOSRecord(this.m_n, 1, this.m_ttl, this.m_long, -180.001d, this.m_alt);
                fail("IllegalArgumentException not thrown");
            } catch (IllegalArgumentException e) {
            }
        }

        public void test_toobig_latitude() throws TextParseException {
            try {
                new GPOSRecord(this.m_n, 1, this.m_ttl, this.m_long, 180.001d, this.m_alt);
                fail("IllegalArgumentException not thrown");
            } catch (IllegalArgumentException e) {
            }
        }

        public void test_invalid_string() {
            try {
                new GPOSRecord(this.m_n, 1, this.m_ttl, new Double(this.m_long).toString(), "120.\\00ABC", new Double(this.m_alt).toString());
                fail("IllegalArgumentException not thrown");
            } catch (IllegalArgumentException e) {
            }
        }
    }

    public static class Test_Ctor_6arg_Strings extends TestCase {
        private double m_alt;
        private double m_lat;
        private double m_long;
        private Name m_n;
        private long m_ttl;

        /* access modifiers changed from: protected */
        public void setUp() throws TextParseException {
            this.m_n = Name.fromString("The.Name.");
            this.m_ttl = 43981;
            this.m_lat = -10.43d;
            this.m_long = 76.12d;
            this.m_alt = 100.101d;
        }

        public void test_basic() throws TextParseException {
            GPOSRecord gr = new GPOSRecord(this.m_n, 1, this.m_ttl, new Double(this.m_long).toString(), new Double(this.m_lat).toString(), new Double(this.m_alt).toString());
            assertEquals(this.m_n, gr.getName());
            assertEquals(1, gr.getDClass());
            assertEquals(27, gr.getType());
            assertEquals(this.m_ttl, gr.getTTL());
            assertEquals(new Double(this.m_long), new Double(gr.getLongitude()));
            assertEquals(new Double(this.m_lat), new Double(gr.getLatitude()));
            assertEquals(new Double(this.m_alt), new Double(gr.getAltitude()));
            assertEquals(new Double(this.m_long).toString(), gr.getLongitudeString());
            assertEquals(new Double(this.m_lat).toString(), gr.getLatitudeString());
            assertEquals(new Double(this.m_alt).toString(), gr.getAltitudeString());
        }

        public void test_toosmall_longitude() throws TextParseException {
            try {
                new GPOSRecord(this.m_n, 1, this.m_ttl, "-90.001", new Double(this.m_lat).toString(), new Double(this.m_alt).toString());
                fail("IllegalArgumentException not thrown");
            } catch (IllegalArgumentException e) {
            }
        }

        public void test_toobig_longitude() throws TextParseException {
            try {
                new GPOSRecord(this.m_n, 1, this.m_ttl, "90.001", new Double(this.m_lat).toString(), new Double(this.m_alt).toString());
                fail("IllegalArgumentException not thrown");
            } catch (IllegalArgumentException e) {
            }
        }

        public void test_toosmall_latitude() throws TextParseException {
            try {
                new GPOSRecord(this.m_n, 1, this.m_ttl, new Double(this.m_long).toString(), "-180.001", new Double(this.m_alt).toString());
                fail("IllegalArgumentException not thrown");
            } catch (IllegalArgumentException e) {
            }
        }

        public void test_toobig_latitude() throws TextParseException {
            try {
                new GPOSRecord(this.m_n, 1, this.m_ttl, new Double(this.m_long).toString(), "180.001", new Double(this.m_alt).toString());
                fail("IllegalArgumentException not thrown");
            } catch (IllegalArgumentException e) {
            }
        }
    }

    public static class Test_rrFromWire extends TestCase {
        public void test_basic() throws IOException {
            DNSInput in = new DNSInput(new byte[]{5, 45, 56, 46, 49, 50, 6, 49, 50, 51, 46, 48, 55, 3, 48, 46, 48});
            GPOSRecord gr = new GPOSRecord();
            gr.rrFromWire(in);
            assertEquals(new Double(-8.12d), new Double(gr.getLongitude()));
            assertEquals(new Double(123.07d), new Double(gr.getLatitude()));
            assertEquals(new Double(opencv_stitching.Stitcher.ORIG_RESOL), new Double(gr.getAltitude()));
        }

        public void test_longitude_toosmall() throws IOException {
            try {
                new GPOSRecord().rrFromWire(new DNSInput(new byte[]{5, 45, 57, 53, 46, 48, 6, 49, 50, 51, 46, 48, 55, 3, 48, 46, 48}));
                fail("WireParseException not thrown");
            } catch (WireParseException e) {
            }
        }

        public void test_longitude_toobig() throws IOException {
            try {
                new GPOSRecord().rrFromWire(new DNSInput(new byte[]{5, 49, 56, 53, 46, 48, 6, 49, 50, 51, 46, 48, 55, 3, 48, 46, 48}));
                fail("WireParseException not thrown");
            } catch (WireParseException e) {
            }
        }

        public void test_latitude_toosmall() throws IOException {
            try {
                new GPOSRecord().rrFromWire(new DNSInput(new byte[]{5, 45, 56, 53, 46, 48, 6, 45, 49, 57, 48, 46, 48, 3, 48, 46, 48}));
                fail("WireParseException not thrown");
            } catch (WireParseException e) {
            }
        }

        public void test_latitude_toobig() throws IOException {
            try {
                new GPOSRecord().rrFromWire(new DNSInput(new byte[]{5, 45, 56, 53, 46, 48, 6, 50, 49, 57, 48, 46, 48, 3, 48, 46, 48}));
                fail("WireParseException not thrown");
            } catch (WireParseException e) {
            }
        }
    }

    public static class Test_rdataFromString extends TestCase {
        public void test_basic() throws IOException {
            Tokenizer t = new Tokenizer("10.45 171.121212 1010787");
            GPOSRecord gr = new GPOSRecord();
            gr.rdataFromString(t, (Name) null);
            assertEquals(new Double(10.45d), new Double(gr.getLongitude()));
            assertEquals(new Double(171.121212d), new Double(gr.getLatitude()));
            assertEquals(new Double(1010787.0d), new Double(gr.getAltitude()));
        }

        public void test_longitude_toosmall() throws IOException {
            try {
                new GPOSRecord().rdataFromString(new Tokenizer("-100.390 171.121212 1010787"), (Name) null);
                fail("IOException not thrown");
            } catch (IOException e) {
            }
        }

        public void test_longitude_toobig() throws IOException {
            try {
                new GPOSRecord().rdataFromString(new Tokenizer("90.00001 171.121212 1010787"), (Name) null);
                fail("IOException not thrown");
            } catch (IOException e) {
            }
        }

        public void test_latitude_toosmall() throws IOException {
            try {
                new GPOSRecord().rdataFromString(new Tokenizer("0.0 -180.01 1010787"), (Name) null);
                fail("IOException not thrown");
            } catch (IOException e) {
            }
        }

        public void test_latitude_toobig() throws IOException {
            try {
                new GPOSRecord().rdataFromString(new Tokenizer("0.0 180.01 1010787"), (Name) null);
                fail("IOException not thrown");
            } catch (IOException e) {
            }
        }

        public void test_invalid_string() throws IOException {
            try {
                new GPOSRecord().rdataFromString(new Tokenizer("1.0 2.0 \\435"), (Name) null);
            } catch (TextParseException e) {
            }
        }
    }

    public void test_rrToString() throws TextParseException {
        assertEquals("\"10.45\" \"171.121212\" \"1010787.0\"", new GPOSRecord(Name.fromString("The.Name."), 1, 291, 10.45d, 171.121212d, 1010787.0d).rrToString());
    }

    public void test_rrToWire() throws TextParseException {
        GPOSRecord gr = new GPOSRecord(Name.fromString("The.Name."), 1, 291, -10.45d, 120.0d, 111.0d);
        byte[] exp = {6, 45, 49, 48, 46, 52, 53, 5, 49, 50, 48, 46, 48, 5, 49, 49, 49, 46, 48};
        DNSOutput out = new DNSOutput();
        gr.rrToWire(out, (Compression) null, true);
        byte[] bar = out.toByteArray();
        assertEquals(exp.length, bar.length);
        for (int i = 0; i < exp.length; i++) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("i=");
            stringBuffer.append(i);
            assertEquals(stringBuffer.toString(), exp[i], bar[i]);
        }
    }

    public static Test suite() {
        Class cls;
        Class cls2;
        Class cls3;
        Class cls4;
        Class cls5;
        TestSuite s = new TestSuite();
        if (class$org$xbill$DNS$GPOSRecordTest$Test_Ctor_6arg_doubles == null) {
            cls = class$("org.xbill.DNS.GPOSRecordTest$Test_Ctor_6arg_doubles");
            class$org$xbill$DNS$GPOSRecordTest$Test_Ctor_6arg_doubles = cls;
        } else {
            cls = class$org$xbill$DNS$GPOSRecordTest$Test_Ctor_6arg_doubles;
        }
        s.addTestSuite(cls);
        if (class$org$xbill$DNS$GPOSRecordTest$Test_Ctor_6arg_Strings == null) {
            cls2 = class$("org.xbill.DNS.GPOSRecordTest$Test_Ctor_6arg_Strings");
            class$org$xbill$DNS$GPOSRecordTest$Test_Ctor_6arg_Strings = cls2;
        } else {
            cls2 = class$org$xbill$DNS$GPOSRecordTest$Test_Ctor_6arg_Strings;
        }
        s.addTestSuite(cls2);
        if (class$org$xbill$DNS$GPOSRecordTest$Test_rrFromWire == null) {
            cls3 = class$("org.xbill.DNS.GPOSRecordTest$Test_rrFromWire");
            class$org$xbill$DNS$GPOSRecordTest$Test_rrFromWire = cls3;
        } else {
            cls3 = class$org$xbill$DNS$GPOSRecordTest$Test_rrFromWire;
        }
        s.addTestSuite(cls3);
        if (class$org$xbill$DNS$GPOSRecordTest$Test_rdataFromString == null) {
            cls4 = class$("org.xbill.DNS.GPOSRecordTest$Test_rdataFromString");
            class$org$xbill$DNS$GPOSRecordTest$Test_rdataFromString = cls4;
        } else {
            cls4 = class$org$xbill$DNS$GPOSRecordTest$Test_rdataFromString;
        }
        s.addTestSuite(cls4);
        if (class$org$xbill$DNS$GPOSRecordTest == null) {
            cls5 = class$("org.xbill.DNS.GPOSRecordTest");
            class$org$xbill$DNS$GPOSRecordTest = cls5;
        } else {
            cls5 = class$org$xbill$DNS$GPOSRecordTest;
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
