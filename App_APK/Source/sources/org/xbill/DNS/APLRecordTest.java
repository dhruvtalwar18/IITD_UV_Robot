package org.xbill.DNS;

import com.google.common.base.Ascii;
import com.google.common.primitives.SignedBytes;
import com.google.common.primitives.UnsignedBytes;
import java.io.IOException;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import junit.framework.Test;
import junit.framework.TestCase;
import junit.framework.TestSuite;
import org.xbill.DNS.APLRecord;
import rocon_app_manager_msgs.ErrorCodes;

public class APLRecordTest {
    static /* synthetic */ Class class$org$xbill$DNS$APLRecordTest$Test_Element_init;
    static /* synthetic */ Class class$org$xbill$DNS$APLRecordTest$Test_init;
    static /* synthetic */ Class class$org$xbill$DNS$APLRecordTest$Test_rdataFromString;
    static /* synthetic */ Class class$org$xbill$DNS$APLRecordTest$Test_rrFromWire;
    static /* synthetic */ Class class$org$xbill$DNS$APLRecordTest$Test_rrToString;
    static /* synthetic */ Class class$org$xbill$DNS$APLRecordTest$Test_rrToWire;

    public static class Test_Element_init extends TestCase {
        InetAddress m_addr4;
        InetAddress m_addr6;

        /* access modifiers changed from: protected */
        public void setUp() throws TextParseException, UnknownHostException {
            this.m_addr4 = InetAddress.getByName("193.160.232.5");
            this.m_addr6 = InetAddress.getByName("2001:db8:85a3:8d3:1319:8a2e:370:7334");
        }

        public void test_valid_IPv4() {
            APLRecord.Element el = new APLRecord.Element(true, this.m_addr4, 16);
            assertEquals(1, el.family);
            assertEquals(true, el.negative);
            assertEquals(this.m_addr4, el.address);
            assertEquals(16, el.prefixLength);
        }

        public void test_invalid_IPv4() {
            try {
                new APLRecord.Element(true, this.m_addr4, 33);
                fail("IllegalArgumentException not thrown");
            } catch (IllegalArgumentException e) {
            }
        }

        public void test_valid_IPv6() {
            APLRecord.Element el = new APLRecord.Element(false, this.m_addr6, 74);
            assertEquals(2, el.family);
            assertEquals(false, el.negative);
            assertEquals(this.m_addr6, el.address);
            assertEquals(74, el.prefixLength);
        }

        public void test_invalid_IPv6() {
            try {
                new APLRecord.Element(true, this.m_addr6, 129);
                fail("IllegalArgumentException not thrown");
            } catch (IllegalArgumentException e) {
            }
        }
    }

    public static class Test_init extends TestCase {
        InetAddress m_addr4;
        byte[] m_addr4_bytes;
        String m_addr4_string;
        InetAddress m_addr6;
        byte[] m_addr6_bytes;
        String m_addr6_string;
        Name m_an;
        ArrayList m_elements;
        Name m_rn;
        long m_ttl;

        /* access modifiers changed from: protected */
        public void setUp() throws TextParseException, UnknownHostException {
            this.m_an = Name.fromString("My.Absolute.Name.");
            this.m_rn = Name.fromString("My.Relative.Name");
            this.m_ttl = 79225;
            this.m_addr4_string = "193.160.232.5";
            this.m_addr4 = InetAddress.getByName(this.m_addr4_string);
            this.m_addr4_bytes = this.m_addr4.getAddress();
            this.m_addr6_string = "2001:db8:85a3:8d3:1319:8a2e:370:7334";
            this.m_addr6 = InetAddress.getByName(this.m_addr6_string);
            this.m_addr6_bytes = this.m_addr6.getAddress();
            this.m_elements = new ArrayList(2);
            this.m_elements.add(new APLRecord.Element(true, this.m_addr4, 12));
            this.m_elements.add(new APLRecord.Element(false, this.m_addr6, 64));
        }

        public void test_0arg() throws UnknownHostException {
            APLRecord ar = new APLRecord();
            assertNull(ar.getName());
            assertEquals(0, ar.getType());
            assertEquals(0, ar.getDClass());
            assertEquals(0, ar.getTTL());
            assertNull(ar.getElements());
        }

        public void test_getObject() {
            assertTrue(new APLRecord().getObject() instanceof APLRecord);
        }

        public void test_4arg_basic() {
            APLRecord ar = new APLRecord(this.m_an, 1, this.m_ttl, this.m_elements);
            assertEquals(this.m_an, ar.getName());
            assertEquals(42, ar.getType());
            assertEquals(1, ar.getDClass());
            assertEquals(this.m_ttl, ar.getTTL());
            assertEquals(this.m_elements, ar.getElements());
        }

        public void test_4arg_empty_elements() {
            assertEquals(new ArrayList(), new APLRecord(this.m_an, 1, this.m_ttl, new ArrayList()).getElements());
        }

        public void test_4arg_relative_name() {
            try {
                new APLRecord(this.m_rn, 1, this.m_ttl, this.m_elements);
                fail("RelativeNameException not thrown");
            } catch (RelativeNameException e) {
            }
        }

        public void test_4arg_invalid_elements() {
            this.m_elements = new ArrayList();
            this.m_elements.add(new Object());
            try {
                new APLRecord(this.m_an, 1, this.m_ttl, this.m_elements);
                fail("IllegalArgumentException not thrown");
            } catch (IllegalArgumentException e) {
            }
        }
    }

    public static class Test_rrFromWire extends TestCase {
        InetAddress m_addr4;
        byte[] m_addr4_bytes;
        InetAddress m_addr6;
        byte[] m_addr6_bytes;

        /* access modifiers changed from: protected */
        public void setUp() throws TextParseException, UnknownHostException {
            this.m_addr4 = InetAddress.getByName("193.160.232.5");
            this.m_addr4_bytes = this.m_addr4.getAddress();
            this.m_addr6 = InetAddress.getByName("2001:db8:85a3:8d3:1319:8a2e:370:7334");
            this.m_addr6_bytes = this.m_addr6.getAddress();
        }

        public void test_validIPv4() throws IOException {
            DNSInput di = new DNSInput(new byte[]{0, 1, 8, -124, this.m_addr4_bytes[0], this.m_addr4_bytes[1], this.m_addr4_bytes[2], this.m_addr4_bytes[3]});
            APLRecord ar = new APLRecord();
            ar.rrFromWire(di);
            ArrayList exp = new ArrayList();
            exp.add(new APLRecord.Element(true, this.m_addr4, 8));
            assertEquals(exp, ar.getElements());
        }

        public void test_validIPv4_short_address() throws IOException {
            DNSInput di = new DNSInput(new byte[]{0, 1, 20, -125, this.m_addr4_bytes[0], this.m_addr4_bytes[1], this.m_addr4_bytes[2]});
            APLRecord ar = new APLRecord();
            ar.rrFromWire(di);
            InetAddress a = InetAddress.getByName("193.160.232.0");
            ArrayList exp = new ArrayList();
            exp.add(new APLRecord.Element(true, a, 20));
            assertEquals(exp, ar.getElements());
        }

        public void test_invalid_IPv4_prefix() throws IOException {
            try {
                new APLRecord().rrFromWire(new DNSInput(new byte[]{0, 1, ErrorCodes.NOT_CURRENT_REMOTE_CONTROLLER, -124, this.m_addr4_bytes[0], this.m_addr4_bytes[1], this.m_addr4_bytes[2], this.m_addr4_bytes[3]}));
                fail("WireParseException not thrown");
            } catch (WireParseException e) {
            }
        }

        public void test_invalid_IPv4_length() throws IOException {
            try {
                new APLRecord().rrFromWire(new DNSInput(new byte[]{0, 1, 8, -123, this.m_addr4_bytes[0], this.m_addr4_bytes[1], this.m_addr4_bytes[2], this.m_addr4_bytes[3], 10}));
                fail("WireParseException not thrown");
            } catch (WireParseException e) {
            }
        }

        public void test_multiple_validIPv4() throws IOException {
            DNSInput di = new DNSInput(new byte[]{0, 1, 8, -124, this.m_addr4_bytes[0], this.m_addr4_bytes[1], this.m_addr4_bytes[2], this.m_addr4_bytes[3], 0, 1, 30, 4, this.m_addr4_bytes[0], this.m_addr4_bytes[1], this.m_addr4_bytes[2], this.m_addr4_bytes[3]});
            APLRecord ar = new APLRecord();
            ar.rrFromWire(di);
            ArrayList exp = new ArrayList();
            exp.add(new APLRecord.Element(true, this.m_addr4, 8));
            exp.add(new APLRecord.Element(false, this.m_addr4, 30));
            assertEquals(exp, ar.getElements());
        }

        public void test_validIPv6() throws IOException {
            DNSInput di = new DNSInput(new byte[]{0, 2, 115, 16, this.m_addr6_bytes[0], this.m_addr6_bytes[1], this.m_addr6_bytes[2], this.m_addr6_bytes[3], this.m_addr6_bytes[4], this.m_addr6_bytes[5], this.m_addr6_bytes[6], this.m_addr6_bytes[7], this.m_addr6_bytes[8], this.m_addr6_bytes[9], this.m_addr6_bytes[10], this.m_addr6_bytes[11], this.m_addr6_bytes[12], this.m_addr6_bytes[13], this.m_addr6_bytes[14], this.m_addr6_bytes[15]});
            APLRecord ar = new APLRecord();
            ar.rrFromWire(di);
            ArrayList exp = new ArrayList();
            exp.add(new APLRecord.Element(false, this.m_addr6, 115));
            assertEquals(exp, ar.getElements());
        }

        public void test_valid_nonIP() throws IOException {
            DNSInput di = new DNSInput(new byte[]{0, 3, -126, -123, 1, 2, 3, 4, 5});
            APLRecord ar = new APLRecord();
            ar.rrFromWire(di);
            List l = ar.getElements();
            assertEquals(1, l.size());
            APLRecord.Element el = (APLRecord.Element) l.get(0);
            assertEquals(3, el.family);
            assertEquals(true, el.negative);
            assertEquals(130, el.prefixLength);
            assertTrue(Arrays.equals(new byte[]{1, 2, 3, 4, 5}, (byte[]) el.address));
        }
    }

    public static class Test_rdataFromString extends TestCase {
        InetAddress m_addr4;
        byte[] m_addr4_bytes;
        String m_addr4_string;
        InetAddress m_addr6;
        byte[] m_addr6_bytes;
        String m_addr6_string;

        /* access modifiers changed from: protected */
        public void setUp() throws TextParseException, UnknownHostException {
            this.m_addr4_string = "193.160.232.5";
            this.m_addr4 = InetAddress.getByName(this.m_addr4_string);
            this.m_addr4_bytes = this.m_addr4.getAddress();
            this.m_addr6_string = "2001:db8:85a3:8d3:1319:8a2e:370:7334";
            this.m_addr6 = InetAddress.getByName(this.m_addr6_string);
            this.m_addr6_bytes = this.m_addr6.getAddress();
        }

        public void test_validIPv4() throws IOException {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("1:");
            stringBuffer.append(this.m_addr4_string);
            stringBuffer.append("/11\n");
            Tokenizer t = new Tokenizer(stringBuffer.toString());
            APLRecord ar = new APLRecord();
            ar.rdataFromString(t, (Name) null);
            ArrayList exp = new ArrayList();
            exp.add(new APLRecord.Element(false, this.m_addr4, 11));
            assertEquals(exp, ar.getElements());
            assertEquals(1, t.get().type);
        }

        public void test_valid_multi() throws IOException {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("1:");
            stringBuffer.append(this.m_addr4_string);
            stringBuffer.append("/11 !2:");
            stringBuffer.append(this.m_addr6_string);
            stringBuffer.append("/100");
            Tokenizer t = new Tokenizer(stringBuffer.toString());
            APLRecord ar = new APLRecord();
            ar.rdataFromString(t, (Name) null);
            ArrayList exp = new ArrayList();
            exp.add(new APLRecord.Element(false, this.m_addr4, 11));
            exp.add(new APLRecord.Element(true, this.m_addr6, 100));
            assertEquals(exp, ar.getElements());
        }

        public void test_validIPv6() throws IOException {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("!2:");
            stringBuffer.append(this.m_addr6_string);
            stringBuffer.append("/36\n");
            Tokenizer t = new Tokenizer(stringBuffer.toString());
            APLRecord ar = new APLRecord();
            ar.rdataFromString(t, (Name) null);
            ArrayList exp = new ArrayList();
            exp.add(new APLRecord.Element(true, this.m_addr6, 36));
            assertEquals(exp, ar.getElements());
            assertEquals(1, t.get().type);
        }

        public void test_no_colon() throws IOException {
            try {
                new APLRecord().rdataFromString(new Tokenizer("!1192.68.0.1/20"), (Name) null);
                fail("TextParseException not thrown");
            } catch (TextParseException e) {
            }
        }

        public void test_colon_and_slash_swapped() throws IOException {
            try {
                new APLRecord().rdataFromString(new Tokenizer("!1/192.68.0.1:20"), (Name) null);
                fail("TextParseException not thrown");
            } catch (TextParseException e) {
            }
        }

        public void test_no_slash() throws IOException {
            try {
                new APLRecord().rdataFromString(new Tokenizer("!1:192.68.0.1|20"), (Name) null);
                fail("TextParseException not thrown");
            } catch (TextParseException e) {
            }
        }

        public void test_empty_family() throws IOException {
            try {
                new APLRecord().rdataFromString(new Tokenizer("!:192.68.0.1/20"), (Name) null);
                fail("TextParseException not thrown");
            } catch (TextParseException e) {
            }
        }

        public void test_malformed_family() throws IOException {
            try {
                new APLRecord().rdataFromString(new Tokenizer("family:192.68.0.1/20"), (Name) null);
                fail("TextParseException not thrown");
            } catch (TextParseException e) {
            }
        }

        public void test_invalid_family() throws IOException {
            try {
                new APLRecord().rdataFromString(new Tokenizer("3:192.68.0.1/20"), (Name) null);
                fail("TextParseException not thrown");
            } catch (TextParseException e) {
            }
        }

        public void test_empty_prefix() throws IOException {
            try {
                new APLRecord().rdataFromString(new Tokenizer("1:192.68.0.1/"), (Name) null);
                fail("TextParseException not thrown");
            } catch (TextParseException e) {
            }
        }

        public void test_malformed_prefix() throws IOException {
            try {
                new APLRecord().rdataFromString(new Tokenizer("1:192.68.0.1/prefix"), (Name) null);
                fail("TextParseException not thrown");
            } catch (TextParseException e) {
            }
        }

        public void test_invalid_prefix() throws IOException {
            try {
                new APLRecord().rdataFromString(new Tokenizer("1:192.68.0.1/33"), (Name) null);
                fail("TextParseException not thrown");
            } catch (TextParseException e) {
            }
        }

        public void test_empty_address() throws IOException {
            try {
                new APLRecord().rdataFromString(new Tokenizer("1:/33"), (Name) null);
                fail("TextParseException not thrown");
            } catch (TextParseException e) {
            }
        }

        public void test_malformed_address() throws IOException {
            try {
                new APLRecord().rdataFromString(new Tokenizer("1:A.B.C.D/33"), (Name) null);
                fail("TextParseException not thrown");
            } catch (TextParseException e) {
            }
        }
    }

    public static class Test_rrToString extends TestCase {
        InetAddress m_addr4;
        byte[] m_addr4_bytes;
        String m_addr4_string;
        InetAddress m_addr6;
        byte[] m_addr6_bytes;
        String m_addr6_string;
        Name m_an;
        ArrayList m_elements;
        Name m_rn;
        long m_ttl;

        /* access modifiers changed from: protected */
        public void setUp() throws TextParseException, UnknownHostException {
            this.m_an = Name.fromString("My.Absolute.Name.");
            this.m_rn = Name.fromString("My.Relative.Name");
            this.m_ttl = 79225;
            this.m_addr4_string = "193.160.232.5";
            this.m_addr4 = InetAddress.getByName(this.m_addr4_string);
            this.m_addr4_bytes = this.m_addr4.getAddress();
            this.m_addr6_string = "2001:db8:85a3:8d3:1319:8a2e:370:7334";
            this.m_addr6 = InetAddress.getByName(this.m_addr6_string);
            this.m_addr6_bytes = this.m_addr6.getAddress();
            this.m_elements = new ArrayList(2);
            this.m_elements.add(new APLRecord.Element(true, this.m_addr4, 12));
            this.m_elements.add(new APLRecord.Element(false, this.m_addr6, 64));
        }

        public void test() {
            APLRecord ar = new APLRecord(this.m_an, 1, this.m_ttl, this.m_elements);
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("!1:");
            stringBuffer.append(this.m_addr4_string);
            stringBuffer.append("/12 2:");
            stringBuffer.append(this.m_addr6_string);
            stringBuffer.append("/64");
            assertEquals(stringBuffer.toString(), ar.rrToString());
        }
    }

    public static class Test_rrToWire extends TestCase {
        InetAddress m_addr4;
        byte[] m_addr4_bytes;
        String m_addr4_string;
        InetAddress m_addr6;
        byte[] m_addr6_bytes;
        String m_addr6_string;
        Name m_an;
        ArrayList m_elements;
        Name m_rn;
        long m_ttl;

        /* access modifiers changed from: protected */
        public void setUp() throws TextParseException, UnknownHostException {
            this.m_an = Name.fromString("My.Absolute.Name.");
            this.m_rn = Name.fromString("My.Relative.Name");
            this.m_ttl = 79225;
            this.m_addr4_string = "193.160.232.5";
            this.m_addr4 = InetAddress.getByName(this.m_addr4_string);
            this.m_addr4_bytes = this.m_addr4.getAddress();
            this.m_addr6_string = "2001:db8:85a3:8d3:1319:8a2e:370:7334";
            this.m_addr6 = InetAddress.getByName(this.m_addr6_string);
            this.m_addr6_bytes = this.m_addr6.getAddress();
            this.m_elements = new ArrayList(2);
            this.m_elements.add(new APLRecord.Element(true, this.m_addr4, 12));
            this.m_elements.add(new APLRecord.Element(false, this.m_addr6, 64));
        }

        public void test_empty() {
            APLRecord ar = new APLRecord(this.m_an, 1, this.m_ttl, new ArrayList());
            DNSOutput dout = new DNSOutput();
            ar.rrToWire(dout, (Compression) null, true);
            assertTrue(Arrays.equals(new byte[0], dout.toByteArray()));
        }

        public void test_basic() {
            APLRecord ar = new APLRecord(this.m_an, 1, this.m_ttl, this.m_elements);
            byte[] exp = {0, 1, Ascii.FF, -124, this.m_addr4_bytes[0], this.m_addr4_bytes[1], this.m_addr4_bytes[2], this.m_addr4_bytes[3], 0, 2, SignedBytes.MAX_POWER_OF_TWO, 16, this.m_addr6_bytes[0], this.m_addr6_bytes[1], this.m_addr6_bytes[2], this.m_addr6_bytes[3], this.m_addr6_bytes[4], this.m_addr6_bytes[5], this.m_addr6_bytes[6], this.m_addr6_bytes[7], this.m_addr6_bytes[8], this.m_addr6_bytes[9], this.m_addr6_bytes[10], this.m_addr6_bytes[11], this.m_addr6_bytes[12], this.m_addr6_bytes[13], this.m_addr6_bytes[14], this.m_addr6_bytes[15]};
            DNSOutput dout = new DNSOutput();
            ar.rrToWire(dout, (Compression) null, true);
            assertTrue(Arrays.equals(exp, dout.toByteArray()));
        }

        public void test_non_IP() throws IOException {
            byte[] exp = {0, 3, -126, -123, 1, 2, 3, 4, 5};
            DNSInput di = new DNSInput(exp);
            APLRecord ar = new APLRecord();
            ar.rrFromWire(di);
            DNSOutput dout = new DNSOutput();
            ar.rrToWire(dout, (Compression) null, true);
            assertTrue(Arrays.equals(exp, dout.toByteArray()));
        }

        public void test_address_with_embedded_zero() throws UnknownHostException {
            InetAddress a = InetAddress.getByName("232.0.11.1");
            ArrayList elements = new ArrayList();
            elements.add(new APLRecord.Element(true, a, 31));
            APLRecord ar = new APLRecord(this.m_an, 1, this.m_ttl, elements);
            DNSOutput dout = new DNSOutput();
            ar.rrToWire(dout, (Compression) null, true);
            assertTrue(Arrays.equals(new byte[]{0, 1, 31, -124, -24, 0, 11, 1}, dout.toByteArray()));
        }

        public void test_short_address() throws UnknownHostException {
            InetAddress a = InetAddress.getByName("232.0.11.0");
            ArrayList elements = new ArrayList();
            elements.add(new APLRecord.Element(true, a, 31));
            APLRecord ar = new APLRecord(this.m_an, 1, this.m_ttl, elements);
            DNSOutput dout = new DNSOutput();
            ar.rrToWire(dout, (Compression) null, true);
            assertTrue(Arrays.equals(new byte[]{0, 1, 31, -125, -24, 0, 11}, dout.toByteArray()));
        }

        public void test_wildcard_address() throws UnknownHostException {
            InetAddress a = InetAddress.getByName("0.0.0.0");
            ArrayList elements = new ArrayList();
            elements.add(new APLRecord.Element(true, a, 31));
            APLRecord ar = new APLRecord(this.m_an, 1, this.m_ttl, elements);
            byte[] exp = {0, 1, 31, UnsignedBytes.MAX_POWER_OF_TWO};
            DNSOutput dout = new DNSOutput();
            ar.rrToWire(dout, (Compression) null, true);
            assertTrue(Arrays.equals(exp, dout.toByteArray()));
        }
    }

    public static Test suite() {
        Class cls;
        Class cls2;
        Class cls3;
        Class cls4;
        Class cls5;
        Class cls6;
        TestSuite s = new TestSuite();
        if (class$org$xbill$DNS$APLRecordTest$Test_Element_init == null) {
            cls = class$("org.xbill.DNS.APLRecordTest$Test_Element_init");
            class$org$xbill$DNS$APLRecordTest$Test_Element_init = cls;
        } else {
            cls = class$org$xbill$DNS$APLRecordTest$Test_Element_init;
        }
        s.addTestSuite(cls);
        if (class$org$xbill$DNS$APLRecordTest$Test_init == null) {
            cls2 = class$("org.xbill.DNS.APLRecordTest$Test_init");
            class$org$xbill$DNS$APLRecordTest$Test_init = cls2;
        } else {
            cls2 = class$org$xbill$DNS$APLRecordTest$Test_init;
        }
        s.addTestSuite(cls2);
        if (class$org$xbill$DNS$APLRecordTest$Test_rrFromWire == null) {
            cls3 = class$("org.xbill.DNS.APLRecordTest$Test_rrFromWire");
            class$org$xbill$DNS$APLRecordTest$Test_rrFromWire = cls3;
        } else {
            cls3 = class$org$xbill$DNS$APLRecordTest$Test_rrFromWire;
        }
        s.addTestSuite(cls3);
        if (class$org$xbill$DNS$APLRecordTest$Test_rdataFromString == null) {
            cls4 = class$("org.xbill.DNS.APLRecordTest$Test_rdataFromString");
            class$org$xbill$DNS$APLRecordTest$Test_rdataFromString = cls4;
        } else {
            cls4 = class$org$xbill$DNS$APLRecordTest$Test_rdataFromString;
        }
        s.addTestSuite(cls4);
        if (class$org$xbill$DNS$APLRecordTest$Test_rrToString == null) {
            cls5 = class$("org.xbill.DNS.APLRecordTest$Test_rrToString");
            class$org$xbill$DNS$APLRecordTest$Test_rrToString = cls5;
        } else {
            cls5 = class$org$xbill$DNS$APLRecordTest$Test_rrToString;
        }
        s.addTestSuite(cls5);
        if (class$org$xbill$DNS$APLRecordTest$Test_rrToWire == null) {
            cls6 = class$("org.xbill.DNS.APLRecordTest$Test_rrToWire");
            class$org$xbill$DNS$APLRecordTest$Test_rrToWire = cls6;
        } else {
            cls6 = class$org$xbill$DNS$APLRecordTest$Test_rrToWire;
        }
        s.addTestSuite(cls6);
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
