package org.xbill.DNS;

import android.support.v4.os.EnvironmentCompat;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.Arrays;
import junit.framework.TestCase;

public class SetResponseTest extends TestCase {
    public void test_ctor_1arg() {
        int[] types = {0, 1, 2, 3, 4, 5, 6};
        for (int i = 0; i < types.length; i++) {
            SetResponse sr = new SetResponse(types[i]);
            assertNull(sr.getNS());
            boolean z = true;
            assertEquals(types[i] == 0, sr.isUnknown());
            assertEquals(types[i] == 1, sr.isNXDOMAIN());
            assertEquals(types[i] == 2, sr.isNXRRSET());
            assertEquals(types[i] == 3, sr.isDelegation());
            assertEquals(types[i] == 4, sr.isCNAME());
            assertEquals(types[i] == 5, sr.isDNAME());
            if (types[i] != 6) {
                z = false;
            }
            assertEquals(z, sr.isSuccessful());
        }
    }

    public void test_ctor_1arg_toosmall() {
        try {
            new SetResponse(-1);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e) {
        }
    }

    public void test_ctor_1arg_toobig() {
        try {
            new SetResponse(7);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e) {
        }
    }

    public void test_ctor_2arg() {
        int[] types = {0, 1, 2, 3, 4, 5, 6};
        for (int i = 0; i < types.length; i++) {
            RRset rs = new RRset();
            SetResponse sr = new SetResponse(types[i], rs);
            assertSame(rs, sr.getNS());
            boolean z = true;
            assertEquals(types[i] == 0, sr.isUnknown());
            assertEquals(types[i] == 1, sr.isNXDOMAIN());
            assertEquals(types[i] == 2, sr.isNXRRSET());
            assertEquals(types[i] == 3, sr.isDelegation());
            assertEquals(types[i] == 4, sr.isCNAME());
            assertEquals(types[i] == 5, sr.isDNAME());
            if (types[i] != 6) {
                z = false;
            }
            assertEquals(z, sr.isSuccessful());
        }
    }

    public void test_ctor_2arg_toosmall() {
        try {
            new SetResponse(-1, new RRset());
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e) {
        }
    }

    public void test_ctor_2arg_toobig() {
        try {
            new SetResponse(7, new RRset());
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e) {
        }
    }

    public void test_ofType_basic() {
        int[] types = {3, 4, 5, 6};
        for (int i = 0; i < types.length; i++) {
            SetResponse sr = SetResponse.ofType(types[i]);
            assertNull(sr.getNS());
            boolean z = true;
            assertEquals(types[i] == 0, sr.isUnknown());
            assertEquals(types[i] == 1, sr.isNXDOMAIN());
            assertEquals(types[i] == 2, sr.isNXRRSET());
            assertEquals(types[i] == 3, sr.isDelegation());
            assertEquals(types[i] == 4, sr.isCNAME());
            assertEquals(types[i] == 5, sr.isDNAME());
            if (types[i] != 6) {
                z = false;
            }
            assertEquals(z, sr.isSuccessful());
            assertNotSame(sr, SetResponse.ofType(types[i]));
        }
    }

    public void test_ofType_singleton() {
        int[] types = {0, 1, 2};
        for (int i = 0; i < types.length; i++) {
            SetResponse sr = SetResponse.ofType(types[i]);
            assertNull(sr.getNS());
            boolean z = true;
            assertEquals(types[i] == 0, sr.isUnknown());
            assertEquals(types[i] == 1, sr.isNXDOMAIN());
            assertEquals(types[i] == 2, sr.isNXRRSET());
            assertEquals(types[i] == 3, sr.isDelegation());
            assertEquals(types[i] == 4, sr.isCNAME());
            assertEquals(types[i] == 5, sr.isDNAME());
            if (types[i] != 6) {
                z = false;
            }
            assertEquals(z, sr.isSuccessful());
            assertSame(sr, SetResponse.ofType(types[i]));
        }
    }

    public void test_ofType_toosmall() {
        try {
            SetResponse.ofType(-1);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e) {
        }
    }

    public void test_ofType_toobig() {
        try {
            SetResponse.ofType(7);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e) {
        }
    }

    public void test_addRRset() throws TextParseException, UnknownHostException {
        RRset rrs = new RRset();
        rrs.addRR(new ARecord(Name.fromString("The.Name."), 1, 43981, InetAddress.getByName("192.168.0.1")));
        rrs.addRR(new ARecord(Name.fromString("The.Name."), 1, 43981, InetAddress.getByName("192.168.0.2")));
        SetResponse sr = new SetResponse(6);
        sr.addRRset(rrs);
        assertTrue(Arrays.equals(new RRset[]{rrs}, sr.answers()));
    }

    public void test_addRRset_multiple() throws TextParseException, UnknownHostException {
        RRset rrs = new RRset();
        rrs.addRR(new ARecord(Name.fromString("The.Name."), 1, 43981, InetAddress.getByName("192.168.0.1")));
        rrs.addRR(new ARecord(Name.fromString("The.Name."), 1, 43981, InetAddress.getByName("192.168.0.2")));
        RRset rrs2 = new RRset();
        rrs2.addRR(new ARecord(Name.fromString("The.Other.Name."), 1, 43982, InetAddress.getByName("192.168.1.1")));
        rrs2.addRR(new ARecord(Name.fromString("The.Other.Name."), 1, 43982, InetAddress.getByName("192.168.1.2")));
        SetResponse sr = new SetResponse(6);
        sr.addRRset(rrs);
        sr.addRRset(rrs2);
        assertTrue(Arrays.equals(new RRset[]{rrs, rrs2}, sr.answers()));
    }

    public void test_answers_nonSUCCESSFUL() {
        assertNull(new SetResponse(0, new RRset()).answers());
    }

    public void test_getCNAME() throws TextParseException, UnknownHostException {
        RRset rrs = new RRset();
        CNAMERecord cr = new CNAMERecord(Name.fromString("The.Name."), 1, 43981, Name.fromString("The.Alias."));
        rrs.addRR(cr);
        assertEquals(cr, new SetResponse(4, rrs).getCNAME());
    }

    public void test_getDNAME() throws TextParseException, UnknownHostException {
        RRset rrs = new RRset();
        DNAMERecord dr = new DNAMERecord(Name.fromString("The.Name."), 1, 43981, Name.fromString("The.Alias."));
        rrs.addRR(dr);
        assertEquals(dr, new SetResponse(5, rrs).getDNAME());
    }

    public void test_toString() throws TextParseException, UnknownHostException {
        int[] types = {0, 1, 2, 3, 4, 5, 6};
        RRset rrs = new RRset();
        rrs.addRR(new ARecord(Name.fromString("The.Name."), 1, 43981, InetAddress.getByName("192.168.0.1")));
        int i = 0;
        StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append("delegation: ");
        stringBuffer.append(rrs);
        StringBuffer stringBuffer2 = new StringBuffer();
        stringBuffer2.append("CNAME: ");
        stringBuffer2.append(rrs);
        StringBuffer stringBuffer3 = new StringBuffer();
        stringBuffer3.append("DNAME: ");
        stringBuffer3.append(rrs);
        String[] labels = {EnvironmentCompat.MEDIA_UNKNOWN, "NXDOMAIN", "NXRRSET", stringBuffer.toString(), stringBuffer2.toString(), stringBuffer3.toString(), "successful"};
        while (true) {
            int i2 = i;
            if (i2 < types.length) {
                assertEquals(labels[i2], new SetResponse(types[i2], rrs).toString());
                i = i2 + 1;
            } else {
                return;
            }
        }
    }
}
