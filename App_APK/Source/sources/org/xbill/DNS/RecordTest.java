package org.xbill.DNS;

import com.google.common.base.Ascii;
import com.google.common.primitives.UnsignedBytes;
import java.io.IOException;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.Arrays;
import java.util.Date;
import junit.framework.TestCase;
import org.jboss.netty.handler.codec.http.HttpConstants;

public class RecordTest extends TestCase {

    private static class SubRecord extends Record {
        public SubRecord() {
        }

        public SubRecord(Name name, int type, int dclass, long ttl) {
            super(name, type, dclass, ttl);
        }

        public Record getObject() {
            return null;
        }

        public void rrFromWire(DNSInput in) throws IOException {
        }

        public String rrToString() {
            return "{SubRecord: rrToString}";
        }

        public void rdataFromString(Tokenizer t, Name origin) throws IOException {
        }

        public void rrToWire(DNSOutput out, Compression c, boolean canonical) {
        }

        public static byte[] byteArrayFromString(String in) throws TextParseException {
            return Record.byteArrayFromString(in);
        }

        public static String byteArrayToString(byte[] in, boolean quote) {
            return Record.byteArrayToString(in, quote);
        }

        public static String unknownToString(byte[] in) {
            return Record.unknownToString(in);
        }

        public Object clone() throws CloneNotSupportedException {
            throw new CloneNotSupportedException();
        }
    }

    public void test_ctor_0arg() {
        SubRecord sr = new SubRecord();
        assertNull(sr.getName());
        assertEquals(0, sr.getType());
        assertEquals(0, sr.getTTL());
        assertEquals(0, sr.getDClass());
    }

    public void test_ctor_4arg() throws TextParseException {
        Name n = Name.fromString("my.name.");
        SubRecord r = new SubRecord(n, 1, 1, 703710);
        assertEquals(n, r.getName());
        assertEquals(1, r.getType());
        assertEquals(1, r.getDClass());
        assertEquals(703710, r.getTTL());
    }

    public void test_ctor_4arg_invalid() throws TextParseException {
        Name n = Name.fromString("my.name.");
        try {
            new SubRecord(Name.fromString("my.relative.name"), 1, 1, 703710);
            fail("RelativeNameException not thrown");
        } catch (RelativeNameException e) {
        }
        try {
            new SubRecord(n, -1, 1, 703710);
            fail("InvalidTypeException not thrown");
        } catch (InvalidTypeException e2) {
        }
        try {
            new SubRecord(n, 1, -1, 703710);
            fail("InvalidDClassException not thrown");
        } catch (InvalidDClassException e3) {
        }
        try {
            new SubRecord(n, 1, 1, -1);
            fail("InvalidTTLException not thrown");
        } catch (InvalidTTLException e4) {
        }
    }

    public void test_newRecord_3arg() throws TextParseException {
        Name n = Name.fromString("my.name.");
        Name r = Name.fromString("my.relative.name");
        Record rec = Record.newRecord(n, 1, 1);
        assertTrue(rec instanceof EmptyRecord);
        assertEquals(n, rec.getName());
        assertEquals(1, rec.getType());
        assertEquals(1, rec.getDClass());
        assertEquals(0, rec.getTTL());
        try {
            Record.newRecord(r, 1, 1);
            fail("RelativeNameException not thrown");
        } catch (RelativeNameException e) {
        }
    }

    public void test_newRecord_4arg() throws TextParseException {
        Name n = Name.fromString("my.name.");
        Name r = Name.fromString("my.relative.name");
        Record rec = Record.newRecord(n, 1, 1, (long) 56296);
        assertTrue(rec instanceof EmptyRecord);
        assertEquals(n, rec.getName());
        assertEquals(1, rec.getType());
        assertEquals(1, rec.getDClass());
        assertEquals((long) 56296, rec.getTTL());
        try {
            Record.newRecord(r, 1, 1, (long) 56296);
            fail("RelativeNameException not thrown");
        } catch (RelativeNameException e) {
        }
    }

    public void test_newRecord_5arg() throws TextParseException, UnknownHostException {
        Name n = Name.fromString("my.name.");
        InetAddress exp = InetAddress.getByName("123.232.0.255");
        Name name = n;
        Record rec = Record.newRecord(name, 1, 1, (long) 56296, new byte[]{123, -24, 0, -1});
        assertTrue(rec instanceof ARecord);
        assertEquals(n, rec.getName());
        assertEquals(1, rec.getType());
        assertEquals(1, rec.getDClass());
        assertEquals((long) 56296, rec.getTTL());
        assertEquals(exp, ((ARecord) rec).getAddress());
    }

    public void test_newRecord_6arg() throws TextParseException, UnknownHostException {
        Name n = Name.fromString("my.name.");
        byte[] data = {123, -24, 0, -1};
        InetAddress exp = InetAddress.getByName("123.232.0.255");
        Record rec = Record.newRecord(n, 1, 1, (long) 56296, 0, (byte[]) null);
        assertTrue(rec instanceof EmptyRecord);
        assertEquals(n, rec.getName());
        assertEquals(1, rec.getType());
        assertEquals(1, rec.getDClass());
        assertEquals((long) 56296, rec.getTTL());
        byte[] bArr = data;
        Record rec2 = Record.newRecord(n, 1, 1, (long) 56296, data.length, bArr);
        assertTrue(rec2 instanceof ARecord);
        assertEquals(n, rec2.getName());
        assertEquals(1, rec2.getType());
        assertEquals(1, rec2.getDClass());
        assertEquals((long) 56296, rec2.getTTL());
        assertEquals(exp, ((ARecord) rec2).getAddress());
        Record rec3 = Record.newRecord(n, 32, 1, (long) 56296, data.length, bArr);
        assertTrue(rec3 instanceof UNKRecord);
        assertEquals(n, rec3.getName());
        assertEquals(32, rec3.getType());
        assertEquals(1, rec3.getDClass());
        assertEquals((long) 56296, rec3.getTTL());
        assertTrue(Arrays.equals(data, ((UNKRecord) rec3).getData()));
    }

    public void test_newRecord_6arg_invalid() throws TextParseException {
        Name n = Name.fromString("my.name.");
        Name r = Name.fromString("my.relative.name");
        byte[] data = {123, -24, 0, -1};
        assertNull(Record.newRecord(n, 1, 1, (long) 56296, 0, new byte[0]));
        assertNull(Record.newRecord(n, 1, 1, (long) 56296, 1, new byte[0]));
        Name name = n;
        assertNull(Record.newRecord(name, 1, 1, (long) 56296, data.length + 1, data));
        Name name2 = n;
        assertNull(Record.newRecord(name2, 1, 1, (long) 56296, 5, new byte[]{data[0], data[1], data[2], data[3], 0}));
        try {
            Record.newRecord(r, 1, 1, (long) 56296, 0, (byte[]) null);
            fail("RelativeNameException not thrown");
        } catch (RelativeNameException e) {
        }
    }

    public void test_fromWire() throws IOException, TextParseException, UnknownHostException {
        Name n = Name.fromString("my.name.");
        byte[] data = {123, -24, 0, -1};
        InetAddress exp = InetAddress.getByName("123.232.0.255");
        DNSOutput out = new DNSOutput();
        n.toWire(out, (Compression) null);
        out.writeU16(1);
        out.writeU16(1);
        out.writeU32((long) 56296);
        out.writeU16(data.length);
        out.writeByteArray(data);
        Record rec = Record.fromWire(new DNSInput(out.toByteArray()), 1, false);
        assertTrue(rec instanceof ARecord);
        assertEquals(n, rec.getName());
        assertEquals(1, rec.getType());
        assertEquals(1, rec.getDClass());
        assertEquals((long) 56296, rec.getTTL());
        assertEquals(exp, ((ARecord) rec).getAddress());
        Record rec2 = Record.fromWire(new DNSInput(out.toByteArray()), 0, false);
        assertTrue(rec2 instanceof EmptyRecord);
        assertEquals(n, rec2.getName());
        assertEquals(1, rec2.getType());
        assertEquals(1, rec2.getDClass());
        assertEquals(0, rec2.getTTL());
        Record rec3 = Record.fromWire(new DNSInput(out.toByteArray()), 0);
        assertTrue(rec3 instanceof EmptyRecord);
        assertEquals(n, rec3.getName());
        assertEquals(1, rec3.getType());
        assertEquals(1, rec3.getDClass());
        assertEquals(0, rec3.getTTL());
        Record rec4 = Record.fromWire(out.toByteArray(), 0);
        assertTrue(rec4 instanceof EmptyRecord);
        assertEquals(n, rec4.getName());
        assertEquals(1, rec4.getType());
        assertEquals(1, rec4.getDClass());
        assertEquals(0, rec4.getTTL());
        DNSOutput out2 = new DNSOutput();
        n.toWire(out2, (Compression) null);
        out2.writeU16(1);
        out2.writeU16(1);
        out2.writeU32((long) 56296);
        out2.writeU16(0);
        Record rec5 = Record.fromWire(new DNSInput(out2.toByteArray()), 1, true);
        assertTrue(rec5 instanceof EmptyRecord);
        assertEquals(n, rec5.getName());
        assertEquals(1, rec5.getType());
        assertEquals(1, rec5.getDClass());
        assertEquals((long) 56296, rec5.getTTL());
    }

    public void test_toWire() throws IOException, TextParseException, UnknownHostException {
        Name n = Name.fromString("my.name.");
        byte[] data = {123, -24, 0, -1};
        DNSOutput out = new DNSOutput();
        n.toWire(out, (Compression) null);
        out.writeU16(1);
        out.writeU16(1);
        out.writeU32((long) 56296);
        out.writeU16(data.length);
        out.writeByteArray(data);
        byte[] exp = out.toByteArray();
        Record rec = Record.newRecord(n, 1, 1, (long) 56296, data.length, data);
        DNSOutput out2 = new DNSOutput();
        rec.toWire(out2, 1, (Compression) null);
        assertTrue(Arrays.equals(exp, out2.toByteArray()));
        assertTrue(Arrays.equals(exp, rec.toWire(1)));
        DNSOutput out3 = new DNSOutput();
        n.toWire(out3, (Compression) null);
        out3.writeU16(1);
        out3.writeU16(1);
        byte[] exp2 = out3.toByteArray();
        DNSOutput out4 = new DNSOutput();
        rec.toWire(out4, 0, (Compression) null);
        assertTrue(Arrays.equals(exp2, out4.toByteArray()));
    }

    public void test_toWireCanonical() throws IOException, TextParseException, UnknownHostException {
        Name n = Name.fromString("My.Name.");
        byte[] data = {123, -24, 0, -1};
        DNSOutput out = new DNSOutput();
        n.toWireCanonical(out);
        out.writeU16(1);
        out.writeU16(1);
        out.writeU32((long) 56296);
        out.writeU16(data.length);
        out.writeByteArray(data);
        assertTrue(Arrays.equals(out.toByteArray(), Record.newRecord(n, 1, 1, (long) 56296, data.length, data).toWireCanonical()));
    }

    public void test_rdataToWireCanonical() throws IOException, TextParseException, UnknownHostException {
        Name n = Name.fromString("My.Name.");
        Name n2 = Name.fromString("My.Second.Name.");
        DNSOutput out = new DNSOutput();
        n2.toWire(out, (Compression) null);
        byte[] data = out.toByteArray();
        DNSOutput out2 = new DNSOutput();
        n2.toWireCanonical(out2);
        byte[] exp = out2.toByteArray();
        Record rec = Record.newRecord(n, 2, 1, (long) 704153, data.length, data);
        assertTrue(rec instanceof NSRecord);
        assertTrue(Arrays.equals(exp, rec.rdataToWireCanonical()));
    }

    public void test_rdataToString() throws IOException, TextParseException, UnknownHostException {
        Name n = Name.fromString("My.Name.");
        Name n2 = Name.fromString("My.Second.Name.");
        DNSOutput out = new DNSOutput();
        n2.toWire(out, (Compression) null);
        byte[] data = out.toByteArray();
        Record rec = Record.newRecord(n, 2, 1, (long) 704153, data.length, data);
        assertTrue(rec instanceof NSRecord);
        assertEquals(rec.rrToString(), rec.rdataToString());
    }

    public void test_toString() throws TextParseException {
        Name n = Name.fromString("My.N.");
        Name n2 = Name.fromString("My.Second.Name.");
        DNSOutput o = new DNSOutput();
        n2.toWire(o, (Compression) null);
        byte[] data = o.toByteArray();
        Record rec = Record.newRecord(n, 2, 1, (long) 704153, data.length, data);
        String out = rec.toString();
        boolean z = false;
        assertFalse(out.indexOf(n.toString()) == -1);
        assertFalse(out.indexOf(n2.toString()) == -1);
        assertFalse(out.indexOf("NS") == -1);
        assertFalse(out.indexOf("IN") == -1);
        StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append(704153);
        stringBuffer.append("");
        assertFalse(out.indexOf(stringBuffer.toString()) == -1);
        Options.set("BINDTTL");
        String out2 = rec.toString();
        assertFalse(out2.indexOf(n.toString()) == -1);
        assertFalse(out2.indexOf(n2.toString()) == -1);
        assertFalse(out2.indexOf("NS") == -1);
        assertFalse(out2.indexOf("IN") == -1);
        assertFalse(out2.indexOf(TTL.format((long) 704153)) == -1);
        Options.set("noPrintIN");
        String out3 = rec.toString();
        assertFalse(out3.indexOf(n.toString()) == -1);
        assertFalse(out3.indexOf(n2.toString()) == -1);
        assertFalse(out3.indexOf("NS") == -1);
        assertTrue(out3.indexOf("IN") == -1);
        if (out3.indexOf(TTL.format((long) 704153)) == -1) {
            z = true;
        }
        assertFalse(z);
    }

    public void test_byteArrayFromString() throws TextParseException {
        assertTrue(Arrays.equals("the 98 \" ' quick 0xAB brown".getBytes(), SubRecord.byteArrayFromString("the 98 \" ' quick 0xAB brown")));
        assertTrue(Arrays.equals(new byte[]{32, 31, 65, 97, HttpConstants.SEMICOLON, 34, 92, 126, Ascii.DEL, -1}, SubRecord.byteArrayFromString(" \\031Aa\\;\\\"\\\\~\\127\\255")));
    }

    public void test_byteArrayFromString_invalid() {
        StringBuffer b = new StringBuffer();
        for (int i = 0; i < 257; i++) {
            b.append('A');
        }
        try {
            SubRecord.byteArrayFromString(b.toString());
            fail("TextParseException not thrown");
        } catch (TextParseException e) {
        }
        try {
            SubRecord.byteArrayFromString("\\256");
            fail("TextParseException not thrown");
        } catch (TextParseException e2) {
        }
        try {
            SubRecord.byteArrayFromString("\\25a");
            fail("TextParseException not thrown");
        } catch (TextParseException e3) {
        }
        try {
            SubRecord.byteArrayFromString("\\25");
            fail("TextParseException not thrown");
        } catch (TextParseException e4) {
        }
        b.append("\\233");
        try {
            SubRecord.byteArrayFromString(b.toString());
            fail("TextParseException not thrown");
        } catch (TextParseException e5) {
        }
    }

    public void test_byteArrayToString() {
        assertEquals("\" \\031Aa;\\\"\\\\~\\127\\255\"", SubRecord.byteArrayToString(new byte[]{32, 31, 65, 97, HttpConstants.SEMICOLON, 34, 92, 126, Ascii.DEL, -1}, true));
    }

    public void test_unknownToString() {
        byte[] data = {Ascii.DC2, 52, 86, 120, -102, -68, -34, -1};
        String out = SubRecord.unknownToString(data);
        StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append("");
        stringBuffer.append(data.length);
        boolean z = false;
        assertFalse(out.indexOf(stringBuffer.toString()) == -1);
        if (out.indexOf("123456789ABCDEFF") == -1) {
            z = true;
        }
        assertFalse(z);
    }

    public void test_fromString() throws IOException, TextParseException {
        Name n = Name.fromString("My.N.");
        Name n2 = Name.fromString("My.Second.Name.");
        InetAddress addr = InetAddress.getByName("191.234.43.10");
        Name name = n;
        Record rec = Record.fromString(name, 1, 1, (long) 704153, new Tokenizer("191.234.43.10"), n2);
        assertTrue(rec instanceof ARecord);
        assertEquals(n, rec.getName());
        assertEquals(1, rec.getType());
        assertEquals(1, rec.getDClass());
        assertEquals((long) 704153, rec.getTTL());
        assertEquals(addr, ((ARecord) rec).getAddress());
        String unkData = SubRecord.unknownToString(new byte[]{-65, -22, 43, 10});
        Name name2 = n;
        String str = unkData;
        Record rec2 = Record.fromString(name2, 1, 1, (long) 704153, new Tokenizer(unkData), n2);
        assertTrue(rec2 instanceof ARecord);
        assertEquals(n, rec2.getName());
        assertEquals(1, rec2.getType());
        assertEquals(1, rec2.getDClass());
        assertEquals((long) 704153, rec2.getTTL());
        assertEquals(addr, ((ARecord) rec2).getAddress());
    }

    public void test_fromString_invalid() throws IOException, TextParseException {
        Name n = Name.fromString("My.N.");
        Name rel = Name.fromString("My.R");
        Name n2 = Name.fromString("My.Second.Name.");
        InetAddress byName = InetAddress.getByName("191.234.43.10");
        try {
            Record.fromString(rel, 1, 1, (long) 704153, new Tokenizer("191.234.43.10"), n2);
            fail("RelativeNameException not thrown");
        } catch (RelativeNameException e) {
        }
        try {
            Record.fromString(n, 1, 1, (long) 704153, new Tokenizer("191.234.43.10 another_token"), n2);
            fail("TextParseException not thrown");
        } catch (TextParseException e2) {
        }
        try {
            Record.fromString(n, 1, 1, (long) 704153, new Tokenizer("\\# 100 ABCDE"), n2);
            fail("TextParseException not thrown");
        } catch (TextParseException e3) {
        }
        try {
            Record.fromString(n, 1, 1, (long) 704153, "\\# 100", n2);
            fail("TextParseException not thrown");
        } catch (TextParseException e4) {
        }
    }

    public void test_getRRsetType() throws TextParseException {
        Name n = Name.fromString("My.N.");
        assertEquals(1, Record.newRecord(n, 1, 1, 0).getRRsetType());
        assertEquals(1, new RRSIGRecord(n, 1, 0, 1, 1, 0, new Date(), new Date(), 10, n, new byte[0]).getRRsetType());
    }

    public void test_sameRRset() throws TextParseException {
        Name n = Name.fromString("My.N.");
        Name m = Name.fromString("My.M.");
        RRSIGRecord rRSIGRecord = new RRSIGRecord(n, 1, 0, 1, 1, 0, new Date(), new Date(), 10, n, new byte[0]);
        Record r1 = Record.newRecord(n, 1, 1, 0);
        assertTrue(r1.sameRRset(rRSIGRecord));
        assertTrue(rRSIGRecord.sameRRset(r1));
        Record r12 = Record.newRecord(n, 1, 4, 0);
        RRSIGRecord rRSIGRecord2 = rRSIGRecord;
        RRSIGRecord rRSIGRecord3 = new RRSIGRecord(n, 1, 0, 1, 1, 0, new Date(), new Date(), 10, n, new byte[0]);
        Record r13 = r12;
        assertFalse(r13.sameRRset(rRSIGRecord3));
        assertFalse(rRSIGRecord3.sameRRset(r13));
        RRSIGRecord rRSIGRecord4 = rRSIGRecord3;
        Record r2 = new RRSIGRecord(m, 1, 0, 1, 1, 0, new Date(), new Date(), 10, n, new byte[0]);
        Record r14 = Record.newRecord(n, 1, 1, 0);
        assertFalse(r14.sameRRset(r2));
        assertFalse(r2.sameRRset(r14));
    }

    public void test_equals() throws TextParseException {
        Name n = Name.fromString("My.N.");
        Name n2 = Name.fromString("my.n.");
        Name m = Name.fromString("My.M.");
        Record r1 = Record.newRecord(n, 1, 1, 0);
        assertFalse(r1.equals((Object) null));
        assertFalse(r1.equals(new Object()));
        Record r2 = Record.newRecord(n, 1, 1, 0);
        assertEquals(r1, r2);
        assertEquals(r2, r1);
        Record r22 = Record.newRecord(n2, 1, 1, 0);
        assertEquals(r1, r22);
        assertEquals(r22, r1);
        Record r23 = Record.newRecord(n2, 1, 1, 703710);
        assertEquals(r1, r23);
        assertEquals(r23, r1);
        Record r24 = Record.newRecord(m, 1, 1, 703710);
        assertFalse(r1.equals(r24));
        assertFalse(r24.equals(r1));
        Record r25 = Record.newRecord(n2, 15, 1, 703710);
        assertFalse(r1.equals(r25));
        assertFalse(r25.equals(r1));
        Record r26 = Record.newRecord(n2, 1, 3, 703710);
        assertFalse(r1.equals(r26));
        assertFalse(r26.equals(r1));
        byte[] d1 = {Ascii.ETB, Ascii.FF, 9, -127};
        Record r12 = Record.newRecord(n, 1, 1, 11259369, d1);
        Record r27 = Record.newRecord(n, 1, 1, 11259369, d1);
        assertEquals(r12, r27);
        assertEquals(r27, r12);
        Name name = m;
        Name name2 = n2;
        Record r28 = Record.newRecord(n, 1, 1, 11259369, new byte[]{-36, 1, -125, -44});
        assertFalse(r12.equals(r28));
        assertFalse(r28.equals(r12));
    }

    public void test_hashCode() throws TextParseException {
        Name n = Name.fromString("My.N.");
        Name n2 = Name.fromString("my.n.");
        Name m = Name.fromString("My.M.");
        byte[] d1 = {Ascii.ETB, Ascii.FF, 9, -127};
        byte[] d2 = {-36, 1, -125, -44};
        Record r1 = Record.newRecord(n, 1, 1, 11259369, d1);
        byte[] bArr = d1;
        assertEquals(r1.hashCode(), Record.newRecord(n, 1, 1, 11259369, bArr).hashCode());
        assertEquals(r1.hashCode(), Record.newRecord(n2, 1, 1, 11259369, bArr).hashCode());
        boolean z = true;
        assertFalse(r1.hashCode() == Record.newRecord(m, 1, 1, 11259369, bArr).hashCode());
        assertFalse(r1.hashCode() == Record.newRecord(n, 1, 3, 11259369, d1).hashCode());
        assertEquals(r1.hashCode(), Record.newRecord(n, 1, 1, 703710, d1).hashCode());
        if (r1.hashCode() != Record.newRecord(n, 1, 1, 11259369, d2).hashCode()) {
            z = false;
        }
        assertFalse(z);
    }

    public void test_cloneRecord() throws TextParseException {
        Name n = Name.fromString("My.N.");
        Record r = Record.newRecord(n, 1, 1, 11259369, new byte[]{Ascii.ETB, Ascii.FF, 9, -127});
        Record r2 = r.cloneRecord();
        assertNotSame(r, r2);
        assertEquals(r, r2);
        try {
            new SubRecord(n, 1, 1, 11259369).cloneRecord();
            fail("IllegalStateException not thrown");
        } catch (IllegalStateException e) {
        }
    }

    public void test_withName() throws TextParseException {
        Name n = Name.fromString("My.N.");
        Name m = Name.fromString("My.M.Name.");
        Name rel = Name.fromString("My.Relative.Name");
        Record r = Record.newRecord(n, 1, 1, 11259369, new byte[]{Ascii.ETB, Ascii.FF, 9, -127});
        Record r1 = r.withName(m);
        assertEquals(m, r1.getName());
        assertEquals(1, r1.getType());
        assertEquals(1, r1.getDClass());
        assertEquals(11259369, r1.getTTL());
        assertEquals(((ARecord) r).getAddress(), ((ARecord) r1).getAddress());
        try {
            r.withName(rel);
            fail("RelativeNameException not thrown");
        } catch (RelativeNameException e) {
        }
    }

    public void test_withDClass() throws TextParseException {
        Name n = Name.fromString("My.N.");
        Record r = Record.newRecord(n, 1, 1, 11259369, new byte[]{Ascii.ETB, Ascii.FF, 9, -127});
        Record r1 = r.withDClass(4, 39030);
        assertEquals(n, r1.getName());
        assertEquals(1, r1.getType());
        assertEquals(4, r1.getDClass());
        assertEquals(39030, r1.getTTL());
        assertEquals(((ARecord) r).getAddress(), ((ARecord) r1).getAddress());
    }

    public void test_setTTL() throws TextParseException, UnknownHostException {
        Name n = Name.fromString("My.N.");
        byte[] d = {Ascii.ETB, Ascii.FF, 9, -127};
        InetAddress exp = InetAddress.getByName("23.12.9.129");
        Record r = Record.newRecord(n, 1, 1, 11259369, d);
        assertEquals(11259369, r.getTTL());
        r.setTTL(39030);
        assertEquals(n, r.getName());
        assertEquals(1, r.getType());
        assertEquals(1, r.getDClass());
        assertEquals(39030, r.getTTL());
        assertEquals(exp, ((ARecord) r).getAddress());
    }

    public void test_compareTo() throws TextParseException {
        Name n = Name.fromString("My.N.");
        Name n2 = Name.fromString("my.n.");
        Name m = Name.fromString("My.M.");
        byte[] d = {Ascii.ETB, Ascii.FF, 9, -127};
        byte[] d2 = {Ascii.ETB, Ascii.FF, 9, UnsignedBytes.MAX_POWER_OF_TWO};
        Record r1 = Record.newRecord(n, 1, 1, 11259369, d);
        byte[] bArr = d;
        Record r2 = Record.newRecord(n, 1, 1, 11259369, bArr);
        assertEquals(0, r1.compareTo(r1));
        assertEquals(0, r1.compareTo(r2));
        assertEquals(0, r2.compareTo(r1));
        Record r22 = Record.newRecord(n2, 1, 1, 11259369, bArr);
        assertEquals(0, r1.compareTo(r22));
        assertEquals(0, r22.compareTo(r1));
        Record r23 = Record.newRecord(m, 1, 1, 11259369, bArr);
        assertEquals(n.compareTo(m), r1.compareTo(r23));
        assertEquals(m.compareTo(n), r23.compareTo(r1));
        Name name = n;
        Record r24 = Record.newRecord(name, 1, 3, 11259369, bArr);
        assertEquals(-2, r1.compareTo(r24));
        assertEquals(2, r24.compareTo(r1));
        Record r25 = Record.newRecord(name, 2, 1, 11259369, m.toWire());
        assertEquals(-1, r1.compareTo(r25));
        assertEquals(1, r25.compareTo(r1));
        Name name2 = m;
        Record r26 = Record.newRecord(n, 1, 1, 11259369, d2);
        assertEquals(1, r1.compareTo(r26));
        assertEquals(-1, r26.compareTo(r1));
        Name m2 = Name.fromString("My.N.L.");
        Name name3 = n;
        Record r12 = Record.newRecord(name3, 2, 1, 11259369, n.toWire());
        Record r27 = Record.newRecord(name3, 2, 1, 11259369, m2.toWire());
        assertEquals(-1, r12.compareTo(r27));
        assertEquals(1, r27.compareTo(r12));
    }

    public void test_getAdditionalName() throws TextParseException {
        assertNull(new SubRecord(Name.fromString("My.N."), 1, 1, 11259369).getAdditionalName());
    }

    public void test_checkU8() {
        try {
            Record.checkU8("field", -1);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e) {
        }
        assertEquals(0, Record.checkU8("field", 0));
        assertEquals(157, Record.checkU8("field", 157));
        assertEquals(255, Record.checkU8("field", 255));
        try {
            Record.checkU8("field", 256);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e2) {
        }
    }

    public void test_checkU16() {
        try {
            Record.checkU16("field", -1);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e) {
        }
        assertEquals(0, Record.checkU16("field", 0));
        assertEquals(40353, Record.checkU16("field", 40353));
        assertEquals(65535, Record.checkU16("field", 65535));
        try {
            Record.checkU16("field", 65536);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e2) {
        }
    }

    public void test_checkU32() {
        try {
            Record.checkU32("field", -1);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e) {
        }
        assertEquals(0, Record.checkU32("field", 0));
        assertEquals(2644635693L, Record.checkU32("field", 2644635693L));
        assertEquals(4294967295L, Record.checkU32("field", 4294967295L));
        try {
            Record.checkU32("field", 4294967296L);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e2) {
        }
    }

    public void test_checkName() throws TextParseException {
        Name n = Name.fromString("My.N.");
        Name m = Name.fromString("My.m");
        assertEquals(n, Record.checkName("field", n));
        try {
            Record.checkName("field", m);
            fail("RelativeNameException not thrown");
        } catch (RelativeNameException e) {
        }
    }
}
