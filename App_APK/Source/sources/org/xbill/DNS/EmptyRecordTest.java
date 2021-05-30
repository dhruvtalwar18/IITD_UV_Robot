package org.xbill.DNS;

import java.io.IOException;
import java.net.UnknownHostException;
import junit.framework.TestCase;

public class EmptyRecordTest extends TestCase {
    public void test_ctor() throws UnknownHostException {
        EmptyRecord ar = new EmptyRecord();
        assertNull(ar.getName());
        assertEquals(0, ar.getType());
        assertEquals(0, ar.getDClass());
        assertEquals(0, ar.getTTL());
    }

    public void test_getObject() {
        assertTrue(new EmptyRecord().getObject() instanceof EmptyRecord);
    }

    public void test_rrFromWire() throws IOException {
        DNSInput i = new DNSInput(new byte[]{1, 2, 3, 4, 5});
        i.jump(3);
        EmptyRecord er = new EmptyRecord();
        er.rrFromWire(i);
        assertEquals(3, i.current());
        assertNull(er.getName());
        assertEquals(0, er.getType());
        assertEquals(0, er.getDClass());
        assertEquals(0, er.getTTL());
    }

    public void test_rdataFromString() throws IOException {
        Tokenizer t = new Tokenizer("these are the tokens");
        EmptyRecord er = new EmptyRecord();
        er.rdataFromString(t, (Name) null);
        assertNull(er.getName());
        assertEquals(0, er.getType());
        assertEquals(0, er.getDClass());
        assertEquals(0, er.getTTL());
        assertEquals("these", t.getString());
    }

    public void test_rrToString() {
        assertEquals("", new EmptyRecord().rrToString());
    }

    public void test_rrToWire() {
        EmptyRecord er = new EmptyRecord();
        DNSOutput out = new DNSOutput();
        er.rrToWire(out, (Compression) null, true);
        assertEquals(0, out.toByteArray().length);
    }
}
