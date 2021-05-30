package org.xbill.DNS;

import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.Date;
import java.util.Iterator;
import junit.framework.TestCase;

public class RRsetTest extends TestCase {
    ARecord m_a1;
    ARecord m_a2;
    Name m_name;
    Name m_name2;
    private RRset m_rs;
    RRSIGRecord m_s1;
    RRSIGRecord m_s2;
    long m_ttl;

    public void setUp() throws TextParseException, UnknownHostException {
        this.m_rs = new RRset();
        this.m_name = Name.fromString("this.is.a.test.");
        this.m_name2 = Name.fromString("this.is.another.test.");
        this.m_ttl = 43981;
        this.m_a1 = new ARecord(this.m_name, 1, this.m_ttl, InetAddress.getByName("192.169.232.11"));
        this.m_a2 = new ARecord(this.m_name, 1, this.m_ttl + 1, InetAddress.getByName("192.169.232.12"));
        Name name = this.m_name;
        this.m_s1 = new RRSIGRecord(name, 1, this.m_ttl, 1, 15, 703710, new Date(), new Date(), 10, this.m_name, new byte[0]);
        Name name2 = this.m_name;
        this.m_s2 = new RRSIGRecord(name2, 1, this.m_ttl, 1, 15, 703710, new Date(), new Date(), 10, this.m_name2, new byte[0]);
    }

    public void test_ctor_0arg() {
        assertEquals(0, this.m_rs.size());
        try {
            this.m_rs.getDClass();
            fail("IllegalStateException not thrown");
        } catch (IllegalStateException e) {
        }
        try {
            this.m_rs.getType();
            fail("IllegalStateException not thrown");
        } catch (IllegalStateException e2) {
        }
        try {
            this.m_rs.getTTL();
            fail("IllegalStateException not thrown");
        } catch (IllegalStateException e3) {
        }
        try {
            this.m_rs.getName();
            fail("IllegalStateException not thrown");
        } catch (IllegalStateException e4) {
        }
        try {
            this.m_rs.first();
            fail("IllegalStateException not thrown");
        } catch (IllegalStateException e5) {
        }
        try {
            this.m_rs.toString();
            fail("IllegalStateException not thrown");
        } catch (IllegalStateException e6) {
        }
        Iterator itr = this.m_rs.rrs();
        assertNotNull(itr);
        assertFalse(itr.hasNext());
        Iterator itr2 = this.m_rs.sigs();
        assertNotNull(itr2);
        assertFalse(itr2.hasNext());
    }

    public void test_basics() throws TextParseException, UnknownHostException {
        this.m_rs.addRR(this.m_a1);
        assertEquals(1, this.m_rs.size());
        assertEquals(1, this.m_rs.getDClass());
        assertEquals(this.m_a1, this.m_rs.first());
        assertEquals(this.m_name, this.m_rs.getName());
        assertEquals(this.m_ttl, this.m_rs.getTTL());
        assertEquals(1, this.m_rs.getType());
        this.m_rs.addRR(this.m_a1);
        assertEquals(1, this.m_rs.size());
        assertEquals(1, this.m_rs.getDClass());
        assertEquals(this.m_a1, this.m_rs.first());
        assertEquals(this.m_name, this.m_rs.getName());
        assertEquals(this.m_ttl, this.m_rs.getTTL());
        assertEquals(1, this.m_rs.getType());
        this.m_rs.addRR(this.m_a2);
        assertEquals(2, this.m_rs.size());
        assertEquals(1, this.m_rs.getDClass());
        assertEquals(this.m_a1, this.m_rs.first());
        assertEquals(this.m_name, this.m_rs.getName());
        assertEquals(this.m_ttl, this.m_rs.getTTL());
        assertEquals(1, this.m_rs.getType());
        Iterator itr = this.m_rs.rrs();
        assertEquals(this.m_a1, itr.next());
        assertEquals(this.m_a2, itr.next());
        Iterator itr2 = this.m_rs.rrs();
        assertEquals(this.m_a2, itr2.next());
        assertEquals(this.m_a1, itr2.next());
        Iterator itr3 = this.m_rs.rrs();
        assertEquals(this.m_a1, itr3.next());
        assertEquals(this.m_a2, itr3.next());
        this.m_rs.deleteRR(this.m_a1);
        assertEquals(1, this.m_rs.size());
        assertEquals(1, this.m_rs.getDClass());
        assertEquals(this.m_a2, this.m_rs.first());
        assertEquals(this.m_name, this.m_rs.getName());
        assertEquals(this.m_ttl, this.m_rs.getTTL());
        assertEquals(1, this.m_rs.getType());
        this.m_rs.addRR(this.m_s1);
        assertEquals(1, this.m_rs.size());
        Iterator itr4 = this.m_rs.sigs();
        assertEquals(this.m_s1, itr4.next());
        assertFalse(itr4.hasNext());
        this.m_rs.addRR(this.m_s1);
        Iterator itr5 = this.m_rs.sigs();
        assertEquals(this.m_s1, itr5.next());
        assertFalse(itr5.hasNext());
        this.m_rs.addRR(this.m_s2);
        Iterator itr6 = this.m_rs.sigs();
        assertEquals(this.m_s1, itr6.next());
        assertEquals(this.m_s2, itr6.next());
        assertFalse(itr6.hasNext());
        this.m_rs.deleteRR(this.m_s1);
        Iterator itr7 = this.m_rs.sigs();
        assertEquals(this.m_s2, itr7.next());
        assertFalse(itr7.hasNext());
        this.m_rs.clear();
        assertEquals(0, this.m_rs.size());
        assertFalse(this.m_rs.rrs().hasNext());
        assertFalse(this.m_rs.sigs().hasNext());
    }

    public void test_ctor_1arg() {
        this.m_rs.addRR(this.m_a1);
        this.m_rs.addRR(this.m_a2);
        this.m_rs.addRR(this.m_s1);
        this.m_rs.addRR(this.m_s2);
        RRset rs2 = new RRset(this.m_rs);
        assertEquals(2, rs2.size());
        assertEquals(this.m_a1, rs2.first());
        Iterator itr = rs2.rrs();
        assertEquals(this.m_a1, itr.next());
        assertEquals(this.m_a2, itr.next());
        assertFalse(itr.hasNext());
        Iterator itr2 = rs2.sigs();
        assertTrue(itr2.hasNext());
        assertEquals(this.m_s1, itr2.next());
        assertTrue(itr2.hasNext());
        assertEquals(this.m_s2, itr2.next());
        assertFalse(itr2.hasNext());
    }

    public void test_toString() {
        this.m_rs.addRR(this.m_a1);
        this.m_rs.addRR(this.m_a2);
        this.m_rs.addRR(this.m_s1);
        this.m_rs.addRR(this.m_s2);
        String out = this.m_rs.toString();
        boolean z = false;
        assertTrue(out.indexOf(this.m_name.toString()) != -1);
        assertTrue(out.indexOf(" IN A ") != -1);
        assertTrue(out.indexOf("[192.169.232.11]") != -1);
        if (out.indexOf("[192.169.232.12]") != -1) {
            z = true;
        }
        assertTrue(z);
    }

    public void test_addRR_invalidType() throws TextParseException {
        this.m_rs.addRR(this.m_a1);
        try {
            this.m_rs.addRR(new CNAMERecord(this.m_name, 1, this.m_ttl, Name.fromString("an.alias.")));
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e) {
        }
    }

    public void test_addRR_invalidName() throws TextParseException, UnknownHostException {
        this.m_rs.addRR(this.m_a1);
        this.m_a2 = new ARecord(this.m_name2, 1, this.m_ttl, InetAddress.getByName("192.169.232.11"));
        try {
            this.m_rs.addRR(this.m_a2);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e) {
        }
    }

    public void test_addRR_invalidDClass() throws TextParseException, UnknownHostException {
        this.m_rs.addRR(this.m_a1);
        this.m_a2 = new ARecord(this.m_name, 3, this.m_ttl, InetAddress.getByName("192.169.232.11"));
        try {
            this.m_rs.addRR(this.m_a2);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e) {
        }
    }

    public void test_TTLcalculation() {
        this.m_rs.addRR(this.m_a2);
        assertEquals(this.m_a2.getTTL(), this.m_rs.getTTL());
        this.m_rs.addRR(this.m_a1);
        assertEquals(this.m_a1.getTTL(), this.m_rs.getTTL());
        Iterator itr = this.m_rs.rrs();
        while (itr.hasNext()) {
            assertEquals(this.m_a1.getTTL(), ((Record) itr.next()).getTTL());
        }
    }

    public void test_Record_placement() {
        this.m_rs.addRR(this.m_a1);
        this.m_rs.addRR(this.m_s1);
        this.m_rs.addRR(this.m_a2);
        Iterator itr = this.m_rs.rrs();
        assertTrue(itr.hasNext());
        assertEquals(this.m_a1, itr.next());
        assertTrue(itr.hasNext());
        assertEquals(this.m_a2, itr.next());
        assertFalse(itr.hasNext());
        Iterator itr2 = this.m_rs.sigs();
        assertTrue(itr2.hasNext());
        assertEquals(this.m_s1, itr2.next());
        assertFalse(itr2.hasNext());
    }

    public void test_noncycling_iterator() {
        this.m_rs.addRR(this.m_a1);
        this.m_rs.addRR(this.m_a2);
        Iterator itr = this.m_rs.rrs(false);
        assertTrue(itr.hasNext());
        assertEquals(this.m_a1, itr.next());
        assertTrue(itr.hasNext());
        assertEquals(this.m_a2, itr.next());
        Iterator itr2 = this.m_rs.rrs(false);
        assertTrue(itr2.hasNext());
        assertEquals(this.m_a1, itr2.next());
        assertTrue(itr2.hasNext());
        assertEquals(this.m_a2, itr2.next());
    }
}
