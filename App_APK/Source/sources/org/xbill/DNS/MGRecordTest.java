package org.xbill.DNS;

import junit.framework.TestCase;

public class MGRecordTest extends TestCase {
    public void test_ctor_0arg() {
        MGRecord d = new MGRecord();
        assertNull(d.getName());
        assertNull(d.getMailbox());
    }

    public void test_ctor_4arg() throws TextParseException {
        Name n = Name.fromString("my.name.");
        Name a = Name.fromString("my.alias.");
        MGRecord d = new MGRecord(n, 1, 703710, a);
        assertEquals(n, d.getName());
        assertEquals(8, d.getType());
        assertEquals(1, d.getDClass());
        assertEquals(703710, d.getTTL());
        assertEquals(a, d.getMailbox());
    }

    public void test_getObject() {
        assertTrue(new MGRecord().getObject() instanceof MGRecord);
    }
}
