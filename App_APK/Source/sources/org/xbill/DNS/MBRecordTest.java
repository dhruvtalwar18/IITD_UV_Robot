package org.xbill.DNS;

import junit.framework.TestCase;

public class MBRecordTest extends TestCase {
    public void test_ctor_0arg() {
        MBRecord d = new MBRecord();
        assertNull(d.getName());
        assertNull(d.getAdditionalName());
        assertNull(d.getMailbox());
    }

    public void test_ctor_4arg() throws TextParseException {
        Name n = Name.fromString("my.name.");
        Name a = Name.fromString("my.alias.");
        MBRecord d = new MBRecord(n, 1, 703710, a);
        assertEquals(n, d.getName());
        assertEquals(7, d.getType());
        assertEquals(1, d.getDClass());
        assertEquals(703710, d.getTTL());
        assertEquals(a, d.getAdditionalName());
        assertEquals(a, d.getMailbox());
    }

    public void test_getObject() {
        assertTrue(new MBRecord().getObject() instanceof MBRecord);
    }
}
