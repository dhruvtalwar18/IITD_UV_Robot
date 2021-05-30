package org.xbill.DNS;

import junit.framework.TestCase;

public class CNAMERecordTest extends TestCase {
    public void test_ctor_0arg() {
        CNAMERecord d = new CNAMERecord();
        assertNull(d.getName());
        assertNull(d.getTarget());
        assertNull(d.getAlias());
    }

    public void test_ctor_4arg() throws TextParseException {
        Name n = Name.fromString("my.name.");
        Name a = Name.fromString("my.alias.");
        CNAMERecord d = new CNAMERecord(n, 1, 703710, a);
        assertEquals(n, d.getName());
        assertEquals(5, d.getType());
        assertEquals(1, d.getDClass());
        assertEquals(703710, d.getTTL());
        assertEquals(a, d.getTarget());
        assertEquals(a, d.getAlias());
    }

    public void test_getObject() {
        assertTrue(new CNAMERecord().getObject() instanceof CNAMERecord);
    }
}
