package org.xbill.DNS;

import junit.framework.TestCase;

public class DNAMERecordTest extends TestCase {
    public void test_ctor_0arg() {
        DNAMERecord d = new DNAMERecord();
        assertNull(d.getName());
        assertNull(d.getTarget());
        assertNull(d.getAlias());
    }

    public void test_ctor_4arg() throws TextParseException {
        Name n = Name.fromString("my.name.");
        Name a = Name.fromString("my.alias.");
        DNAMERecord d = new DNAMERecord(n, 1, 703710, a);
        assertEquals(n, d.getName());
        assertEquals(39, d.getType());
        assertEquals(1, d.getDClass());
        assertEquals(703710, d.getTTL());
        assertEquals(a, d.getTarget());
        assertEquals(a, d.getAlias());
    }

    public void test_getObject() {
        assertTrue(new DNAMERecord().getObject() instanceof DNAMERecord);
    }
}
