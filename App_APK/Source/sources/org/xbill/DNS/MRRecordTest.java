package org.xbill.DNS;

import junit.framework.TestCase;

public class MRRecordTest extends TestCase {
    public void test_ctor_0arg() {
        MRRecord d = new MRRecord();
        assertNull(d.getName());
        assertNull(d.getNewName());
    }

    public void test_ctor_4arg() throws TextParseException {
        Name n = Name.fromString("my.name.");
        Name a = Name.fromString("my.alias.");
        MRRecord d = new MRRecord(n, 1, 703710, a);
        assertEquals(n, d.getName());
        assertEquals(9, d.getType());
        assertEquals(1, d.getDClass());
        assertEquals(703710, d.getTTL());
        assertEquals(a, d.getNewName());
    }

    public void test_getObject() {
        assertTrue(new MRRecord().getObject() instanceof MRRecord);
    }
}
