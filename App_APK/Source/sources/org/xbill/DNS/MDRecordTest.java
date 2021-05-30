package org.xbill.DNS;

import junit.framework.TestCase;

public class MDRecordTest extends TestCase {
    public void test_ctor_0arg() {
        MDRecord d = new MDRecord();
        assertNull(d.getName());
        assertNull(d.getAdditionalName());
        assertNull(d.getMailAgent());
    }

    public void test_ctor_4arg() throws TextParseException {
        Name n = Name.fromString("my.name.");
        Name a = Name.fromString("my.alias.");
        MDRecord d = new MDRecord(n, 1, 703710, a);
        assertEquals(n, d.getName());
        assertEquals(3, d.getType());
        assertEquals(1, d.getDClass());
        assertEquals(703710, d.getTTL());
        assertEquals(a, d.getAdditionalName());
        assertEquals(a, d.getMailAgent());
    }

    public void test_getObject() {
        assertTrue(new MDRecord().getObject() instanceof MDRecord);
    }
}
