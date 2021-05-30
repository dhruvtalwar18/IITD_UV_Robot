package org.xbill.DNS;

import junit.framework.TestCase;

public class NSAP_PTRRecordTest extends TestCase {
    public void test_ctor_0arg() {
        NSAP_PTRRecord d = new NSAP_PTRRecord();
        assertNull(d.getName());
        assertNull(d.getTarget());
    }

    public void test_ctor_4arg() throws TextParseException {
        Name n = Name.fromString("my.name.");
        Name a = Name.fromString("my.alias.");
        NSAP_PTRRecord d = new NSAP_PTRRecord(n, 1, 703710, a);
        assertEquals(n, d.getName());
        assertEquals(23, d.getType());
        assertEquals(1, d.getDClass());
        assertEquals(703710, d.getTTL());
        assertEquals(a, d.getTarget());
    }

    public void test_getObject() {
        assertTrue(new NSAP_PTRRecord().getObject() instanceof NSAP_PTRRecord);
    }
}
