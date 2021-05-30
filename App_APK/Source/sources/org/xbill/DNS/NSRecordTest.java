package org.xbill.DNS;

import junit.framework.TestCase;

public class NSRecordTest extends TestCase {
    public void test_ctor_0arg() {
        NSRecord d = new NSRecord();
        assertNull(d.getName());
        assertNull(d.getTarget());
        assertNull(d.getAdditionalName());
    }

    public void test_ctor_4arg() throws TextParseException {
        Name n = Name.fromString("my.name.");
        Name a = Name.fromString("my.alias.");
        NSRecord d = new NSRecord(n, 1, 703710, a);
        assertEquals(n, d.getName());
        assertEquals(2, d.getType());
        assertEquals(1, d.getDClass());
        assertEquals(703710, d.getTTL());
        assertEquals(a, d.getTarget());
        assertEquals(a, d.getAdditionalName());
    }

    public void test_getObject() {
        assertTrue(new NSRecord().getObject() instanceof NSRecord);
    }
}
