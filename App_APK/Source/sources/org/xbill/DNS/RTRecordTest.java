package org.xbill.DNS;

import junit.framework.TestCase;
import org.apache.commons.net.telnet.TelnetCommand;

public class RTRecordTest extends TestCase {
    public void test_getObject() {
        assertTrue(new RTRecord().getObject() instanceof RTRecord);
    }

    public void test_ctor_5arg() throws TextParseException {
        Name n = Name.fromString("My.Name.");
        Name m = Name.fromString("My.OtherName.");
        RTRecord d = new RTRecord(n, 1, 703710, TelnetCommand.NOP, m);
        assertEquals(n, d.getName());
        assertEquals(21, d.getType());
        assertEquals(1, d.getDClass());
        assertEquals(703710, d.getTTL());
        assertEquals(TelnetCommand.NOP, d.getPreference());
        assertEquals(m, d.getIntermediateHost());
    }
}
