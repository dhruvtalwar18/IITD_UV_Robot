package org.xbill.DNS;

import java.util.Arrays;
import junit.framework.TestCase;
import org.apache.commons.net.telnet.TelnetCommand;

public class MXRecordTest extends TestCase {
    public void test_getObject() {
        assertTrue(new MXRecord().getObject() instanceof MXRecord);
    }

    public void test_ctor_5arg() throws TextParseException {
        Name n = Name.fromString("My.Name.");
        Name m = Name.fromString("My.OtherName.");
        MXRecord d = new MXRecord(n, 1, 703710, TelnetCommand.NOP, m);
        assertEquals(n, d.getName());
        assertEquals(15, d.getType());
        assertEquals(1, d.getDClass());
        assertEquals(703710, d.getTTL());
        assertEquals(TelnetCommand.NOP, d.getPriority());
        assertEquals(m, d.getTarget());
        assertEquals(m, d.getAdditionalName());
    }

    public void test_rrToWire() throws TextParseException {
        MXRecord mr = new MXRecord(Name.fromString("My.Name."), 1, 45359, 7979, Name.fromString("M.O.n."));
        DNSOutput dout = new DNSOutput();
        mr.rrToWire(dout, (Compression) null, true);
        assertTrue(Arrays.equals(new byte[]{31, 43, 1, 109, 1, 111, 1, 110, 0}, dout.toByteArray()));
        DNSOutput dout2 = new DNSOutput();
        mr.rrToWire(dout2, (Compression) null, false);
        assertTrue(Arrays.equals(new byte[]{31, 43, 1, 77, 1, 79, 1, 110, 0}, dout2.toByteArray()));
    }
}
