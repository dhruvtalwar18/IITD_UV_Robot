package org.xbill.DNS;

import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.Arrays;
import junit.framework.Test;
import junit.framework.TestCase;
import junit.framework.TestSuite;

public class MessageTest {
    static /* synthetic */ Class class$org$xbill$DNS$MessageTest$Test_init;

    public static class Test_init extends TestCase {
        public void test_0arg() {
            Message m = new Message();
            assertTrue(Arrays.equals(new Record[0], m.getSectionArray(0)));
            assertTrue(Arrays.equals(new Record[0], m.getSectionArray(1)));
            assertTrue(Arrays.equals(new Record[0], m.getSectionArray(2)));
            assertTrue(Arrays.equals(new Record[0], m.getSectionArray(3)));
            try {
                m.getSectionArray(4);
                fail("IndexOutOfBoundsException not thrown");
            } catch (IndexOutOfBoundsException e) {
            }
            Header h = m.getHeader();
            assertEquals(0, h.getCount(0));
            assertEquals(0, h.getCount(1));
            assertEquals(0, h.getCount(2));
            assertEquals(0, h.getCount(3));
        }

        public void test_1arg() {
            Message m = new Message(10);
            assertEquals(new Header(10).toString(), m.getHeader().toString());
            assertTrue(Arrays.equals(new Record[0], m.getSectionArray(0)));
            assertTrue(Arrays.equals(new Record[0], m.getSectionArray(1)));
            assertTrue(Arrays.equals(new Record[0], m.getSectionArray(2)));
            assertTrue(Arrays.equals(new Record[0], m.getSectionArray(3)));
            try {
                m.getSectionArray(4);
                fail("IndexOutOfBoundsException not thrown");
            } catch (IndexOutOfBoundsException e) {
            }
            Header h = m.getHeader();
            assertEquals(0, h.getCount(0));
            assertEquals(0, h.getCount(1));
            assertEquals(0, h.getCount(2));
            assertEquals(0, h.getCount(3));
        }

        public void test_newQuery() throws TextParseException, UnknownHostException {
            ARecord ar = new ARecord(Name.fromString("The.Name."), 1, 1, InetAddress.getByName("192.168.101.110"));
            Message m = Message.newQuery(ar);
            assertTrue(Arrays.equals(new Record[]{ar}, m.getSectionArray(0)));
            assertTrue(Arrays.equals(new Record[0], m.getSectionArray(1)));
            assertTrue(Arrays.equals(new Record[0], m.getSectionArray(2)));
            assertTrue(Arrays.equals(new Record[0], m.getSectionArray(3)));
            Header h = m.getHeader();
            assertEquals(1, h.getCount(0));
            assertEquals(0, h.getCount(1));
            assertEquals(0, h.getCount(2));
            assertEquals(0, h.getCount(3));
            assertEquals(0, h.getOpcode());
            assertEquals(true, h.getFlag(7));
        }
    }

    public static Test suite() {
        Class cls;
        TestSuite s = new TestSuite();
        if (class$org$xbill$DNS$MessageTest$Test_init == null) {
            cls = class$("org.xbill.DNS.MessageTest$Test_init");
            class$org$xbill$DNS$MessageTest$Test_init = cls;
        } else {
            cls = class$org$xbill$DNS$MessageTest$Test_init;
        }
        s.addTestSuite(cls);
        return s;
    }

    static /* synthetic */ Class class$(String x0) {
        try {
            return Class.forName(x0);
        } catch (ClassNotFoundException x1) {
            throw new NoClassDefFoundError().initCause(x1);
        }
    }
}
