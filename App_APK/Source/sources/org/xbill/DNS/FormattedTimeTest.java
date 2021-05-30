package org.xbill.DNS;

import java.util.Date;
import java.util.GregorianCalendar;
import java.util.TimeZone;
import junit.framework.TestCase;

public class FormattedTimeTest extends TestCase {
    public void test_format() {
        GregorianCalendar cal = new GregorianCalendar(TimeZone.getTimeZone("UTC"));
        cal.set(2005, 2, 19, 4, 4, 5);
        assertEquals("20050319040405", FormattedTime.format(cal.getTime()));
    }

    public void test_parse() throws TextParseException {
        GregorianCalendar cal = new GregorianCalendar(TimeZone.getTimeZone("UTC"));
        cal.set(2005, 2, 19, 4, 4, 5);
        cal.set(14, 0);
        Date out = FormattedTime.parse("20050319040405");
        GregorianCalendar cal2 = new GregorianCalendar(TimeZone.getTimeZone("UTC"));
        cal2.setTimeInMillis(out.getTime());
        cal2.set(14, 0);
        assertEquals(cal, cal2);
    }

    public void test_parse_invalid() {
        try {
            FormattedTime.parse("2004010101010");
            fail("TextParseException not thrown");
        } catch (TextParseException e) {
        }
        try {
            FormattedTime.parse("200401010101010");
            fail("TextParseException not thrown");
        } catch (TextParseException e2) {
        }
        try {
            FormattedTime.parse("2004010101010A");
            fail("TextParseException not thrown");
        } catch (TextParseException e3) {
        }
    }
}
