package org.xbill.DNS;

import junit.framework.TestCase;

public class SectionTest extends TestCase {
    public void test_string() {
        assertEquals("au", Section.string(2));
        try {
            Section.string(-1);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e) {
        }
        try {
            Section.string(4);
            fail("IllegalArgumentException not thrown");
        } catch (IllegalArgumentException e2) {
        }
    }

    public void test_value() {
        assertEquals(3, Section.value("ad"));
        assertEquals(-1, Section.value("THIS IS DEFINITELY UNKNOWN"));
        assertEquals(-1, Section.value(""));
    }

    public void test_longString() {
        assertEquals("ADDITIONAL RECORDS", Section.longString(3));
        try {
            Section.longString(-1);
        } catch (IllegalArgumentException e) {
        }
        try {
            Section.longString(4);
        } catch (IllegalArgumentException e2) {
        }
    }

    public void test_updString() {
        assertEquals("ZONE", Section.updString(0));
        try {
            Section.longString(-1);
        } catch (IllegalArgumentException e) {
        }
        try {
            Section.longString(4);
        } catch (IllegalArgumentException e2) {
        }
    }
}
