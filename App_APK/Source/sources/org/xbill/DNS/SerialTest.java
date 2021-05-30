package org.xbill.DNS;

import junit.framework.TestCase;

public class SerialTest extends TestCase {
    public void test_compare_NegativeArg1() {
        try {
            Serial.compare(-1, 1);
            fail("compare accepted negative argument 1");
        } catch (IllegalArgumentException e) {
        }
    }

    public void test_compare_OOBArg1() {
        try {
            Serial.compare(4294967296L, 1);
            fail("compare accepted out-of-bounds argument 1");
        } catch (IllegalArgumentException e) {
        }
    }

    public void test_compare_NegativeArg2() {
        try {
            Serial.compare(1, -1);
            fail("compare accepted negative argument 2");
        } catch (IllegalArgumentException e) {
        }
    }

    public void test_compare_OOBArg2() {
        try {
            Serial.compare(1, 4294967296L);
            fail("compare accepted out-of-bounds argument 1");
        } catch (IllegalArgumentException e) {
        }
    }

    public void test_compare_Arg1Greater() {
        assertTrue(Serial.compare(10, 9) > 0);
    }

    public void test_compare_Arg2Greater() {
        assertTrue(Serial.compare(9, 10) < 0);
    }

    public void test_compare_ArgsEqual() {
        assertEquals(Serial.compare(10, 10), 0);
    }

    public void test_compare_boundary() {
        assertEquals(-1, Serial.compare(4294967295L, 0));
        assertEquals(1, Serial.compare(0, 4294967295L));
    }

    public void test_increment_NegativeArg() {
        try {
            Serial.increment(-1);
            fail("increment accepted negative argument");
        } catch (IllegalArgumentException e) {
        }
    }

    public void test_increment_OOBArg() {
        try {
            Serial.increment(4294967296L);
            fail("increment accepted out-of-bounds argument");
        } catch (IllegalArgumentException e) {
        }
    }

    public void test_increment_reset() {
        assertEquals(0, Serial.increment(4294967295L));
    }

    public void test_increment_normal() {
        assertEquals(1 + 10, Serial.increment(10));
    }
}
