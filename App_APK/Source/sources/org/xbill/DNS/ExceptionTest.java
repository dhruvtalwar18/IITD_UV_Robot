package org.xbill.DNS;

import junit.framework.TestCase;

public class ExceptionTest extends TestCase {
    public void test_InvalidDClassException() {
        assertEquals("Invalid DNS class: 10", new InvalidDClassException(10).getMessage());
    }

    public void test_InvalidTTLException() {
        assertEquals("Invalid DNS TTL: 32345", new InvalidTTLException(32345).getMessage());
    }

    public void test_InvalidTypeException() {
        assertEquals("Invalid DNS type: 32345", new InvalidTypeException(32345).getMessage());
    }

    public void test_NameTooLongException() {
        assertNull(new NameTooLongException().getMessage());
        assertEquals("This is my too long name", new NameTooLongException("This is my too long name").getMessage());
    }

    public void test_RelativeNameException() throws TextParseException {
        assertEquals("This is my relative name", new RelativeNameException("This is my relative name").getMessage());
        assertEquals("'relative' is not an absolute name", new RelativeNameException(Name.fromString("relative")).getMessage());
    }

    public void test_TextParseException() {
        assertNull(new TextParseException().getMessage());
        assertEquals("This is my message", new TextParseException("This is my message").getMessage());
    }

    public void test_WireParseException() {
        assertNull(new WireParseException().getMessage());
        assertEquals("This is my message", new WireParseException("This is my message").getMessage());
    }

    public void test_ZoneTransferException() {
        assertNull(new ZoneTransferException().getMessage());
        assertEquals("This is my message", new ZoneTransferException("This is my message").getMessage());
    }
}
