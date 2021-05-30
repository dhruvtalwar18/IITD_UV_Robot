package org.xbill.DNS;

import junit.framework.TestCase;

public class CompressionTest extends TestCase {
    public void setUp() {
        Options.set("verbosecompression");
    }

    public void test() throws TextParseException {
        Compression c = new Compression();
        Name n = Name.fromString("www.amazon.com.");
        c.add(10, n);
        assertEquals(10, c.get(n));
        Name n2 = Name.fromString("www.cnn.com.");
        c.add(10, n2);
        assertEquals(10, c.get(n2));
    }
}
