package org.apache.xmlrpc.parser;

import java.math.BigInteger;
import org.xml.sax.SAXException;
import org.xml.sax.SAXParseException;

public class BigIntegerParser extends AtomicParser {
    /* access modifiers changed from: protected */
    public void setResult(String pResult) throws SAXException {
        try {
            super.setResult(new BigInteger(pResult));
        } catch (NumberFormatException e) {
            throw new SAXParseException("Failed to parse BigInteger value: " + pResult, getDocumentLocator());
        }
    }
}
