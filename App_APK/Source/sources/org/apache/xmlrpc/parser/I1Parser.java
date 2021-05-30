package org.apache.xmlrpc.parser;

import org.xml.sax.SAXException;
import org.xml.sax.SAXParseException;

public class I1Parser extends AtomicParser {
    /* access modifiers changed from: protected */
    public void setResult(String pResult) throws SAXException {
        try {
            super.setResult(new Byte(pResult.trim()));
        } catch (NumberFormatException e) {
            throw new SAXParseException("Failed to parse byte value: " + pResult, getDocumentLocator());
        }
    }
}
