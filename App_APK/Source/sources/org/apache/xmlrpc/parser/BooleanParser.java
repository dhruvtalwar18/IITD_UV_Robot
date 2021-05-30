package org.apache.xmlrpc.parser;

import org.xml.sax.SAXException;
import org.xml.sax.SAXParseException;

public class BooleanParser extends AtomicParser {
    /* access modifiers changed from: protected */
    public void setResult(String pResult) throws SAXException {
        String s = pResult.trim();
        if ("1".equals(s)) {
            super.setResult(Boolean.TRUE);
        } else if ("0".equals(s)) {
            super.setResult(Boolean.FALSE);
        } else {
            throw new SAXParseException("Failed to parse boolean value: " + pResult, getDocumentLocator());
        }
    }
}
