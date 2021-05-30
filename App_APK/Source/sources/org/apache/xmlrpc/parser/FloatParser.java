package org.apache.xmlrpc.parser;

import org.xml.sax.SAXException;
import org.xml.sax.SAXParseException;

public class FloatParser extends AtomicParser {
    /* access modifiers changed from: protected */
    public void setResult(String pResult) throws SAXException {
        try {
            super.setResult(new Float(pResult));
        } catch (NumberFormatException e) {
            throw new SAXParseException("Failed to parse float value: " + pResult, getDocumentLocator());
        }
    }
}
