package org.apache.xmlrpc.parser;

import org.xml.sax.SAXException;
import org.xml.sax.SAXParseException;

public class I4Parser extends AtomicParser {
    /* access modifiers changed from: protected */
    public void setResult(String pResult) throws SAXException {
        try {
            super.setResult(new Integer(pResult.trim()));
        } catch (NumberFormatException e) {
            throw new SAXParseException("Failed to parse integer value: " + pResult, getDocumentLocator());
        }
    }
}
