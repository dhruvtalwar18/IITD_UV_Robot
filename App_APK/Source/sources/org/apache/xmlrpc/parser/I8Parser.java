package org.apache.xmlrpc.parser;

import org.xml.sax.SAXException;
import org.xml.sax.SAXParseException;

public class I8Parser extends AtomicParser {
    /* access modifiers changed from: protected */
    public void setResult(String pResult) throws SAXException {
        try {
            super.setResult(new Long(pResult.trim()));
        } catch (NumberFormatException e) {
            throw new SAXParseException("Failed to parse long value: " + pResult, getDocumentLocator());
        }
    }
}
