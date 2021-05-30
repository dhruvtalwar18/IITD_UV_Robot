package org.apache.xmlrpc.parser;

import org.xml.sax.SAXException;
import org.xml.sax.SAXParseException;

public class NullParser extends AtomicParser {
    /* access modifiers changed from: protected */
    public void setResult(String pResult) throws SAXException {
        if (pResult == null || "".equals(pResult.trim())) {
            super.setResult((Object) null);
            return;
        }
        throw new SAXParseException("Unexpected characters in nil element.", getDocumentLocator());
    }
}
