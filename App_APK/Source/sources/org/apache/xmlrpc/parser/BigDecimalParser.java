package org.apache.xmlrpc.parser;

import java.math.BigDecimal;
import org.xml.sax.SAXException;
import org.xml.sax.SAXParseException;

public class BigDecimalParser extends AtomicParser {
    /* access modifiers changed from: protected */
    public void setResult(String pResult) throws SAXException {
        try {
            super.setResult(new BigDecimal(pResult));
        } catch (NumberFormatException e) {
            throw new SAXParseException("Failed to parse BigDecimal value: " + pResult, getDocumentLocator());
        }
    }
}
