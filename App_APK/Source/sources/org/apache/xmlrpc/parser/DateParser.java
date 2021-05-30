package org.apache.xmlrpc.parser;

import java.text.Format;
import java.text.ParseException;
import org.xml.sax.SAXException;
import org.xml.sax.SAXParseException;

public class DateParser extends AtomicParser {
    private final Format f;

    public DateParser(Format pFormat) {
        this.f = pFormat;
    }

    /* access modifiers changed from: protected */
    public void setResult(String pResult) throws SAXException {
        String msg;
        String s = pResult.trim();
        if (s.length() != 0) {
            try {
                super.setResult(this.f.parseObject(s));
            } catch (ParseException e) {
                int offset = e.getErrorOffset();
                if (e.getErrorOffset() == -1) {
                    msg = "Failed to parse date value: " + pResult;
                } else {
                    msg = "Failed to parse date value " + pResult + " at position " + offset;
                }
                throw new SAXParseException(msg, getDocumentLocator(), e);
            }
        }
    }
}
