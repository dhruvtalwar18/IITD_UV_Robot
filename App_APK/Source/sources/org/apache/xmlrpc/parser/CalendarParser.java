package org.apache.xmlrpc.parser;

import java.text.ParseException;
import org.apache.ws.commons.util.XsDateTimeFormat;
import org.xml.sax.SAXException;
import org.xml.sax.SAXParseException;

public class CalendarParser extends AtomicParser {
    private static final XsDateTimeFormat format = new XsDateTimeFormat();

    /* access modifiers changed from: protected */
    public void setResult(String pResult) throws SAXException {
        String msg;
        try {
            super.setResult(format.parseObject(pResult.trim()));
        } catch (ParseException e) {
            if (e.getErrorOffset() == -1) {
                msg = "Failed to parse dateTime value: " + pResult;
            } else {
                msg = "Failed to parse dateTime value " + pResult + " at position " + e.getErrorOffset();
            }
            throw new SAXParseException(msg, getDocumentLocator(), e);
        }
    }
}
