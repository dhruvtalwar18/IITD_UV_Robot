package org.apache.ws.commons.serialize;

import java.nio.charset.Charset;
import java.nio.charset.CharsetEncoder;
import org.xml.sax.SAXException;

public class CharSetXMLWriter extends XMLWriterImpl {
    private CharsetEncoder charsetEncoder;

    public void startDocument() throws SAXException {
        Charset charSet = Charset.forName(getEncoding());
        if (charSet.canEncode()) {
            this.charsetEncoder = charSet.newEncoder();
        }
    }

    public boolean canEncode(char c) {
        if (this.charsetEncoder == null) {
            return false;
        }
        return this.charsetEncoder.canEncode(c);
    }
}
