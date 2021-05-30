package org.apache.xmlrpc.serializer;

import java.text.Format;
import org.xml.sax.ContentHandler;
import org.xml.sax.SAXException;

public class DateSerializer extends TypeSerializerImpl {
    public static final String DATE_TAG = "dateTime.iso8601";
    private final Format format;

    public DateSerializer(Format pFormat) {
        this.format = pFormat;
    }

    public void write(ContentHandler pHandler, Object pObject) throws SAXException {
        write(pHandler, "dateTime.iso8601", this.format.format(pObject));
    }
}
