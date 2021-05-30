package org.apache.xmlrpc.serializer;

import org.apache.ws.commons.util.XsDateTimeFormat;
import org.xml.sax.ContentHandler;
import org.xml.sax.SAXException;

public class CalendarSerializer extends TypeSerializerImpl {
    public static final String CALENDAR_TAG = "dateTime";
    public static final String DATE_TAG = "dateTime.iso8601";
    private static final String EX_CALENDAR_TAG = "ex:dateTime";
    private static final XsDateTimeFormat format = new XsDateTimeFormat();

    public void write(ContentHandler pHandler, Object pObject) throws SAXException {
        write(pHandler, CALENDAR_TAG, EX_CALENDAR_TAG, format.format(pObject));
    }
}
