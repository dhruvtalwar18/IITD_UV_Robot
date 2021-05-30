package org.apache.xmlrpc.serializer;

import org.xml.sax.ContentHandler;
import org.xml.sax.SAXException;

public class BooleanSerializer extends TypeSerializerImpl {
    public static final String BOOLEAN_TAG = "boolean";
    private static final char[] FALSE = {'0'};
    private static final char[] TRUE = {'1'};

    public void write(ContentHandler pHandler, Object pObject) throws SAXException {
        write(pHandler, BOOLEAN_TAG, ((Boolean) pObject).booleanValue() ? TRUE : FALSE);
    }
}
