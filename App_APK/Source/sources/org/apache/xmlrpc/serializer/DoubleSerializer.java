package org.apache.xmlrpc.serializer;

import org.xml.sax.ContentHandler;
import org.xml.sax.SAXException;

public class DoubleSerializer extends TypeSerializerImpl {
    public static final String DOUBLE_TAG = "double";

    public void write(ContentHandler pHandler, Object pObject) throws SAXException {
        write(pHandler, DOUBLE_TAG, pObject.toString());
    }
}
