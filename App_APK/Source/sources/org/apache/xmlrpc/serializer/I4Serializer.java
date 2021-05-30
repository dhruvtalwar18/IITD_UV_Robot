package org.apache.xmlrpc.serializer;

import org.xml.sax.ContentHandler;
import org.xml.sax.SAXException;

public class I4Serializer extends TypeSerializerImpl {
    public static final String I4_TAG = "i4";
    public static final String INT_TAG = "int";

    public void write(ContentHandler pHandler, Object pObject) throws SAXException {
        write(pHandler, I4_TAG, pObject.toString());
    }
}
