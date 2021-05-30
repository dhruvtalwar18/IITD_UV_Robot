package org.apache.xmlrpc.serializer;

import org.xml.sax.ContentHandler;
import org.xml.sax.SAXException;

public class I8Serializer extends TypeSerializerImpl {
    public static final String EX_I8_TAG = "ex:i8";
    public static final String I8_TAG = "i8";

    public void write(ContentHandler pHandler, Object pObject) throws SAXException {
        write(pHandler, I8_TAG, EX_I8_TAG, pObject.toString());
    }
}
