package org.apache.xmlrpc.serializer;

import org.xml.sax.ContentHandler;
import org.xml.sax.SAXException;

public class FloatSerializer extends TypeSerializerImpl {
    public static final String EX_FLOAT_TAG = "ex:float";
    public static final String FLOAT_TAG = "float";

    public void write(ContentHandler pHandler, Object pObject) throws SAXException {
        write(pHandler, FLOAT_TAG, EX_FLOAT_TAG, pObject.toString());
    }
}
