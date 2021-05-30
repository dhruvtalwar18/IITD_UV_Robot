package org.apache.xmlrpc.serializer;

import org.xml.sax.ContentHandler;
import org.xml.sax.SAXException;

public class BigIntegerSerializer extends TypeSerializerImpl {
    public static final String BIGINTEGER_TAG = "biginteger";
    private static final String EX_BIGINTEGER_TAG = "ex:biginteger";

    public void write(ContentHandler pHandler, Object pObject) throws SAXException {
        write(pHandler, BIGINTEGER_TAG, EX_BIGINTEGER_TAG, pObject.toString());
    }
}
