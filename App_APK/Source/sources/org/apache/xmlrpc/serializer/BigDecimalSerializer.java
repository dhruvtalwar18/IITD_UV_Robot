package org.apache.xmlrpc.serializer;

import org.xml.sax.ContentHandler;
import org.xml.sax.SAXException;

public class BigDecimalSerializer extends TypeSerializerImpl {
    public static final String BIGDECIMAL_TAG = "bigdecimal";
    private static final String EX_BIGDECIMAL_TAG = "ex:bigdecimal";

    public void write(ContentHandler pHandler, Object pObject) throws SAXException {
        write(pHandler, BIGDECIMAL_TAG, EX_BIGDECIMAL_TAG, pObject.toString());
    }
}
