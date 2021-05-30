package org.apache.xmlrpc.serializer;

import org.xml.sax.ContentHandler;
import org.xml.sax.SAXException;

public class I1Serializer extends TypeSerializerImpl {
    public static final String EX_I1_TAG = "ex:i1";
    public static final String I1_TAG = "i1";

    public void write(ContentHandler pHandler, Object pObject) throws SAXException {
        write(pHandler, I1_TAG, EX_I1_TAG, pObject.toString());
    }
}
