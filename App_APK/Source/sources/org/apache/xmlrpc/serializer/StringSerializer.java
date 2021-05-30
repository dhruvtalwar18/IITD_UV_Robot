package org.apache.xmlrpc.serializer;

import org.xml.sax.ContentHandler;
import org.xml.sax.SAXException;

public class StringSerializer extends TypeSerializerImpl {
    public static final String STRING_TAG = "string";

    public void write(ContentHandler pHandler, Object pObject) throws SAXException {
        write(pHandler, (String) null, pObject.toString());
    }
}
