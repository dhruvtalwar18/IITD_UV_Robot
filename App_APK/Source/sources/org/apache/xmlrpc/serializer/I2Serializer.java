package org.apache.xmlrpc.serializer;

import org.xml.sax.ContentHandler;
import org.xml.sax.SAXException;

public class I2Serializer extends TypeSerializerImpl {
    public static final String EX_I2_TAG = "ex:i2";
    public static final String I2_TAG = "i2";

    public void write(ContentHandler pHandler, Object pObject) throws SAXException {
        write(pHandler, I2_TAG, EX_I2_TAG, pObject.toString());
    }
}
