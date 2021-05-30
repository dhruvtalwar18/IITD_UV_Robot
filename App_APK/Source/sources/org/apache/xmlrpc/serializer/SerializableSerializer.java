package org.apache.xmlrpc.serializer;

import java.io.IOException;
import java.io.ObjectOutputStream;
import org.apache.ws.commons.util.Base64;
import org.xml.sax.ContentHandler;
import org.xml.sax.SAXException;

public class SerializableSerializer extends TypeSerializerImpl {
    private static final String EX_SERIALIZABLE_TAG = "ex:serializable";
    public static final String SERIALIZABLE_TAG = "serializable";

    public void write(ContentHandler pHandler, Object pObject) throws SAXException {
        pHandler.startElement("", TypeSerializerImpl.VALUE_TAG, TypeSerializerImpl.VALUE_TAG, ZERO_ATTRIBUTES);
        pHandler.startElement("", SERIALIZABLE_TAG, EX_SERIALIZABLE_TAG, ZERO_ATTRIBUTES);
        try {
            ObjectOutputStream oos = new ObjectOutputStream(new Base64.EncoderOutputStream(new Base64.SAXEncoder(new char[1024], 0, (String) null, pHandler)));
            oos.writeObject(pObject);
            oos.close();
            pHandler.endElement("", SERIALIZABLE_TAG, EX_SERIALIZABLE_TAG);
            pHandler.endElement("", TypeSerializerImpl.VALUE_TAG, TypeSerializerImpl.VALUE_TAG);
        } catch (Base64.SAXIOException e) {
            throw e.getSAXException();
        } catch (IOException e2) {
            throw new SAXException(e2);
        }
    }
}
