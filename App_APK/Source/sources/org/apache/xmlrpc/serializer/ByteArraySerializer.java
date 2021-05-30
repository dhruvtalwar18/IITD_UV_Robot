package org.apache.xmlrpc.serializer;

import java.io.IOException;
import org.apache.ws.commons.util.Base64;
import org.xml.sax.ContentHandler;
import org.xml.sax.SAXException;

public class ByteArraySerializer extends TypeSerializerImpl {
    public static final String BASE_64_TAG = "base64";

    public void write(ContentHandler pHandler, Object pObject) throws SAXException {
        pHandler.startElement("", TypeSerializerImpl.VALUE_TAG, TypeSerializerImpl.VALUE_TAG, ZERO_ATTRIBUTES);
        pHandler.startElement("", "base64", "base64", ZERO_ATTRIBUTES);
        byte[] buffer = (byte[]) pObject;
        if (buffer.length > 0) {
            int i = 1024;
            if (buffer.length < 1024) {
                i = ((buffer.length + 3) / 4) * 4;
            }
            Base64.Encoder encoder = new Base64.SAXEncoder(new char[i], 0, (String) null, pHandler);
            try {
                encoder.write(buffer, 0, buffer.length);
                encoder.flush();
            } catch (Base64.SAXIOException e) {
                throw e.getSAXException();
            } catch (IOException e2) {
                throw new SAXException(e2);
            }
        }
        pHandler.endElement("", "base64", "base64");
        pHandler.endElement("", TypeSerializerImpl.VALUE_TAG, TypeSerializerImpl.VALUE_TAG);
    }
}
