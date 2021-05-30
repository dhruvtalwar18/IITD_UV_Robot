package org.apache.xmlrpc.serializer;

import java.util.List;
import org.apache.xmlrpc.common.TypeFactory;
import org.apache.xmlrpc.common.XmlRpcStreamConfig;
import org.xml.sax.ContentHandler;
import org.xml.sax.SAXException;

public class ListSerializer extends ObjectArraySerializer {
    public ListSerializer(TypeFactory pTypeFactory, XmlRpcStreamConfig pConfig) {
        super(pTypeFactory, pConfig);
    }

    /* access modifiers changed from: protected */
    public void writeData(ContentHandler pHandler, Object pObject) throws SAXException {
        List data = (List) pObject;
        for (int i = 0; i < data.size(); i++) {
            writeObject(pHandler, data.get(i));
        }
    }
}
