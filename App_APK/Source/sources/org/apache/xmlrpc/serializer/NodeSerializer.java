package org.apache.xmlrpc.serializer;

import org.apache.ws.commons.serialize.DOMSerializer;
import org.w3c.dom.Node;
import org.xml.sax.ContentHandler;
import org.xml.sax.SAXException;

public class NodeSerializer extends ExtSerializer {
    public static final String DOM_TAG = "dom";
    private static final DOMSerializer ser = new DOMSerializer();

    static {
        ser.setStartingDocument(false);
    }

    /* access modifiers changed from: protected */
    public String getTagName() {
        return DOM_TAG;
    }

    /* access modifiers changed from: protected */
    public void serialize(ContentHandler pHandler, Object pObject) throws SAXException {
        ser.serialize((Node) pObject, pHandler);
    }
}
