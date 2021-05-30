package org.apache.xmlrpc.serializer;

import org.xml.sax.ContentHandler;
import org.xml.sax.SAXException;

public interface TypeSerializer {
    void write(ContentHandler contentHandler, Object obj) throws SAXException;
}
