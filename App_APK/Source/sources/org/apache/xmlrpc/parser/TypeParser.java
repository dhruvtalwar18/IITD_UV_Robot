package org.apache.xmlrpc.parser;

import org.apache.xmlrpc.XmlRpcException;
import org.xml.sax.ContentHandler;

public interface TypeParser extends ContentHandler {
    Object getResult() throws XmlRpcException;
}
