package org.apache.xmlrpc.parser;

import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import org.apache.ws.commons.serialize.DOMBuilder;
import org.apache.xmlrpc.serializer.NodeSerializer;
import org.xml.sax.ContentHandler;
import org.xml.sax.SAXException;

public class NodeParser extends ExtParser {
    private static final DocumentBuilderFactory dbf = DocumentBuilderFactory.newInstance();
    private final DOMBuilder builder = new DOMBuilder();

    /* access modifiers changed from: protected */
    public String getTagName() {
        return NodeSerializer.DOM_TAG;
    }

    /* access modifiers changed from: protected */
    public ContentHandler getExtHandler() throws SAXException {
        try {
            this.builder.setTarget(dbf.newDocumentBuilder().newDocument());
            return this.builder;
        } catch (ParserConfigurationException e) {
            throw new SAXException(e);
        }
    }

    public Object getResult() {
        return this.builder.getTarget();
    }
}
