package org.apache.ws.commons.serialize;

import java.util.ArrayList;
import java.util.List;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.Text;
import org.xml.sax.Attributes;
import org.xml.sax.ContentHandler;
import org.xml.sax.Locator;
import org.xml.sax.SAXException;

public class DOMBuilder implements ContentHandler {
    private Node currentNode;
    private Document document;
    private Locator locator;
    private boolean prefixMappingIsAttribute;
    private List prefixes;
    private Node target;

    public boolean isPrefixMappingIsAttribute() {
        return this.prefixMappingIsAttribute;
    }

    public void setPrefixMappingIsAttribute(boolean pPrefixMappingIsAttribute) {
        this.prefixMappingIsAttribute = pPrefixMappingIsAttribute;
    }

    public void setDocument(Document pDocument) {
        this.document = pDocument;
    }

    public Document getDocument() {
        return this.document;
    }

    public void setDocumentLocator(Locator pLocator) {
        this.locator = pLocator;
    }

    public Locator getDocumentLocator() {
        return this.locator;
    }

    public void setTarget(Node pNode) {
        this.target = pNode;
        this.currentNode = pNode;
        if (getDocument() == null) {
            setDocument(pNode.getNodeType() == 9 ? (Document) pNode : pNode.getOwnerDocument());
        }
    }

    public Node getTarget() {
        return this.target;
    }

    public void startDocument() throws SAXException {
    }

    public void endDocument() throws SAXException {
    }

    public void startPrefixMapping(String prefix, String uri) throws SAXException {
        if (isPrefixMappingIsAttribute()) {
            if (this.prefixes == null) {
                this.prefixes = new ArrayList();
            }
            this.prefixes.add(prefix);
            this.prefixes.add(uri);
        }
    }

    public void endPrefixMapping(String prefix) throws SAXException {
    }

    public void startElement(String pNamespaceURI, String pLocalName, String pQName, Attributes pAttr) throws SAXException {
        Element element;
        Document doc = getDocument();
        if (pNamespaceURI == null || pNamespaceURI.length() == 0) {
            element = doc.createElement(pQName);
        } else {
            element = doc.createElementNS(pNamespaceURI, pQName);
        }
        if (pAttr != null) {
            for (int i = 0; i < pAttr.getLength(); i++) {
                String uri = pAttr.getURI(i);
                String qName = pAttr.getQName(i);
                String value = pAttr.getValue(i);
                if (uri == null || uri.length() == 0) {
                    element.setAttribute(qName, value);
                } else {
                    element.setAttributeNS(uri, qName, value);
                }
            }
        }
        if (this.prefixes != null) {
            for (int i2 = 0; i2 < this.prefixes.size(); i2 += 2) {
                String prefix = (String) this.prefixes.get(i2);
                String uri2 = (String) this.prefixes.get(i2 + 1);
                if (prefix == null || "".equals(prefix)) {
                    element.setAttributeNS("http://www.w3.org/2000/xmlns/", "xmlns", uri2);
                } else {
                    StringBuffer stringBuffer = new StringBuffer();
                    stringBuffer.append("xmlns:");
                    stringBuffer.append(prefix);
                    element.setAttributeNS("http://www.w3.org/2000/xmlns/", stringBuffer.toString(), uri2);
                }
            }
            this.prefixes.clear();
        }
        this.currentNode.appendChild(element);
        this.currentNode = element;
    }

    public void endElement(String namespaceURI, String localName, String qName) throws SAXException {
        this.currentNode = this.currentNode.getParentNode();
    }

    public void characters(char[] ch, int start, int length) throws SAXException {
        Node node = this.currentNode.getLastChild();
        String s = new String(ch, start, length);
        if (node == null || node.getNodeType() != 3) {
            this.currentNode.appendChild(getDocument().createTextNode(s));
            return;
        }
        ((Text) node).appendData(s);
    }

    public void ignorableWhitespace(char[] ch, int start, int length) throws SAXException {
        characters(ch, start, length);
    }

    public void processingInstruction(String pTarget, String pData) throws SAXException {
        this.currentNode.appendChild(getDocument().createProcessingInstruction(pTarget, pData));
    }

    public void skippedEntity(String pName) throws SAXException {
        this.currentNode.appendChild(getDocument().createEntityReference(pName));
    }
}
