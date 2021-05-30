package org.apache.ws.commons.serialize;

import org.w3c.dom.NamedNodeMap;
import org.w3c.dom.Node;
import org.xml.sax.ContentHandler;
import org.xml.sax.SAXException;
import org.xml.sax.ext.LexicalHandler;
import org.xml.sax.helpers.AttributesImpl;

public class DOMSerializer {
    private boolean namespaceDeclarationAttribute;
    private boolean parentsNamespaceDeclarationDisabled;
    private boolean startingDocument = true;

    public void setNamespaceDeclarationAttribute(boolean pXmlDeclarationAttribute) {
        this.namespaceDeclarationAttribute = pXmlDeclarationAttribute;
    }

    public boolean isNamespaceDeclarationAttribute() {
        return this.namespaceDeclarationAttribute;
    }

    public void setParentsNamespaceDeclarationDisabled(boolean pParentsXmlDeclarationDisabled) {
        this.parentsNamespaceDeclarationDisabled = pParentsXmlDeclarationDisabled;
    }

    public boolean isParentsNamespaceDeclarationDisabled() {
        return this.parentsNamespaceDeclarationDisabled;
    }

    public boolean isStartingDocument() {
        return this.startingDocument;
    }

    public void setStartingDocument(boolean pStartingDocument) {
        this.startingDocument = pStartingDocument;
    }

    /* access modifiers changed from: protected */
    public void doSerializeChilds(Node pNode, ContentHandler pHandler) throws SAXException {
        for (Node child = pNode.getFirstChild(); child != null; child = child.getNextSibling()) {
            doSerialize(child, pHandler);
        }
    }

    private void parentsStartPrefixMappingEvents(Node pNode, ContentHandler pHandler) throws SAXException {
        if (pNode != null) {
            parentsStartPrefixMappingEvents(pNode.getParentNode(), pHandler);
            if (pNode.getNodeType() == 1) {
                startPrefixMappingEvents(pNode, pHandler);
            }
        }
    }

    private void parentsEndPrefixMappingEvents(Node pNode, ContentHandler pHandler) throws SAXException {
        if (pNode != null) {
            if (pNode.getNodeType() == 1) {
                endPrefixMappingEvents(pNode, pHandler);
            }
            parentsEndPrefixMappingEvents(pNode.getParentNode(), pHandler);
        }
    }

    private void startPrefixMappingEvents(Node pNode, ContentHandler pHandler) throws SAXException {
        String prefix;
        NamedNodeMap nnm = pNode.getAttributes();
        if (nnm != null) {
            for (int i = 0; i < nnm.getLength(); i++) {
                Node attr = nnm.item(i);
                if ("http://www.w3.org/2000/xmlns/".equals(attr.getNamespaceURI())) {
                    if ("xmlns".equals(attr.getPrefix())) {
                        prefix = attr.getLocalName();
                    } else if ("xmlns".equals(attr.getNodeName())) {
                        prefix = "";
                    } else {
                        StringBuffer stringBuffer = new StringBuffer();
                        stringBuffer.append("Unable to parse namespace declaration: ");
                        stringBuffer.append(attr.getNodeName());
                        throw new IllegalStateException(stringBuffer.toString());
                    }
                    String uri = attr.getNodeValue();
                    if (uri == null) {
                        uri = "";
                    }
                    pHandler.startPrefixMapping(prefix, uri);
                }
            }
        }
    }

    private void endPrefixMappingEvents(Node pNode, ContentHandler pHandler) throws SAXException {
        NamedNodeMap nnm = pNode.getAttributes();
        if (nnm != null) {
            for (int i = nnm.getLength() - 1; i >= 0; i--) {
                Node attr = nnm.item(i);
                if ("http://www.w3.org/2000/xmlns/".equals(attr.getNamespaceURI())) {
                    pHandler.endPrefixMapping(attr.getLocalName());
                }
            }
        }
    }

    private void characters(ContentHandler pHandler, String pValue, boolean pCdata) throws SAXException {
        LexicalHandler lh = null;
        if (pCdata && (pHandler instanceof LexicalHandler)) {
            lh = (LexicalHandler) pHandler;
        }
        if (lh != null) {
            lh.startCDATA();
        }
        pHandler.characters(pValue.toCharArray(), 0, pValue.length());
        if (lh != null) {
            lh.endCDATA();
        }
    }

    public void serialize(Node pNode, ContentHandler pHandler) throws SAXException {
        if (!isNamespaceDeclarationAttribute() && !isParentsNamespaceDeclarationDisabled()) {
            parentsStartPrefixMappingEvents(pNode.getParentNode(), pHandler);
        }
        doSerialize(pNode, pHandler);
        if (!isNamespaceDeclarationAttribute() && !isParentsNamespaceDeclarationDisabled()) {
            parentsEndPrefixMappingEvents(pNode.getParentNode(), pHandler);
        }
    }

    /* access modifiers changed from: protected */
    public void doSerialize(Node pNode, ContentHandler pHandler) throws SAXException {
        ContentHandler contentHandler = pHandler;
        switch (pNode.getNodeType()) {
            case 1:
                AttributesImpl attr = new AttributesImpl();
                boolean isNamespaceDeclarationAttribute = isNamespaceDeclarationAttribute();
                if (!isNamespaceDeclarationAttribute) {
                    startPrefixMappingEvents(pNode, pHandler);
                }
                NamedNodeMap nnm = pNode.getAttributes();
                if (nnm != null) {
                    for (int i = 0; i < nnm.getLength(); i++) {
                        Node a = nnm.item(i);
                        if (isNamespaceDeclarationAttribute || !"http://www.w3.org/2000/xmlns/".equals(a.getNamespaceURI())) {
                            String aUri = a.getNamespaceURI();
                            String aLocalName = a.getLocalName();
                            String aNodeName = a.getNodeName();
                            if (aLocalName == null) {
                                if (aUri == null || aUri.length() == 0) {
                                    aLocalName = aNodeName;
                                } else {
                                    throw new IllegalStateException("aLocalName is null");
                                }
                            }
                            attr.addAttribute(aUri == null ? "" : aUri, aNodeName, aLocalName, "CDATA", a.getNodeValue());
                        }
                    }
                }
                String nUri = pNode.getNamespaceURI();
                if (nUri == null) {
                    nUri = "";
                }
                contentHandler.startElement(nUri, pNode.getLocalName(), pNode.getNodeName(), attr);
                doSerializeChilds(pNode, pHandler);
                contentHandler.endElement(nUri, pNode.getLocalName(), pNode.getNodeName());
                if (!isNamespaceDeclarationAttribute) {
                    endPrefixMappingEvents(pNode, pHandler);
                    return;
                }
                return;
            case 3:
                characters(contentHandler, pNode.getNodeValue(), false);
                return;
            case 4:
                characters(contentHandler, pNode.getNodeValue(), true);
                return;
            case 5:
                contentHandler.skippedEntity(pNode.getNodeName());
                return;
            case 7:
                contentHandler.processingInstruction(pNode.getNodeName(), pNode.getNodeValue());
                return;
            case 8:
                if (contentHandler instanceof LexicalHandler) {
                    String s = pNode.getNodeValue();
                    ((LexicalHandler) contentHandler).comment(s.toCharArray(), 0, s.length());
                    return;
                }
                return;
            case 9:
                boolean startDocumentEvent = isStartingDocument();
                if (startDocumentEvent) {
                    pHandler.startDocument();
                }
                doSerializeChilds(pNode, pHandler);
                if (startDocumentEvent) {
                    pHandler.endDocument();
                    return;
                }
                return;
            case 11:
                doSerializeChilds(pNode, pHandler);
                return;
            default:
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append("Unknown node type: ");
                stringBuffer.append(pNode.getNodeType());
                throw new IllegalStateException(stringBuffer.toString());
        }
    }
}
