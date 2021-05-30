package org.apache.xmlrpc.parser;

import javax.xml.namespace.QName;
import org.apache.ws.commons.util.NamespaceContextImpl;
import org.apache.xmlrpc.XmlRpcException;
import org.apache.xmlrpc.common.TypeFactory;
import org.apache.xmlrpc.common.XmlRpcExtensionException;
import org.apache.xmlrpc.common.XmlRpcStreamConfig;
import org.apache.xmlrpc.serializer.XmlRpcWriter;
import org.xml.sax.Attributes;
import org.xml.sax.SAXException;
import org.xml.sax.SAXParseException;

public abstract class RecursiveTypeParserImpl extends TypeParserImpl {
    protected final XmlRpcStreamConfig cfg;
    private final NamespaceContextImpl context;
    private final TypeFactory factory;
    private boolean inValueTag;
    private StringBuffer text = new StringBuffer();
    private TypeParser typeParser;

    /* access modifiers changed from: protected */
    public abstract void addResult(Object obj) throws SAXException;

    protected RecursiveTypeParserImpl(XmlRpcStreamConfig pConfig, NamespaceContextImpl pContext, TypeFactory pFactory) {
        this.cfg = pConfig;
        this.context = pContext;
        this.factory = pFactory;
    }

    /* access modifiers changed from: protected */
    public void startValueTag() throws SAXException {
        this.inValueTag = true;
        this.text.setLength(0);
        this.typeParser = null;
    }

    /* access modifiers changed from: protected */
    public void endValueTag() throws SAXException {
        if (!this.inValueTag) {
            throw new SAXParseException("Invalid state: Not inside value tag.", getDocumentLocator());
        } else if (this.typeParser == null) {
            addResult(this.text.toString());
            this.text.setLength(0);
        } else {
            this.typeParser.endDocument();
            try {
                addResult(this.typeParser.getResult());
                this.typeParser = null;
            } catch (XmlRpcException e) {
                throw new SAXException(e);
            }
        }
    }

    public void startDocument() throws SAXException {
        this.inValueTag = false;
        this.text.setLength(0);
        this.typeParser = null;
    }

    public void endElement(String pURI, String pLocalName, String pQName) throws SAXException {
        if (!this.inValueTag) {
            throw new SAXParseException("Invalid state: Not inside value tag.", getDocumentLocator());
        } else if (this.typeParser != null) {
            this.typeParser.endElement(pURI, pLocalName, pQName);
        } else {
            throw new SAXParseException("Invalid state: No type parser configured.", getDocumentLocator());
        }
    }

    public void startElement(String pURI, String pLocalName, String pQName, Attributes pAttrs) throws SAXException {
        if (this.inValueTag) {
            if (this.typeParser == null) {
                this.typeParser = this.factory.getParser(this.cfg, this.context, pURI, pLocalName);
                if (this.typeParser != null) {
                    this.typeParser.setDocumentLocator(getDocumentLocator());
                    this.typeParser.startDocument();
                    if (this.text.length() > 0) {
                        this.typeParser.characters(this.text.toString().toCharArray(), 0, this.text.length());
                        this.text.setLength(0);
                    }
                } else if (!XmlRpcWriter.EXTENSIONS_URI.equals(pURI) || this.cfg.isEnabledForExtensions()) {
                    throw new SAXParseException("Unknown type: " + new QName(pURI, pLocalName), getDocumentLocator());
                } else {
                    String msg = "The tag " + new QName(pURI, pLocalName) + " is invalid, if isEnabledForExtensions() == false.";
                    throw new SAXParseException(msg, getDocumentLocator(), new XmlRpcExtensionException(msg));
                }
            }
            this.typeParser.startElement(pURI, pLocalName, pQName, pAttrs);
            return;
        }
        throw new SAXParseException("Invalid state: Not inside value tag.", getDocumentLocator());
    }

    public void characters(char[] pChars, int pOffset, int pLength) throws SAXException {
        if (this.typeParser != null) {
            this.typeParser.characters(pChars, pOffset, pLength);
        } else if (this.inValueTag) {
            this.text.append(pChars, pOffset, pLength);
        } else {
            super.characters(pChars, pOffset, pLength);
        }
    }

    public void ignorableWhitespace(char[] pChars, int pOffset, int pLength) throws SAXException {
        if (this.typeParser != null) {
            this.typeParser.ignorableWhitespace(pChars, pOffset, pLength);
        } else if (this.inValueTag) {
            this.text.append(pChars, pOffset, pLength);
        } else {
            super.ignorableWhitespace(pChars, pOffset, pLength);
        }
    }

    public void processingInstruction(String pTarget, String pData) throws SAXException {
        if (this.typeParser == null) {
            super.processingInstruction(pTarget, pData);
        } else {
            this.typeParser.processingInstruction(pTarget, pData);
        }
    }

    public void skippedEntity(String pEntity) throws SAXException {
        if (this.typeParser == null) {
            super.skippedEntity(pEntity);
        } else {
            this.typeParser.skippedEntity(pEntity);
        }
    }

    public void startPrefixMapping(String pPrefix, String pURI) throws SAXException {
        if (this.typeParser == null) {
            super.startPrefixMapping(pPrefix, pURI);
        } else {
            this.context.startPrefixMapping(pPrefix, pURI);
        }
    }

    public void endPrefixMapping(String pPrefix) throws SAXException {
        if (this.typeParser == null) {
            super.endPrefixMapping(pPrefix);
        } else {
            this.context.endPrefixMapping(pPrefix);
        }
    }
}
