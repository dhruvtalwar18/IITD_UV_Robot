package org.apache.xmlrpc.jaxb;

import javax.xml.bind.JAXBContext;
import javax.xml.bind.JAXBException;
import org.apache.xmlrpc.serializer.ExtSerializer;
import org.xml.sax.Attributes;
import org.xml.sax.ContentHandler;
import org.xml.sax.Locator;
import org.xml.sax.SAXException;

public class JaxbSerializer extends ExtSerializer {
    public static final String JAXB_TAG = "jaxb";
    private final JAXBContext context;

    public JaxbSerializer(JAXBContext pContext) {
        this.context = pContext;
    }

    /* access modifiers changed from: protected */
    public String getTagName() {
        return JAXB_TAG;
    }

    /* access modifiers changed from: protected */
    public void serialize(final ContentHandler pHandler, Object pObject) throws SAXException {
        try {
            this.context.createMarshaller().marshal(pObject, new ContentHandler() {
                public void endDocument() throws SAXException {
                }

                public void startDocument() throws SAXException {
                }

                public void characters(char[] pChars, int pOffset, int pLength) throws SAXException {
                    pHandler.characters(pChars, pOffset, pLength);
                }

                public void ignorableWhitespace(char[] pChars, int pOffset, int pLength) throws SAXException {
                    pHandler.ignorableWhitespace(pChars, pOffset, pLength);
                }

                public void endPrefixMapping(String pPrefix) throws SAXException {
                    pHandler.endPrefixMapping(pPrefix);
                }

                public void skippedEntity(String pName) throws SAXException {
                    pHandler.endPrefixMapping(pName);
                }

                public void setDocumentLocator(Locator pLocator) {
                    pHandler.setDocumentLocator(pLocator);
                }

                public void processingInstruction(String pTarget, String pData) throws SAXException {
                    pHandler.processingInstruction(pTarget, pData);
                }

                public void startPrefixMapping(String pPrefix, String pURI) throws SAXException {
                    pHandler.startPrefixMapping(pPrefix, pURI);
                }

                public void endElement(String pURI, String pLocalName, String pQName) throws SAXException {
                    pHandler.endElement(pURI, pLocalName, pQName);
                }

                public void startElement(String pURI, String pLocalName, String pQName, Attributes pAttrs) throws SAXException {
                    pHandler.startElement(pURI, pLocalName, pQName, pAttrs);
                }
            });
        } catch (JAXBException e) {
            Throwable t = e.getLinkedException();
            if (t == null || !(t instanceof SAXException)) {
                throw new SAXException(e);
            }
            throw ((SAXException) t);
        }
    }
}
