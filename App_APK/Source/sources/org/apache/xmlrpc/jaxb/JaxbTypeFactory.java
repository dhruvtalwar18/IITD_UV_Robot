package org.apache.xmlrpc.jaxb;

import javax.xml.bind.Element;
import javax.xml.bind.JAXBContext;
import org.apache.ws.commons.util.NamespaceContextImpl;
import org.apache.xmlrpc.common.TypeFactoryImpl;
import org.apache.xmlrpc.common.XmlRpcController;
import org.apache.xmlrpc.common.XmlRpcStreamConfig;
import org.apache.xmlrpc.parser.TypeParser;
import org.apache.xmlrpc.serializer.TypeSerializer;
import org.apache.xmlrpc.serializer.XmlRpcWriter;
import org.xml.sax.SAXException;

public class JaxbTypeFactory extends TypeFactoryImpl {
    private final JAXBContext context;
    private final JaxbSerializer serializer = new JaxbSerializer(this.context);

    public JaxbTypeFactory(XmlRpcController pController, JAXBContext pContext) {
        super(pController);
        this.context = pContext;
    }

    public TypeParser getParser(XmlRpcStreamConfig pConfig, NamespaceContextImpl pContext, String pURI, String pLocalName) {
        TypeParser tp = super.getParser(pConfig, pContext, pURI, pLocalName);
        if (tp != null || !XmlRpcWriter.EXTENSIONS_URI.equals(pURI) || !JaxbSerializer.JAXB_TAG.equals(pLocalName)) {
            return tp;
        }
        return new JaxbParser(this.context);
    }

    public TypeSerializer getSerializer(XmlRpcStreamConfig pConfig, Object pObject) throws SAXException {
        TypeSerializer ts = super.getSerializer(pConfig, pObject);
        if (ts != null || !(pObject instanceof Element)) {
            return ts;
        }
        return this.serializer;
    }
}
