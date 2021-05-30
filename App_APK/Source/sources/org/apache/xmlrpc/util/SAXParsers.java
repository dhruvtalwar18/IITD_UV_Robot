package org.apache.xmlrpc.util;

import javax.xml.parsers.ParserConfigurationException;
import javax.xml.parsers.SAXParserFactory;
import org.apache.xmlrpc.XmlRpcException;
import org.xml.sax.SAXException;
import org.xml.sax.XMLReader;

public class SAXParsers {
    private static SAXParserFactory spf = SAXParserFactory.newInstance();

    static {
        spf.setNamespaceAware(true);
        spf.setValidating(false);
    }

    public static XMLReader newXMLReader() throws XmlRpcException {
        try {
            return spf.newSAXParser().getXMLReader();
        } catch (ParserConfigurationException e) {
            throw new XmlRpcException("Unable to create XML parser: " + e.getMessage(), (Throwable) e);
        } catch (SAXException e2) {
            throw new XmlRpcException("Unable to create XML parser: " + e2.getMessage(), (Throwable) e2);
        }
    }

    public static SAXParserFactory getSAXParserFactory() {
        return spf;
    }

    public static void setSAXParserFactory(SAXParserFactory pFactory) {
        spf = pFactory;
    }
}
