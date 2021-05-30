package org.apache.ws.commons.serialize;

import java.util.Arrays;
import java.util.Comparator;
import org.xml.sax.Attributes;
import org.xml.sax.SAXException;
import org.xml.sax.helpers.AttributesImpl;

public class OrderedAttributeXMLWriter extends XMLWriterImpl {
    public void startElement(String pNamespaceURI, String pLocalName, String pQName, final Attributes pAttrs) throws SAXException {
        Integer[] attributeNumbers = new Integer[pAttrs.getLength()];
        for (int i = 0; i < attributeNumbers.length; i++) {
            attributeNumbers[i] = new Integer(i);
        }
        Arrays.sort(attributeNumbers, new Comparator() {
            public int compare(Object pNum1, Object pNum2) {
                int i1 = ((Integer) pNum1).intValue();
                int i2 = ((Integer) pNum2).intValue();
                String uri1 = pAttrs.getURI(i1);
                if (uri1 == null) {
                    uri1 = "";
                }
                String uri2 = pAttrs.getURI(i2);
                if (uri2 == null) {
                    uri2 = "";
                }
                int result = uri1.compareTo(uri2);
                if (result == 0) {
                    return pAttrs.getLocalName(i1).compareTo(pAttrs.getLocalName(i2));
                }
                return result;
            }
        });
        AttributesImpl orderedAttributes = new AttributesImpl();
        for (Integer intValue : attributeNumbers) {
            int num = intValue.intValue();
            orderedAttributes.addAttribute(pAttrs.getURI(num), pAttrs.getLocalName(num), pAttrs.getQName(num), pAttrs.getType(num), pAttrs.getValue(num));
        }
        super.startElement(pNamespaceURI, pLocalName, pQName, orderedAttributes);
    }
}
