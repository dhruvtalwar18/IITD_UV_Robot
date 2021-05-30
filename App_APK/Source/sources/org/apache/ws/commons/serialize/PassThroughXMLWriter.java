package org.apache.ws.commons.serialize;

public class PassThroughXMLWriter extends XMLWriterImpl {
    public boolean canEncode(char c) {
        return true;
    }
}
