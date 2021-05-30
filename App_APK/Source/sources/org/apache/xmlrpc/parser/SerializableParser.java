package org.apache.xmlrpc.parser;

import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import org.apache.xmlrpc.XmlRpcException;

public class SerializableParser extends ByteArrayParser {
    public Object getResult() throws XmlRpcException {
        try {
            return new ObjectInputStream(new ByteArrayInputStream((byte[]) super.getResult())).readObject();
        } catch (IOException e) {
            throw new XmlRpcException("Failed to read result object: " + e.getMessage(), (Throwable) e);
        } catch (ClassNotFoundException e2) {
            throw new XmlRpcException("Failed to load class for result object: " + e2.getMessage(), (Throwable) e2);
        }
    }
}
