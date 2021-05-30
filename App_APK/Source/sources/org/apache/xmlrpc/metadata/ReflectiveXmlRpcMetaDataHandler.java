package org.apache.xmlrpc.metadata;

import java.lang.reflect.Method;
import org.apache.xmlrpc.XmlRpcException;
import org.apache.xmlrpc.common.TypeConverterFactory;
import org.apache.xmlrpc.server.AbstractReflectiveHandlerMapping;
import org.apache.xmlrpc.server.ReflectiveXmlRpcHandler;
import org.apache.xmlrpc.server.RequestProcessorFactoryFactory;

public class ReflectiveXmlRpcMetaDataHandler extends ReflectiveXmlRpcHandler implements XmlRpcMetaDataHandler {
    private final String methodHelp;
    private final String[][] signatures;

    public ReflectiveXmlRpcMetaDataHandler(AbstractReflectiveHandlerMapping pMapping, TypeConverterFactory pTypeConverterFactory, Class pClass, RequestProcessorFactoryFactory.RequestProcessorFactory pFactory, Method[] pMethods, String[][] pSignatures, String pMethodHelp) {
        super(pMapping, pTypeConverterFactory, pClass, pFactory, pMethods);
        this.signatures = pSignatures;
        this.methodHelp = pMethodHelp;
    }

    public String[][] getSignatures() throws XmlRpcException {
        return this.signatures;
    }

    public String getMethodHelp() throws XmlRpcException {
        return this.methodHelp;
    }
}
