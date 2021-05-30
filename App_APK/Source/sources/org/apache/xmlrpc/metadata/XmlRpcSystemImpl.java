package org.apache.xmlrpc.metadata;

import org.apache.xmlrpc.XmlRpcException;
import org.apache.xmlrpc.XmlRpcRequest;
import org.apache.xmlrpc.server.PropertyHandlerMapping;
import org.apache.xmlrpc.server.RequestProcessorFactoryFactory;

public class XmlRpcSystemImpl {
    private XmlRpcListableHandlerMapping mapping;

    public XmlRpcSystemImpl(XmlRpcListableHandlerMapping pMapping) {
        this.mapping = pMapping;
    }

    public String[][] methodSignature(String methodName) throws XmlRpcException {
        return this.mapping.getMethodSignature(methodName);
    }

    public String methodHelp(String methodName) throws XmlRpcException {
        return this.mapping.getMethodHelp(methodName);
    }

    public String[] listMethods() throws XmlRpcException {
        return this.mapping.getListMethods();
    }

    public static void addSystemHandler(PropertyHandlerMapping pMapping) throws XmlRpcException {
        final RequestProcessorFactoryFactory factory = pMapping.getRequestProcessorFactoryFactory();
        pMapping.setRequestProcessorFactoryFactory(new RequestProcessorFactoryFactory(new XmlRpcSystemImpl(pMapping)) {
            final /* synthetic */ XmlRpcSystemImpl val$systemHandler;

            {
                this.val$systemHandler = r1;
            }

            public RequestProcessorFactoryFactory.RequestProcessorFactory getRequestProcessorFactory(Class pClass) throws XmlRpcException {
                if (XmlRpcSystemImpl.class.equals(pClass)) {
                    return new RequestProcessorFactoryFactory.RequestProcessorFactory() {
                        public Object getRequestProcessor(XmlRpcRequest request) throws XmlRpcException {
                            return AnonymousClass1.this.val$systemHandler;
                        }
                    };
                }
                return factory.getRequestProcessorFactory(pClass);
            }
        });
        pMapping.addHandler("system", XmlRpcSystemImpl.class);
    }
}
