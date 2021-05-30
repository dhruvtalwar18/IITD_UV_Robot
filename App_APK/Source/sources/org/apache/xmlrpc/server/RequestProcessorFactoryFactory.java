package org.apache.xmlrpc.server;

import org.apache.xmlrpc.XmlRpcException;
import org.apache.xmlrpc.XmlRpcRequest;
import org.apache.xmlrpc.metadata.Util;

public interface RequestProcessorFactoryFactory {

    public interface RequestProcessorFactory {
        Object getRequestProcessor(XmlRpcRequest xmlRpcRequest) throws XmlRpcException;
    }

    RequestProcessorFactory getRequestProcessorFactory(Class cls) throws XmlRpcException;

    public static class RequestSpecificProcessorFactoryFactory implements RequestProcessorFactoryFactory {
        /* access modifiers changed from: protected */
        public Object getRequestProcessor(Class pClass, XmlRpcRequest pRequest) throws XmlRpcException {
            return Util.newInstance(pClass);
        }

        public RequestProcessorFactory getRequestProcessorFactory(final Class pClass) throws XmlRpcException {
            return new RequestProcessorFactory() {
                public Object getRequestProcessor(XmlRpcRequest pRequest) throws XmlRpcException {
                    return RequestSpecificProcessorFactoryFactory.this.getRequestProcessor(pClass, pRequest);
                }
            };
        }
    }

    public static class StatelessProcessorFactoryFactory implements RequestProcessorFactoryFactory {
        /* access modifiers changed from: protected */
        public Object getRequestProcessor(Class pClass) throws XmlRpcException {
            return Util.newInstance(pClass);
        }

        public RequestProcessorFactory getRequestProcessorFactory(Class pClass) throws XmlRpcException {
            final Object processor = getRequestProcessor(pClass);
            return new RequestProcessorFactory() {
                public Object getRequestProcessor(XmlRpcRequest pRequest) throws XmlRpcException {
                    return processor;
                }
            };
        }
    }
}
