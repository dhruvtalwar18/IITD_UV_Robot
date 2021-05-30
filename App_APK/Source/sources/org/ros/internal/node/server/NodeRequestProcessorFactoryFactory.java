package org.ros.internal.node.server;

import org.apache.xmlrpc.XmlRpcRequest;
import org.apache.xmlrpc.server.RequestProcessorFactoryFactory;
import org.ros.internal.node.xmlrpc.XmlRpcEndpoint;

class NodeRequestProcessorFactoryFactory<T extends XmlRpcEndpoint> implements RequestProcessorFactoryFactory {
    private final RequestProcessorFactoryFactory.RequestProcessorFactory factory = new NodeRequestProcessorFactory();
    /* access modifiers changed from: private */
    public final T node;

    public NodeRequestProcessorFactoryFactory(T instance) {
        this.node = instance;
    }

    public RequestProcessorFactoryFactory.RequestProcessorFactory getRequestProcessorFactory(Class unused) {
        return this.factory;
    }

    private class NodeRequestProcessorFactory implements RequestProcessorFactoryFactory.RequestProcessorFactory {
        private NodeRequestProcessorFactory() {
        }

        public Object getRequestProcessor(XmlRpcRequest xmlRpcRequest) {
            return NodeRequestProcessorFactoryFactory.this.node;
        }
    }
}
