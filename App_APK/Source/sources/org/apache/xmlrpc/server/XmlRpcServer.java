package org.apache.xmlrpc.server;

import org.apache.xmlrpc.XmlRpcConfig;
import org.apache.xmlrpc.XmlRpcException;
import org.apache.xmlrpc.XmlRpcRequest;
import org.apache.xmlrpc.common.TypeConverterFactory;
import org.apache.xmlrpc.common.TypeConverterFactoryImpl;
import org.apache.xmlrpc.common.XmlRpcController;
import org.apache.xmlrpc.common.XmlRpcRequestProcessor;
import org.apache.xmlrpc.common.XmlRpcWorker;
import org.apache.xmlrpc.common.XmlRpcWorkerFactory;

public class XmlRpcServer extends XmlRpcController implements XmlRpcRequestProcessor {
    private XmlRpcServerConfig config = new XmlRpcServerConfigImpl();
    private XmlRpcHandlerMapping handlerMapping;
    private TypeConverterFactory typeConverterFactory = new TypeConverterFactoryImpl();

    /* access modifiers changed from: protected */
    public XmlRpcWorkerFactory getDefaultXmlRpcWorkerFactory() {
        return new XmlRpcServerWorkerFactory(this);
    }

    public void setTypeConverterFactory(TypeConverterFactory pFactory) {
        this.typeConverterFactory = pFactory;
    }

    public TypeConverterFactory getTypeConverterFactory() {
        return this.typeConverterFactory;
    }

    public void setConfig(XmlRpcServerConfig pConfig) {
        this.config = pConfig;
    }

    public XmlRpcConfig getConfig() {
        return this.config;
    }

    public void setHandlerMapping(XmlRpcHandlerMapping pMapping) {
        this.handlerMapping = pMapping;
    }

    public XmlRpcHandlerMapping getHandlerMapping() {
        return this.handlerMapping;
    }

    public Object execute(XmlRpcRequest pRequest) throws XmlRpcException {
        XmlRpcWorkerFactory factory = getWorkerFactory();
        XmlRpcWorker worker = factory.getWorker();
        try {
            return worker.execute(pRequest);
        } finally {
            factory.releaseWorker(worker);
        }
    }
}
