package org.apache.xmlrpc.common;

import org.apache.xmlrpc.XmlRpcConfig;

public abstract class XmlRpcController {
    private int maxThreads;
    private TypeFactory typeFactory = new TypeFactoryImpl(this);
    private XmlRpcWorkerFactory workerFactory = getDefaultXmlRpcWorkerFactory();

    public abstract XmlRpcConfig getConfig();

    /* access modifiers changed from: protected */
    public abstract XmlRpcWorkerFactory getDefaultXmlRpcWorkerFactory();

    public void setMaxThreads(int pMaxThreads) {
        this.maxThreads = pMaxThreads;
    }

    public int getMaxThreads() {
        return this.maxThreads;
    }

    public void setWorkerFactory(XmlRpcWorkerFactory pFactory) {
        this.workerFactory = pFactory;
    }

    public XmlRpcWorkerFactory getWorkerFactory() {
        return this.workerFactory;
    }

    public void setTypeFactory(TypeFactory pTypeFactory) {
        this.typeFactory = pTypeFactory;
    }

    public TypeFactory getTypeFactory() {
        return this.typeFactory;
    }
}
