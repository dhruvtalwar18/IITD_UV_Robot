package org.apache.xmlrpc.client;

import java.util.List;
import org.apache.xmlrpc.XmlRpcConfig;
import org.apache.xmlrpc.XmlRpcException;
import org.apache.xmlrpc.XmlRpcRequest;
import org.apache.xmlrpc.XmlRpcRequestConfig;
import org.apache.xmlrpc.common.XmlRpcController;
import org.apache.xmlrpc.common.XmlRpcWorkerFactory;
import org.apache.xmlrpc.serializer.XmlWriterFactory;

public class XmlRpcClient extends XmlRpcController {
    private XmlRpcClientConfig config = XmlRpcClientDefaults.newXmlRpcClientConfig();
    private XmlRpcTransportFactory transportFactory = XmlRpcClientDefaults.newTransportFactory(this);
    private XmlWriterFactory xmlWriterFactory = XmlRpcClientDefaults.newXmlWriterFactory();

    /* access modifiers changed from: protected */
    public XmlRpcWorkerFactory getDefaultXmlRpcWorkerFactory() {
        return new XmlRpcClientWorkerFactory(this);
    }

    public void setConfig(XmlRpcClientConfig pConfig) {
        this.config = pConfig;
    }

    public XmlRpcConfig getConfig() {
        return this.config;
    }

    public XmlRpcClientConfig getClientConfig() {
        return this.config;
    }

    public void setTransportFactory(XmlRpcTransportFactory pFactory) {
        this.transportFactory = pFactory;
    }

    public XmlRpcTransportFactory getTransportFactory() {
        return this.transportFactory;
    }

    public Object execute(String pMethodName, Object[] pParams) throws XmlRpcException {
        return execute(getClientConfig(), pMethodName, pParams);
    }

    public Object execute(XmlRpcClientConfig pConfig, String pMethodName, Object[] pParams) throws XmlRpcException {
        return execute(new XmlRpcClientRequestImpl((XmlRpcRequestConfig) pConfig, pMethodName, pParams));
    }

    public Object execute(String pMethodName, List pParams) throws XmlRpcException {
        return execute(getClientConfig(), pMethodName, pParams);
    }

    public Object execute(XmlRpcClientConfig pConfig, String pMethodName, List pParams) throws XmlRpcException {
        return execute(new XmlRpcClientRequestImpl((XmlRpcRequestConfig) pConfig, pMethodName, pParams));
    }

    public Object execute(XmlRpcRequest pRequest) throws XmlRpcException {
        return getWorkerFactory().getWorker().execute(pRequest);
    }

    public void executeAsync(String pMethodName, Object[] pParams, AsyncCallback pCallback) throws XmlRpcException {
        executeAsync(getClientConfig(), pMethodName, pParams, pCallback);
    }

    public void executeAsync(XmlRpcClientConfig pConfig, String pMethodName, Object[] pParams, AsyncCallback pCallback) throws XmlRpcException {
        executeAsync(new XmlRpcClientRequestImpl((XmlRpcRequestConfig) pConfig, pMethodName, pParams), pCallback);
    }

    public void executeAsync(String pMethodName, List pParams, AsyncCallback pCallback) throws XmlRpcException {
        executeAsync(getClientConfig(), pMethodName, pParams, pCallback);
    }

    public void executeAsync(XmlRpcClientConfig pConfig, String pMethodName, List pParams, AsyncCallback pCallback) throws XmlRpcException {
        executeAsync(new XmlRpcClientRequestImpl((XmlRpcRequestConfig) pConfig, pMethodName, pParams), pCallback);
    }

    public void executeAsync(XmlRpcRequest pRequest, AsyncCallback pCallback) throws XmlRpcException {
        ((XmlRpcClientWorker) getWorkerFactory().getWorker()).execute(pRequest, pCallback);
    }

    public XmlWriterFactory getXmlWriterFactory() {
        return this.xmlWriterFactory;
    }

    public void setXmlWriterFactory(XmlWriterFactory pFactory) {
        this.xmlWriterFactory = pFactory;
    }
}
