package org.apache.xmlrpc.client;

import java.util.List;
import org.apache.xmlrpc.XmlRpcRequest;
import org.apache.xmlrpc.XmlRpcRequestConfig;

public class XmlRpcClientRequestImpl implements XmlRpcRequest {
    private static final Object[] ZERO_PARAMS = new Object[0];
    private final XmlRpcRequestConfig config;
    private final String methodName;
    private final Object[] params;

    public XmlRpcClientRequestImpl(XmlRpcRequestConfig pConfig, String pMethodName, Object[] pParams) {
        this.config = pConfig;
        if (this.config != null) {
            this.methodName = pMethodName;
            if (this.methodName != null) {
                this.params = pParams == null ? ZERO_PARAMS : pParams;
                return;
            }
            throw new NullPointerException("The method name must not be null.");
        }
        throw new NullPointerException("The request configuration must not be null.");
    }

    /* JADX INFO: this call moved to the top of the method (can break code semantics) */
    public XmlRpcClientRequestImpl(XmlRpcRequestConfig pConfig, String pMethodName, List pParams) {
        this(pConfig, pMethodName, pParams == null ? null : pParams.toArray());
    }

    public String getMethodName() {
        return this.methodName;
    }

    public int getParameterCount() {
        return this.params.length;
    }

    public Object getParameter(int pIndex) {
        return this.params[pIndex];
    }

    public XmlRpcRequestConfig getConfig() {
        return this.config;
    }
}
