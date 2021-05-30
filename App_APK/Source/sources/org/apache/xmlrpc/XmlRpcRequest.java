package org.apache.xmlrpc;

public interface XmlRpcRequest {
    XmlRpcRequestConfig getConfig();

    String getMethodName();

    Object getParameter(int i);

    int getParameterCount();
}
