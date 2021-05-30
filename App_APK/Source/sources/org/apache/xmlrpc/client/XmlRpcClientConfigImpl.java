package org.apache.xmlrpc.client;

import java.io.Serializable;
import java.net.URL;
import org.apache.xmlrpc.common.XmlRpcHttpRequestConfigImpl;
import org.apache.xmlrpc.common.XmlRpcRequestProcessor;

public class XmlRpcClientConfigImpl extends XmlRpcHttpRequestConfigImpl implements XmlRpcHttpClientConfig, XmlRpcLocalClientConfig, Cloneable, Serializable {
    private static final long serialVersionUID = 4121131450507800889L;
    private URL serverURL;
    private String userAgent;
    private XmlRpcRequestProcessor xmlRpcServer;

    public XmlRpcClientConfigImpl cloneMe() {
        try {
            return (XmlRpcClientConfigImpl) clone();
        } catch (CloneNotSupportedException e) {
            throw new IllegalStateException("Unable to create my clone");
        }
    }

    public void setServerURL(URL pURL) {
        this.serverURL = pURL;
    }

    public URL getServerURL() {
        return this.serverURL;
    }

    public void setXmlRpcServer(XmlRpcRequestProcessor pServer) {
        this.xmlRpcServer = pServer;
    }

    public XmlRpcRequestProcessor getXmlRpcServer() {
        return this.xmlRpcServer;
    }

    public String getUserAgent() {
        return this.userAgent;
    }

    public void setUserAgent(String pUserAgent) {
        this.userAgent = pUserAgent;
    }
}
