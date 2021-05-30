package org.apache.xmlrpc.client;

import java.net.URL;
import org.apache.xmlrpc.common.XmlRpcHttpRequestConfig;

public interface XmlRpcHttpClientConfig extends XmlRpcHttpRequestConfig {
    URL getServerURL();

    String getUserAgent();
}
