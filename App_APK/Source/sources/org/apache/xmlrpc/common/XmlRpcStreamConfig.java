package org.apache.xmlrpc.common;

import org.apache.xmlrpc.XmlRpcConfig;

public interface XmlRpcStreamConfig extends XmlRpcConfig {
    public static final String UTF8_ENCODING = "UTF-8";

    String getEncoding();
}
