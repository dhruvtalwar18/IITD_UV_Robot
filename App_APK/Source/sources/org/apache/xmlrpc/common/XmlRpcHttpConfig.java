package org.apache.xmlrpc.common;

public interface XmlRpcHttpConfig extends XmlRpcStreamConfig {
    String getBasicEncoding();

    boolean isContentLengthOptional();
}
