package org.apache.xmlrpc.server;

import org.apache.xmlrpc.common.ServerStreamConnection;

public interface ServerHttpConnection extends ServerStreamConnection {
    void setContentLength(int i);

    void setResponseHeader(String str, String str2);
}
