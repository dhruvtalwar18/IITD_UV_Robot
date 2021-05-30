package org.apache.xmlrpc.server;

import java.io.IOException;
import java.io.OutputStream;
import org.apache.xmlrpc.common.ServerStreamConnection;
import org.apache.xmlrpc.common.XmlRpcStreamRequestConfig;

public abstract class XmlRpcHttpServer extends XmlRpcStreamServer {
    /* access modifiers changed from: protected */
    public abstract void setResponseHeader(ServerStreamConnection serverStreamConnection, String str, String str2);

    /* access modifiers changed from: protected */
    public OutputStream getOutputStream(ServerStreamConnection pConnection, XmlRpcStreamRequestConfig pConfig, OutputStream pStream) throws IOException {
        if (pConfig.isEnabledForExtensions() && pConfig.isGzipRequesting()) {
            setResponseHeader(pConnection, "Content-Encoding", "gzip");
        }
        return super.getOutputStream(pConnection, pConfig, pStream);
    }
}
