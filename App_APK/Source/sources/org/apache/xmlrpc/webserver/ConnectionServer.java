package org.apache.xmlrpc.webserver;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import org.apache.xmlrpc.XmlRpcException;
import org.apache.xmlrpc.common.ServerStreamConnection;
import org.apache.xmlrpc.common.XmlRpcStreamRequestConfig;
import org.apache.xmlrpc.server.XmlRpcHttpServer;

class ConnectionServer extends XmlRpcHttpServer {
    ConnectionServer() {
    }

    /* access modifiers changed from: protected */
    public void writeError(XmlRpcStreamRequestConfig pConfig, OutputStream pStream, Throwable pError) throws XmlRpcException {
        RequestData data = (RequestData) pConfig;
        try {
            if (data.isByteArrayRequired()) {
                super.writeError(pConfig, pStream, pError);
                data.getConnection().writeError(data, pError, (ByteArrayOutputStream) pStream);
                return;
            }
            data.getConnection().writeErrorHeader(data, pError, -1);
            super.writeError(pConfig, pStream, pError);
            pStream.flush();
        } catch (IOException e) {
            throw new XmlRpcException(e.getMessage(), (Throwable) e);
        }
    }

    /* access modifiers changed from: protected */
    public void writeResponse(XmlRpcStreamRequestConfig pConfig, OutputStream pStream, Object pResult) throws XmlRpcException {
        RequestData data = (RequestData) pConfig;
        try {
            if (data.isByteArrayRequired()) {
                super.writeResponse(pConfig, pStream, pResult);
                data.getConnection().writeResponse(data, pStream);
                return;
            }
            data.getConnection().writeResponseHeader(data, -1);
            super.writeResponse(pConfig, pStream, pResult);
            pStream.flush();
        } catch (IOException e) {
            throw new XmlRpcException(e.getMessage(), (Throwable) e);
        }
    }

    /* access modifiers changed from: protected */
    public void setResponseHeader(ServerStreamConnection pConnection, String pHeader, String pValue) {
        ((Connection) pConnection).setResponseHeader(pHeader, pValue);
    }
}
