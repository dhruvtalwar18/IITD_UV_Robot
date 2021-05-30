package org.apache.xmlrpc.common;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

public class LocalStreamConnection {
    private final XmlRpcStreamRequestConfig config;
    /* access modifiers changed from: private */
    public final InputStream request;
    /* access modifiers changed from: private */
    public final ByteArrayOutputStream response = new ByteArrayOutputStream();
    private final ServerStreamConnection serverStreamConnection;

    private class LocalServerStreamConnection implements ServerStreamConnection {
        private LocalServerStreamConnection() {
        }

        public InputStream newInputStream() throws IOException {
            return LocalStreamConnection.this.request;
        }

        public OutputStream newOutputStream() throws IOException {
            return LocalStreamConnection.this.response;
        }

        public void close() throws IOException {
            if (LocalStreamConnection.this.response != null) {
                LocalStreamConnection.this.response.close();
            }
        }
    }

    public LocalStreamConnection(XmlRpcStreamRequestConfig pConfig, InputStream pRequest) {
        this.config = pConfig;
        this.request = pRequest;
        this.serverStreamConnection = new LocalServerStreamConnection();
    }

    public InputStream getRequest() {
        return this.request;
    }

    public XmlRpcStreamRequestConfig getConfig() {
        return this.config;
    }

    public ByteArrayOutputStream getResponse() {
        return this.response;
    }

    public ServerStreamConnection getServerStreamConnection() {
        return this.serverStreamConnection;
    }
}
