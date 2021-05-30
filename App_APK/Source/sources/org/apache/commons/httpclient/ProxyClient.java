package org.apache.commons.httpclient;

import java.io.IOException;
import java.net.Socket;
import org.apache.commons.httpclient.params.HttpClientParams;
import org.apache.commons.httpclient.params.HttpConnectionManagerParams;
import org.apache.commons.httpclient.params.HttpParams;

public class ProxyClient {
    private HostConfiguration hostConfiguration;
    private HttpClientParams params;
    private HttpState state;

    public ProxyClient() {
        this(new HttpClientParams());
    }

    public ProxyClient(HttpClientParams params2) {
        this.state = new HttpState();
        this.params = null;
        this.hostConfiguration = new HostConfiguration();
        if (params2 != null) {
            this.params = params2;
            return;
        }
        throw new IllegalArgumentException("Params may not be null");
    }

    public synchronized HttpState getState() {
        return this.state;
    }

    public synchronized void setState(HttpState state2) {
        this.state = state2;
    }

    public synchronized HostConfiguration getHostConfiguration() {
        return this.hostConfiguration;
    }

    public synchronized void setHostConfiguration(HostConfiguration hostConfiguration2) {
        this.hostConfiguration = hostConfiguration2;
    }

    public synchronized HttpClientParams getParams() {
        return this.params;
    }

    public synchronized void setParams(HttpClientParams params2) {
        if (params2 != null) {
            this.params = params2;
        } else {
            throw new IllegalArgumentException("Parameters may not be null");
        }
    }

    public ConnectResponse connect() throws IOException, HttpException {
        HostConfiguration hostconf = getHostConfiguration();
        if (hostconf.getProxyHost() == null) {
            throw new IllegalStateException("proxy host must be configured");
        } else if (hostconf.getHost() == null) {
            throw new IllegalStateException("destination host must be configured");
        } else if (!hostconf.getProtocol().isSecure()) {
            ConnectMethod method = new ConnectMethod(getHostConfiguration());
            method.getParams().setDefaults(getParams());
            DummyConnectionManager connectionManager = new DummyConnectionManager();
            connectionManager.setConnectionParams(getParams());
            new HttpMethodDirector(connectionManager, hostconf, getParams(), getState()).executeMethod(method);
            ConnectResponse response = new ConnectResponse();
            response.setConnectMethod(method);
            if (method.getStatusCode() == 200) {
                response.setSocket(connectionManager.getConnection().getSocket());
            } else {
                connectionManager.getConnection().close();
            }
            return response;
        } else {
            throw new IllegalStateException("secure protocol socket factory may not be used");
        }
    }

    public static class ConnectResponse {
        private ConnectMethod connectMethod;
        private Socket socket;

        private ConnectResponse() {
        }

        public ConnectMethod getConnectMethod() {
            return this.connectMethod;
        }

        /* access modifiers changed from: private */
        public void setConnectMethod(ConnectMethod connectMethod2) {
            this.connectMethod = connectMethod2;
        }

        public Socket getSocket() {
            return this.socket;
        }

        /* access modifiers changed from: private */
        public void setSocket(Socket socket2) {
            this.socket = socket2;
        }
    }

    static class DummyConnectionManager implements HttpConnectionManager {
        private HttpParams connectionParams;
        private HttpConnection httpConnection;

        DummyConnectionManager() {
        }

        public void closeIdleConnections(long idleTimeout) {
        }

        public HttpConnection getConnection() {
            return this.httpConnection;
        }

        public void setConnectionParams(HttpParams httpParams) {
            this.connectionParams = httpParams;
        }

        public HttpConnection getConnectionWithTimeout(HostConfiguration hostConfiguration, long timeout) {
            this.httpConnection = new HttpConnection(hostConfiguration);
            this.httpConnection.setHttpConnectionManager(this);
            this.httpConnection.getParams().setDefaults(this.connectionParams);
            return this.httpConnection;
        }

        public HttpConnection getConnection(HostConfiguration hostConfiguration, long timeout) throws HttpException {
            return getConnectionWithTimeout(hostConfiguration, timeout);
        }

        public HttpConnection getConnection(HostConfiguration hostConfiguration) {
            return getConnectionWithTimeout(hostConfiguration, -1);
        }

        public void releaseConnection(HttpConnection conn) {
        }

        public HttpConnectionManagerParams getParams() {
            return null;
        }

        public void setParams(HttpConnectionManagerParams params) {
        }
    }
}
