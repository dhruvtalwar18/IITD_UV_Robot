package org.apache.commons.httpclient;

import java.io.IOException;
import java.io.InputStream;
import org.apache.commons.httpclient.params.HttpConnectionManagerParams;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

public class SimpleHttpConnectionManager implements HttpConnectionManager {
    private static final Log LOG;
    private static final String MISUSE_MESSAGE = "SimpleHttpConnectionManager being used incorrectly.  Be sure that HttpMethod.releaseConnection() is always called and that only one thread and/or method is using this connection manager at a time.";
    static /* synthetic */ Class class$org$apache$commons$httpclient$SimpleHttpConnectionManager;
    private boolean alwaysClose = false;
    protected HttpConnection httpConnection;
    private long idleStartTime = Long.MAX_VALUE;
    private volatile boolean inUse = false;
    private HttpConnectionManagerParams params = new HttpConnectionManagerParams();

    static {
        Class cls;
        if (class$org$apache$commons$httpclient$SimpleHttpConnectionManager == null) {
            cls = class$("org.apache.commons.httpclient.SimpleHttpConnectionManager");
            class$org$apache$commons$httpclient$SimpleHttpConnectionManager = cls;
        } else {
            cls = class$org$apache$commons$httpclient$SimpleHttpConnectionManager;
        }
        LOG = LogFactory.getLog(cls);
    }

    static /* synthetic */ Class class$(String x0) {
        try {
            return Class.forName(x0);
        } catch (ClassNotFoundException x1) {
            throw new NoClassDefFoundError(x1.getMessage());
        }
    }

    static void finishLastResponse(HttpConnection conn) {
        InputStream lastResponse = conn.getLastResponseInputStream();
        if (lastResponse != null) {
            conn.setLastResponseInputStream((InputStream) null);
            try {
                lastResponse.close();
            } catch (IOException e) {
                conn.close();
            }
        }
    }

    public SimpleHttpConnectionManager(boolean alwaysClose2) {
        this.alwaysClose = alwaysClose2;
    }

    public SimpleHttpConnectionManager() {
    }

    public HttpConnection getConnection(HostConfiguration hostConfiguration) {
        return getConnection(hostConfiguration, 0);
    }

    public boolean isConnectionStaleCheckingEnabled() {
        return this.params.isStaleCheckingEnabled();
    }

    public void setConnectionStaleCheckingEnabled(boolean connectionStaleCheckingEnabled) {
        this.params.setStaleCheckingEnabled(connectionStaleCheckingEnabled);
    }

    public HttpConnection getConnectionWithTimeout(HostConfiguration hostConfiguration, long timeout) {
        if (this.httpConnection == null) {
            this.httpConnection = new HttpConnection(hostConfiguration);
            this.httpConnection.setHttpConnectionManager(this);
            this.httpConnection.getParams().setDefaults(this.params);
        } else if (!hostConfiguration.hostEquals(this.httpConnection) || !hostConfiguration.proxyEquals(this.httpConnection)) {
            if (this.httpConnection.isOpen()) {
                this.httpConnection.close();
            }
            this.httpConnection.setHost(hostConfiguration.getHost());
            this.httpConnection.setPort(hostConfiguration.getPort());
            this.httpConnection.setProtocol(hostConfiguration.getProtocol());
            this.httpConnection.setLocalAddress(hostConfiguration.getLocalAddress());
            this.httpConnection.setProxyHost(hostConfiguration.getProxyHost());
            this.httpConnection.setProxyPort(hostConfiguration.getProxyPort());
        } else {
            finishLastResponse(this.httpConnection);
        }
        this.idleStartTime = Long.MAX_VALUE;
        if (this.inUse) {
            LOG.warn(MISUSE_MESSAGE);
        }
        this.inUse = true;
        return this.httpConnection;
    }

    public HttpConnection getConnection(HostConfiguration hostConfiguration, long timeout) {
        return getConnectionWithTimeout(hostConfiguration, timeout);
    }

    public void releaseConnection(HttpConnection conn) {
        if (conn == this.httpConnection) {
            if (this.alwaysClose) {
                this.httpConnection.close();
            } else {
                finishLastResponse(this.httpConnection);
            }
            this.inUse = false;
            this.idleStartTime = System.currentTimeMillis();
            return;
        }
        throw new IllegalStateException("Unexpected release of an unknown connection.");
    }

    public HttpConnectionManagerParams getParams() {
        return this.params;
    }

    public void setParams(HttpConnectionManagerParams params2) {
        if (params2 != null) {
            this.params = params2;
            return;
        }
        throw new IllegalArgumentException("Parameters may not be null");
    }

    public void closeIdleConnections(long idleTimeout) {
        if (this.idleStartTime <= System.currentTimeMillis() - idleTimeout) {
            this.httpConnection.close();
        }
    }

    public void shutdown() {
        this.httpConnection.close();
    }
}