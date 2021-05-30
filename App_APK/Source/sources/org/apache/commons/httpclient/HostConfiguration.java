package org.apache.commons.httpclient;

import java.net.InetAddress;
import org.apache.commons.httpclient.params.HostParams;
import org.apache.commons.httpclient.protocol.Protocol;
import org.apache.commons.httpclient.util.LangUtils;

public class HostConfiguration implements Cloneable {
    public static final HostConfiguration ANY_HOST_CONFIGURATION = new HostConfiguration();
    private HttpHost host = null;
    private InetAddress localAddress = null;
    private HostParams params = new HostParams();
    private ProxyHost proxyHost = null;

    public HostConfiguration() {
    }

    public HostConfiguration(HostConfiguration hostConfiguration) {
        init(hostConfiguration);
    }

    private void init(HostConfiguration hostConfiguration) {
        synchronized (hostConfiguration) {
            try {
                if (hostConfiguration.host != null) {
                    this.host = (HttpHost) hostConfiguration.host.clone();
                } else {
                    this.host = null;
                }
                if (hostConfiguration.proxyHost != null) {
                    this.proxyHost = (ProxyHost) hostConfiguration.proxyHost.clone();
                } else {
                    this.proxyHost = null;
                }
                this.localAddress = hostConfiguration.getLocalAddress();
                this.params = (HostParams) hostConfiguration.getParams().clone();
            } catch (CloneNotSupportedException e) {
                throw new IllegalArgumentException("Host configuration could not be cloned");
            } catch (Throwable th) {
                throw th;
            }
        }
    }

    public Object clone() {
        try {
            HostConfiguration copy = (HostConfiguration) super.clone();
            copy.init(this);
            return copy;
        } catch (CloneNotSupportedException e) {
            throw new IllegalArgumentException("Host configuration could not be cloned");
        }
    }

    public synchronized String toString() {
        StringBuffer b;
        boolean appendComma = false;
        b = new StringBuffer(50);
        b.append("HostConfiguration[");
        if (this.host != null) {
            appendComma = true;
            b.append("host=");
            b.append(this.host);
        }
        if (this.proxyHost != null) {
            if (appendComma) {
                b.append(", ");
            } else {
                appendComma = true;
            }
            b.append("proxyHost=");
            b.append(this.proxyHost);
        }
        if (this.localAddress != null) {
            if (appendComma) {
                b.append(", ");
            } else {
                appendComma = true;
            }
            b.append("localAddress=");
            b.append(this.localAddress);
            if (appendComma) {
                b.append(", ");
            }
            b.append("params=");
            b.append(this.params);
        }
        b.append("]");
        return b.toString();
    }

    public synchronized boolean hostEquals(HttpConnection connection) {
        if (connection == null) {
            throw new IllegalArgumentException("Connection may not be null");
        } else if (this.host == null) {
            return false;
        } else {
            if (!this.host.getHostName().equalsIgnoreCase(connection.getHost())) {
                return false;
            }
            if (this.host.getPort() != connection.getPort()) {
                return false;
            }
            if (!this.host.getProtocol().equals(connection.getProtocol())) {
                return false;
            }
            if (this.localAddress != null) {
                if (!this.localAddress.equals(connection.getLocalAddress())) {
                    return false;
                }
            } else if (connection.getLocalAddress() != null) {
                return false;
            }
            return true;
        }
    }

    /* JADX WARNING: Code restructure failed: missing block: B:11:0x0028, code lost:
        return r1;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:17:0x0032, code lost:
        return r1;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public synchronized boolean proxyEquals(org.apache.commons.httpclient.HttpConnection r5) {
        /*
            r4 = this;
            monitor-enter(r4)
            if (r5 == 0) goto L_0x0033
            org.apache.commons.httpclient.ProxyHost r0 = r4.proxyHost     // Catch:{ all -> 0x003b }
            r1 = 0
            r2 = 1
            if (r0 == 0) goto L_0x0029
            org.apache.commons.httpclient.ProxyHost r0 = r4.proxyHost     // Catch:{ all -> 0x003b }
            java.lang.String r0 = r0.getHostName()     // Catch:{ all -> 0x003b }
            java.lang.String r3 = r5.getProxyHost()     // Catch:{ all -> 0x003b }
            boolean r0 = r0.equalsIgnoreCase(r3)     // Catch:{ all -> 0x003b }
            if (r0 == 0) goto L_0x0027
            org.apache.commons.httpclient.ProxyHost r0 = r4.proxyHost     // Catch:{ all -> 0x003b }
            int r0 = r0.getPort()     // Catch:{ all -> 0x003b }
            int r3 = r5.getProxyPort()     // Catch:{ all -> 0x003b }
            if (r0 != r3) goto L_0x0027
            r1 = 1
        L_0x0027:
            monitor-exit(r4)
            return r1
        L_0x0029:
            java.lang.String r0 = r5.getProxyHost()     // Catch:{ all -> 0x003b }
            if (r0 != 0) goto L_0x0031
            r1 = 1
        L_0x0031:
            monitor-exit(r4)
            return r1
        L_0x0033:
            java.lang.IllegalArgumentException r0 = new java.lang.IllegalArgumentException     // Catch:{ all -> 0x003b }
            java.lang.String r1 = "Connection may not be null"
            r0.<init>(r1)     // Catch:{ all -> 0x003b }
            throw r0     // Catch:{ all -> 0x003b }
        L_0x003b:
            r5 = move-exception
            monitor-exit(r4)
            throw r5
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.httpclient.HostConfiguration.proxyEquals(org.apache.commons.httpclient.HttpConnection):boolean");
    }

    public synchronized boolean isHostSet() {
        return this.host != null;
    }

    public synchronized void setHost(HttpHost host2) {
        this.host = host2;
    }

    public synchronized void setHost(String host2, int port, String protocol) {
        this.host = new HttpHost(host2, port, Protocol.getProtocol(protocol));
    }

    public synchronized void setHost(String host2, String virtualHost, int port, Protocol protocol) {
        setHost(host2, port, protocol);
        this.params.setVirtualHost(virtualHost);
    }

    public synchronized void setHost(String host2, int port, Protocol protocol) {
        if (host2 == null) {
            throw new IllegalArgumentException("host must not be null");
        } else if (protocol != null) {
            this.host = new HttpHost(host2, port, protocol);
        } else {
            throw new IllegalArgumentException("protocol must not be null");
        }
    }

    public synchronized void setHost(String host2, int port) {
        setHost(host2, port, Protocol.getProtocol("http"));
    }

    public synchronized void setHost(String host2) {
        Protocol defaultProtocol = Protocol.getProtocol("http");
        setHost(host2, defaultProtocol.getDefaultPort(), defaultProtocol);
    }

    public synchronized void setHost(URI uri) {
        try {
            setHost(uri.getHost(), uri.getPort(), uri.getScheme());
        } catch (URIException e) {
            throw new IllegalArgumentException(e.toString());
        }
    }

    public synchronized String getHostURL() {
        if (this.host != null) {
        } else {
            throw new IllegalStateException("Host must be set to create a host URL");
        }
        return this.host.toURI();
    }

    public synchronized String getHost() {
        if (this.host == null) {
            return null;
        }
        return this.host.getHostName();
    }

    public synchronized String getVirtualHost() {
        return this.params.getVirtualHost();
    }

    public synchronized int getPort() {
        if (this.host == null) {
            return -1;
        }
        return this.host.getPort();
    }

    public synchronized Protocol getProtocol() {
        if (this.host == null) {
            return null;
        }
        return this.host.getProtocol();
    }

    public synchronized boolean isProxySet() {
        return this.proxyHost != null;
    }

    public synchronized void setProxyHost(ProxyHost proxyHost2) {
        this.proxyHost = proxyHost2;
    }

    public synchronized void setProxy(String proxyHost2, int proxyPort) {
        this.proxyHost = new ProxyHost(proxyHost2, proxyPort);
    }

    public synchronized String getProxyHost() {
        if (this.proxyHost == null) {
            return null;
        }
        return this.proxyHost.getHostName();
    }

    public synchronized int getProxyPort() {
        if (this.proxyHost == null) {
            return -1;
        }
        return this.proxyHost.getPort();
    }

    public synchronized void setLocalAddress(InetAddress localAddress2) {
        this.localAddress = localAddress2;
    }

    public synchronized InetAddress getLocalAddress() {
        return this.localAddress;
    }

    public HostParams getParams() {
        return this.params;
    }

    public void setParams(HostParams params2) {
        if (params2 != null) {
            this.params = params2;
            return;
        }
        throw new IllegalArgumentException("Parameters may not be null");
    }

    /* JADX WARNING: Code restructure failed: missing block: B:19:0x002f, code lost:
        return r0;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public synchronized boolean equals(java.lang.Object r6) {
        /*
            r5 = this;
            monitor-enter(r5)
            boolean r0 = r6 instanceof org.apache.commons.httpclient.HostConfiguration     // Catch:{ all -> 0x0032 }
            r1 = 0
            if (r0 == 0) goto L_0x0030
            r0 = 1
            if (r6 != r5) goto L_0x000b
            monitor-exit(r5)
            return r0
        L_0x000b:
            r2 = r6
            org.apache.commons.httpclient.HostConfiguration r2 = (org.apache.commons.httpclient.HostConfiguration) r2     // Catch:{ all -> 0x0032 }
            org.apache.commons.httpclient.HttpHost r3 = r5.host     // Catch:{ all -> 0x0032 }
            org.apache.commons.httpclient.HttpHost r4 = r2.host     // Catch:{ all -> 0x0032 }
            boolean r3 = org.apache.commons.httpclient.util.LangUtils.equals(r3, r4)     // Catch:{ all -> 0x0032 }
            if (r3 == 0) goto L_0x002d
            org.apache.commons.httpclient.ProxyHost r3 = r5.proxyHost     // Catch:{ all -> 0x0032 }
            org.apache.commons.httpclient.ProxyHost r4 = r2.proxyHost     // Catch:{ all -> 0x0032 }
            boolean r3 = org.apache.commons.httpclient.util.LangUtils.equals(r3, r4)     // Catch:{ all -> 0x0032 }
            if (r3 == 0) goto L_0x002d
            java.net.InetAddress r3 = r5.localAddress     // Catch:{ all -> 0x0032 }
            java.net.InetAddress r4 = r2.localAddress     // Catch:{ all -> 0x0032 }
            boolean r3 = org.apache.commons.httpclient.util.LangUtils.equals(r3, r4)     // Catch:{ all -> 0x0032 }
            if (r3 == 0) goto L_0x002d
            goto L_0x002e
        L_0x002d:
            r0 = 0
        L_0x002e:
            monitor-exit(r5)
            return r0
        L_0x0030:
            monitor-exit(r5)
            return r1
        L_0x0032:
            r6 = move-exception
            monitor-exit(r5)
            throw r6
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.httpclient.HostConfiguration.equals(java.lang.Object):boolean");
    }

    public synchronized int hashCode() {
        return LangUtils.hashCode(LangUtils.hashCode(LangUtils.hashCode(17, (Object) this.host), (Object) this.proxyHost), (Object) this.localAddress);
    }
}
