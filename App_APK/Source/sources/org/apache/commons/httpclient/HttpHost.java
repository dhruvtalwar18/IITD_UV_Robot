package org.apache.commons.httpclient;

import org.apache.commons.httpclient.protocol.Protocol;
import org.apache.commons.httpclient.util.LangUtils;

public class HttpHost implements Cloneable {
    private String hostname;
    private int port;
    private Protocol protocol;

    public HttpHost(String hostname2, int port2, Protocol protocol2) {
        this.hostname = null;
        this.port = -1;
        this.protocol = null;
        if (hostname2 == null) {
            throw new IllegalArgumentException("Host name may not be null");
        } else if (protocol2 != null) {
            this.hostname = hostname2;
            this.protocol = protocol2;
            if (port2 >= 0) {
                this.port = port2;
            } else {
                this.port = this.protocol.getDefaultPort();
            }
        } else {
            throw new IllegalArgumentException("Protocol may not be null");
        }
    }

    public HttpHost(String hostname2, int port2) {
        this(hostname2, port2, Protocol.getProtocol("http"));
    }

    public HttpHost(String hostname2) {
        this(hostname2, -1, Protocol.getProtocol("http"));
    }

    public HttpHost(URI uri) throws URIException {
        this(uri.getHost(), uri.getPort(), Protocol.getProtocol(uri.getScheme()));
    }

    public HttpHost(HttpHost httphost) {
        this.hostname = null;
        this.port = -1;
        this.protocol = null;
        init(httphost);
    }

    private void init(HttpHost httphost) {
        this.hostname = httphost.hostname;
        this.port = httphost.port;
        this.protocol = httphost.protocol;
    }

    public Object clone() throws CloneNotSupportedException {
        HttpHost copy = (HttpHost) super.clone();
        copy.init(this);
        return copy;
    }

    public String getHostName() {
        return this.hostname;
    }

    public int getPort() {
        return this.port;
    }

    public Protocol getProtocol() {
        return this.protocol;
    }

    public String toURI() {
        StringBuffer buffer = new StringBuffer(50);
        buffer.append(this.protocol.getScheme());
        buffer.append("://");
        buffer.append(this.hostname);
        if (this.port != this.protocol.getDefaultPort()) {
            buffer.append(':');
            buffer.append(this.port);
        }
        return buffer.toString();
    }

    public String toString() {
        StringBuffer buffer = new StringBuffer(50);
        buffer.append(toURI());
        return buffer.toString();
    }

    public boolean equals(Object o) {
        if (!(o instanceof HttpHost)) {
            return false;
        }
        if (o == this) {
            return true;
        }
        HttpHost that = (HttpHost) o;
        if (this.hostname.equalsIgnoreCase(that.hostname) && this.port == that.port && this.protocol.equals(that.protocol)) {
            return true;
        }
        return false;
    }

    public int hashCode() {
        return LangUtils.hashCode(LangUtils.hashCode(LangUtils.hashCode(17, (Object) this.hostname), this.port), (Object) this.protocol);
    }
}
