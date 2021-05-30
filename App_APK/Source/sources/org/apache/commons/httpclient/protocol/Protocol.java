package org.apache.commons.httpclient.protocol;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import org.apache.commons.httpclient.util.LangUtils;

public class Protocol {
    private static final Map PROTOCOLS = Collections.synchronizedMap(new HashMap());
    private int defaultPort;
    private String scheme;
    private boolean secure;
    private ProtocolSocketFactory socketFactory;

    public static void registerProtocol(String id, Protocol protocol) {
        if (id == null) {
            throw new IllegalArgumentException("id is null");
        } else if (protocol != null) {
            PROTOCOLS.put(id, protocol);
        } else {
            throw new IllegalArgumentException("protocol is null");
        }
    }

    public static void unregisterProtocol(String id) {
        if (id != null) {
            PROTOCOLS.remove(id);
            return;
        }
        throw new IllegalArgumentException("id is null");
    }

    public static Protocol getProtocol(String id) throws IllegalStateException {
        if (id != null) {
            Protocol protocol = (Protocol) PROTOCOLS.get(id);
            if (protocol == null) {
                return lazyRegisterProtocol(id);
            }
            return protocol;
        }
        throw new IllegalArgumentException("id is null");
    }

    private static Protocol lazyRegisterProtocol(String id) throws IllegalStateException {
        if ("http".equals(id)) {
            Protocol http = new Protocol("http", (ProtocolSocketFactory) DefaultProtocolSocketFactory.getSocketFactory(), 80);
            registerProtocol("http", http);
            return http;
        } else if ("https".equals(id)) {
            Protocol https = new Protocol("https", (SecureProtocolSocketFactory) SSLProtocolSocketFactory.getSocketFactory(), 443);
            registerProtocol("https", https);
            return https;
        } else {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("unsupported protocol: '");
            stringBuffer.append(id);
            stringBuffer.append("'");
            throw new IllegalStateException(stringBuffer.toString());
        }
    }

    public Protocol(String scheme2, ProtocolSocketFactory factory, int defaultPort2) {
        if (scheme2 == null) {
            throw new IllegalArgumentException("scheme is null");
        } else if (factory == null) {
            throw new IllegalArgumentException("socketFactory is null");
        } else if (defaultPort2 > 0) {
            this.scheme = scheme2;
            this.socketFactory = factory;
            this.defaultPort = defaultPort2;
            this.secure = factory instanceof SecureProtocolSocketFactory;
        } else {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("port is invalid: ");
            stringBuffer.append(defaultPort2);
            throw new IllegalArgumentException(stringBuffer.toString());
        }
    }

    public Protocol(String scheme2, SecureProtocolSocketFactory factory, int defaultPort2) {
        this(scheme2, (ProtocolSocketFactory) factory, defaultPort2);
    }

    public int getDefaultPort() {
        return this.defaultPort;
    }

    public ProtocolSocketFactory getSocketFactory() {
        return this.socketFactory;
    }

    public String getScheme() {
        return this.scheme;
    }

    public boolean isSecure() {
        return this.secure;
    }

    public int resolvePort(int port) {
        return port <= 0 ? getDefaultPort() : port;
    }

    public String toString() {
        StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append(this.scheme);
        stringBuffer.append(":");
        stringBuffer.append(this.defaultPort);
        return stringBuffer.toString();
    }

    public boolean equals(Object obj) {
        if (!(obj instanceof Protocol)) {
            return false;
        }
        Protocol p = (Protocol) obj;
        if (this.defaultPort != p.getDefaultPort() || !this.scheme.equalsIgnoreCase(p.getScheme()) || this.secure != p.isSecure() || !this.socketFactory.equals(p.getSocketFactory())) {
            return false;
        }
        return true;
    }

    public int hashCode() {
        return LangUtils.hashCode(LangUtils.hashCode(LangUtils.hashCode(LangUtils.hashCode(17, this.defaultPort), (Object) this.scheme.toLowerCase()), this.secure), (Object) this.socketFactory);
    }
}
