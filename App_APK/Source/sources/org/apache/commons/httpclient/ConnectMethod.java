package org.apache.commons.httpclient;

import java.io.IOException;
import org.apache.commons.httpclient.cookie.CookieSpec;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

public class ConnectMethod extends HttpMethodBase {
    private static final Log LOG;
    public static final String NAME = "CONNECT";
    static /* synthetic */ Class class$org$apache$commons$httpclient$ConnectMethod;
    private final HostConfiguration targethost;

    public ConnectMethod() {
        this.targethost = null;
    }

    public ConnectMethod(HttpMethod method) {
        this.targethost = null;
    }

    public ConnectMethod(HostConfiguration targethost2) {
        if (targethost2 != null) {
            this.targethost = targethost2;
            return;
        }
        throw new IllegalArgumentException("Target host may not be null");
    }

    public String getName() {
        return NAME;
    }

    public String getPath() {
        if (this.targethost == null) {
            return CookieSpec.PATH_DELIM;
        }
        StringBuffer buffer = new StringBuffer();
        buffer.append(this.targethost.getHost());
        int port = this.targethost.getPort();
        if (port == -1) {
            port = this.targethost.getProtocol().getDefaultPort();
        }
        buffer.append(':');
        buffer.append(port);
        return buffer.toString();
    }

    public URI getURI() throws URIException {
        return new URI(getPath(), true, getParams().getUriCharset());
    }

    /* access modifiers changed from: protected */
    public void addCookieRequestHeader(HttpState state, HttpConnection conn) throws IOException, HttpException {
    }

    /* access modifiers changed from: protected */
    public void addRequestHeaders(HttpState state, HttpConnection conn) throws IOException, HttpException {
        LOG.trace("enter ConnectMethod.addRequestHeaders(HttpState, HttpConnection)");
        addUserAgentRequestHeader(state, conn);
        addHostRequestHeader(state, conn);
        addProxyConnectionHeader(state, conn);
    }

    public int execute(HttpState state, HttpConnection conn) throws IOException, HttpException {
        LOG.trace("enter ConnectMethod.execute(HttpState, HttpConnection)");
        int code = super.execute(state, conn);
        if (LOG.isDebugEnabled()) {
            Log log = LOG;
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("CONNECT status code ");
            stringBuffer.append(code);
            log.debug(stringBuffer.toString());
        }
        return code;
    }

    /* access modifiers changed from: protected */
    public void writeRequestLine(HttpState state, HttpConnection conn) throws IOException, HttpException {
        StringBuffer buffer = new StringBuffer();
        buffer.append(getName());
        buffer.append(' ');
        if (this.targethost != null) {
            buffer.append(getPath());
        } else {
            int port = conn.getPort();
            if (port == -1) {
                port = conn.getProtocol().getDefaultPort();
            }
            buffer.append(conn.getHost());
            buffer.append(':');
            buffer.append(port);
        }
        buffer.append(" ");
        buffer.append(getEffectiveVersion());
        String line = buffer.toString();
        conn.printLine(line, getParams().getHttpElementCharset());
        if (Wire.HEADER_WIRE.enabled()) {
            Wire.HEADER_WIRE.output(line);
        }
    }

    /* access modifiers changed from: protected */
    public boolean shouldCloseConnection(HttpConnection conn) {
        if (getStatusCode() != 200) {
            return super.shouldCloseConnection(conn);
        }
        Header connectionHeader = null;
        if (!conn.isTransparent()) {
            connectionHeader = getResponseHeader("proxy-connection");
        }
        if (connectionHeader == null) {
            connectionHeader = getResponseHeader("connection");
        }
        if (connectionHeader == null || !connectionHeader.getValue().equalsIgnoreCase("close") || !LOG.isWarnEnabled()) {
            return false;
        }
        Log log = LOG;
        StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append("Invalid header encountered '");
        stringBuffer.append(connectionHeader.toExternalForm());
        stringBuffer.append("' in response ");
        stringBuffer.append(getStatusLine().toString());
        log.warn(stringBuffer.toString());
        return false;
    }

    static {
        Class cls;
        if (class$org$apache$commons$httpclient$ConnectMethod == null) {
            cls = class$("org.apache.commons.httpclient.ConnectMethod");
            class$org$apache$commons$httpclient$ConnectMethod = cls;
        } else {
            cls = class$org$apache$commons$httpclient$ConnectMethod;
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
}
