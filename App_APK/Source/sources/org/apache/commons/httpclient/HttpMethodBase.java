package org.apache.commons.httpclient;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InterruptedIOException;
import java.util.Collection;
import org.apache.commons.httpclient.auth.AuthState;
import org.apache.commons.httpclient.cookie.CookiePolicy;
import org.apache.commons.httpclient.cookie.CookieSpec;
import org.apache.commons.httpclient.cookie.CookieVersionSupport;
import org.apache.commons.httpclient.cookie.RFC2109Spec;
import org.apache.commons.httpclient.cookie.RFC2965Spec;
import org.apache.commons.httpclient.params.HttpMethodParams;
import org.apache.commons.httpclient.protocol.Protocol;
import org.apache.commons.httpclient.util.EncodingUtil;
import org.apache.commons.httpclient.util.ExceptionUtil;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.jboss.netty.handler.codec.http.HttpHeaders;
import org.xbill.DNS.TTL;

public abstract class HttpMethodBase implements HttpMethod {
    private static final int DEFAULT_INITIAL_BUFFER_SIZE = 4096;
    private static final Log LOG;
    private static final int RESPONSE_WAIT_TIME_MS = 3000;
    static /* synthetic */ Class class$org$apache$commons$httpclient$HttpMethodBase;
    private volatile boolean aborted = false;
    private boolean connectionCloseForced = false;
    private CookieSpec cookiespec = null;
    private boolean doAuthentication = true;
    protected HttpVersion effectiveVersion = null;
    private boolean followRedirects = false;
    private AuthState hostAuthState = new AuthState();
    private HttpHost httphost = null;
    private MethodRetryHandler methodRetryHandler;
    private HttpMethodParams params = new HttpMethodParams();
    private String path = null;
    private AuthState proxyAuthState = new AuthState();
    private String queryString = null;
    private int recoverableExceptionCount = 0;
    private HeaderGroup requestHeaders = new HeaderGroup();
    private boolean requestSent = false;
    private byte[] responseBody = null;
    private HttpConnection responseConnection = null;
    private HeaderGroup responseHeaders = new HeaderGroup();
    private InputStream responseStream = null;
    private HeaderGroup responseTrailerHeaders = new HeaderGroup();
    protected StatusLine statusLine = null;
    private boolean used = false;

    public abstract String getName();

    static {
        Class cls;
        if (class$org$apache$commons$httpclient$HttpMethodBase == null) {
            cls = class$("org.apache.commons.httpclient.HttpMethodBase");
            class$org$apache$commons$httpclient$HttpMethodBase = cls;
        } else {
            cls = class$org$apache$commons$httpclient$HttpMethodBase;
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

    public HttpMethodBase() {
    }

    public HttpMethodBase(String uri) throws IllegalArgumentException, IllegalStateException {
        if (uri != null) {
            try {
                if (uri.equals("")) {
                }
                setURI(new URI(uri, true, getParams().getUriCharset()));
            } catch (URIException e) {
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append("Invalid uri '");
                stringBuffer.append(uri);
                stringBuffer.append("': ");
                stringBuffer.append(e.getMessage());
                throw new IllegalArgumentException(stringBuffer.toString());
            }
        }
        uri = CookieSpec.PATH_DELIM;
        setURI(new URI(uri, true, getParams().getUriCharset()));
    }

    public URI getURI() throws URIException {
        StringBuffer buffer = new StringBuffer();
        if (this.httphost != null) {
            buffer.append(this.httphost.getProtocol().getScheme());
            buffer.append("://");
            buffer.append(this.httphost.getHostName());
            int port = this.httphost.getPort();
            if (!(port == -1 || port == this.httphost.getProtocol().getDefaultPort())) {
                buffer.append(":");
                buffer.append(port);
            }
        }
        buffer.append(this.path);
        if (this.queryString != null) {
            buffer.append('?');
            buffer.append(this.queryString);
        }
        return new URI(buffer.toString(), true, getParams().getUriCharset());
    }

    public void setURI(URI uri) throws URIException {
        if (uri.isAbsoluteURI()) {
            this.httphost = new HttpHost(uri);
        }
        setPath(uri.getPath() == null ? CookieSpec.PATH_DELIM : uri.getEscapedPath());
        setQueryString(uri.getEscapedQuery());
    }

    public void setFollowRedirects(boolean followRedirects2) {
        this.followRedirects = followRedirects2;
    }

    public boolean getFollowRedirects() {
        return this.followRedirects;
    }

    public void setHttp11(boolean http11) {
        if (http11) {
            this.params.setVersion(HttpVersion.HTTP_1_1);
        } else {
            this.params.setVersion(HttpVersion.HTTP_1_0);
        }
    }

    public boolean getDoAuthentication() {
        return this.doAuthentication;
    }

    public void setDoAuthentication(boolean doAuthentication2) {
        this.doAuthentication = doAuthentication2;
    }

    public boolean isHttp11() {
        return this.params.getVersion().equals(HttpVersion.HTTP_1_1);
    }

    public void setPath(String path2) {
        this.path = path2;
    }

    public void addRequestHeader(Header header) {
        LOG.trace("HttpMethodBase.addRequestHeader(Header)");
        if (header == null) {
            LOG.debug("null header value ignored");
        } else {
            getRequestHeaderGroup().addHeader(header);
        }
    }

    public void addResponseFooter(Header footer) {
        getResponseTrailerHeaderGroup().addHeader(footer);
    }

    public String getPath() {
        return (this.path == null || this.path.equals("")) ? CookieSpec.PATH_DELIM : this.path;
    }

    public void setQueryString(String queryString2) {
        this.queryString = queryString2;
    }

    public void setQueryString(NameValuePair[] params2) {
        LOG.trace("enter HttpMethodBase.setQueryString(NameValuePair[])");
        this.queryString = EncodingUtil.formUrlEncode(params2, "UTF-8");
    }

    public String getQueryString() {
        return this.queryString;
    }

    public void setRequestHeader(String headerName, String headerValue) {
        setRequestHeader(new Header(headerName, headerValue));
    }

    public void setRequestHeader(Header header) {
        Header[] headers = getRequestHeaderGroup().getHeaders(header.getName());
        for (Header removeHeader : headers) {
            getRequestHeaderGroup().removeHeader(removeHeader);
        }
        getRequestHeaderGroup().addHeader(header);
    }

    public Header getRequestHeader(String headerName) {
        if (headerName == null) {
            return null;
        }
        return getRequestHeaderGroup().getCondensedHeader(headerName);
    }

    public Header[] getRequestHeaders() {
        return getRequestHeaderGroup().getAllHeaders();
    }

    public Header[] getRequestHeaders(String headerName) {
        return getRequestHeaderGroup().getHeaders(headerName);
    }

    /* access modifiers changed from: protected */
    public HeaderGroup getRequestHeaderGroup() {
        return this.requestHeaders;
    }

    /* access modifiers changed from: protected */
    public HeaderGroup getResponseTrailerHeaderGroup() {
        return this.responseTrailerHeaders;
    }

    /* access modifiers changed from: protected */
    public HeaderGroup getResponseHeaderGroup() {
        return this.responseHeaders;
    }

    public Header[] getResponseHeaders(String headerName) {
        return getResponseHeaderGroup().getHeaders(headerName);
    }

    public int getStatusCode() {
        return this.statusLine.getStatusCode();
    }

    public StatusLine getStatusLine() {
        return this.statusLine;
    }

    private boolean responseAvailable() {
        return (this.responseBody == null && this.responseStream == null) ? false : true;
    }

    public Header[] getResponseHeaders() {
        return getResponseHeaderGroup().getAllHeaders();
    }

    public Header getResponseHeader(String headerName) {
        if (headerName == null) {
            return null;
        }
        return getResponseHeaderGroup().getCondensedHeader(headerName);
    }

    public long getResponseContentLength() {
        Header[] headers = getResponseHeaderGroup().getHeaders("Content-Length");
        if (headers.length == 0) {
            return -1;
        }
        if (headers.length > 1) {
            LOG.warn("Multiple content-length headers detected");
        }
        int i = headers.length - 1;
        while (i >= 0) {
            try {
                return Long.parseLong(headers[i].getValue());
            } catch (NumberFormatException e) {
                if (LOG.isWarnEnabled()) {
                    Log log = LOG;
                    StringBuffer stringBuffer = new StringBuffer();
                    stringBuffer.append("Invalid content-length value: ");
                    stringBuffer.append(e.getMessage());
                    log.warn(stringBuffer.toString());
                }
                i--;
            }
        }
        return -1;
    }

    public byte[] getResponseBody() throws IOException {
        InputStream instream;
        if (this.responseBody == null && (instream = getResponseBodyAsStream()) != null) {
            long contentLength = getResponseContentLength();
            if (contentLength <= TTL.MAX_VALUE) {
                int limit = getParams().getIntParameter(HttpMethodParams.BUFFER_WARN_TRIGGER_LIMIT, 1048576);
                if (contentLength == -1 || contentLength > ((long) limit)) {
                    LOG.warn("Going to buffer response body of large or unknown size. Using getResponseBodyAsStream instead is recommended.");
                }
                LOG.debug("Buffering response body");
                ByteArrayOutputStream outstream = new ByteArrayOutputStream(contentLength > 0 ? (int) contentLength : 4096);
                byte[] buffer = new byte[4096];
                while (true) {
                    int read = instream.read(buffer);
                    int len = read;
                    if (read <= 0) {
                        break;
                    }
                    outstream.write(buffer, 0, len);
                }
                outstream.close();
                setResponseStream((InputStream) null);
                this.responseBody = outstream.toByteArray();
            } else {
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append("Content too large to be buffered: ");
                stringBuffer.append(contentLength);
                stringBuffer.append(" bytes");
                throw new IOException(stringBuffer.toString());
            }
        }
        return this.responseBody;
    }

    public byte[] getResponseBody(int maxlen) throws IOException {
        InputStream instream;
        if (maxlen >= 0) {
            if (this.responseBody == null && (instream = getResponseBodyAsStream()) != null) {
                long contentLength = getResponseContentLength();
                if (contentLength == -1 || contentLength <= ((long) maxlen)) {
                    LOG.debug("Buffering response body");
                    ByteArrayOutputStream rawdata = new ByteArrayOutputStream(contentLength > 0 ? (int) contentLength : 4096);
                    byte[] buffer = new byte[2048];
                    int pos = 0;
                    do {
                        int len = instream.read(buffer, 0, Math.min(buffer.length, maxlen - pos));
                        if (len == -1) {
                            break;
                        }
                        rawdata.write(buffer, 0, len);
                        pos += len;
                    } while (pos < maxlen);
                    setResponseStream((InputStream) null);
                    if (pos != maxlen || instream.read() == -1) {
                        this.responseBody = rawdata.toByteArray();
                    } else {
                        StringBuffer stringBuffer = new StringBuffer();
                        stringBuffer.append("Content-Length not known but larger than ");
                        stringBuffer.append(maxlen);
                        throw new HttpContentTooLargeException(stringBuffer.toString(), maxlen);
                    }
                } else {
                    StringBuffer stringBuffer2 = new StringBuffer();
                    stringBuffer2.append("Content-Length is ");
                    stringBuffer2.append(contentLength);
                    throw new HttpContentTooLargeException(stringBuffer2.toString(), maxlen);
                }
            }
            return this.responseBody;
        }
        throw new IllegalArgumentException("maxlen must be positive");
    }

    public InputStream getResponseBodyAsStream() throws IOException {
        if (this.responseStream != null) {
            return this.responseStream;
        }
        if (this.responseBody == null) {
            return null;
        }
        InputStream byteResponseStream = new ByteArrayInputStream(this.responseBody);
        LOG.debug("re-creating response stream from byte array");
        return byteResponseStream;
    }

    public String getResponseBodyAsString() throws IOException {
        byte[] rawdata = null;
        if (responseAvailable()) {
            rawdata = getResponseBody();
        }
        if (rawdata != null) {
            return EncodingUtil.getString(rawdata, getResponseCharSet());
        }
        return null;
    }

    public String getResponseBodyAsString(int maxlen) throws IOException {
        if (maxlen >= 0) {
            byte[] rawdata = null;
            if (responseAvailable()) {
                rawdata = getResponseBody(maxlen);
            }
            if (rawdata != null) {
                return EncodingUtil.getString(rawdata, getResponseCharSet());
            }
            return null;
        }
        throw new IllegalArgumentException("maxlen must be positive");
    }

    public Header[] getResponseFooters() {
        return getResponseTrailerHeaderGroup().getAllHeaders();
    }

    public Header getResponseFooter(String footerName) {
        if (footerName == null) {
            return null;
        }
        return getResponseTrailerHeaderGroup().getCondensedHeader(footerName);
    }

    /* access modifiers changed from: protected */
    public void setResponseStream(InputStream responseStream2) {
        this.responseStream = responseStream2;
    }

    /* access modifiers changed from: protected */
    public InputStream getResponseStream() {
        return this.responseStream;
    }

    public String getStatusText() {
        return this.statusLine.getReasonPhrase();
    }

    public void setStrictMode(boolean strictMode) {
        if (strictMode) {
            this.params.makeStrict();
        } else {
            this.params.makeLenient();
        }
    }

    public boolean isStrictMode() {
        return false;
    }

    public void addRequestHeader(String headerName, String headerValue) {
        addRequestHeader(new Header(headerName, headerValue));
    }

    /* access modifiers changed from: protected */
    public boolean isConnectionCloseForced() {
        return this.connectionCloseForced;
    }

    /* access modifiers changed from: protected */
    public void setConnectionCloseForced(boolean b) {
        if (LOG.isDebugEnabled()) {
            Log log = LOG;
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Force-close connection: ");
            stringBuffer.append(b);
            log.debug(stringBuffer.toString());
        }
        this.connectionCloseForced = b;
    }

    /* access modifiers changed from: protected */
    public boolean shouldCloseConnection(HttpConnection conn) {
        if (isConnectionCloseForced()) {
            LOG.debug("Should force-close connection.");
            return true;
        }
        Header connectionHeader = null;
        if (!conn.isTransparent()) {
            connectionHeader = this.responseHeaders.getFirstHeader("proxy-connection");
        }
        if (connectionHeader == null) {
            connectionHeader = this.responseHeaders.getFirstHeader("connection");
        }
        if (connectionHeader == null) {
            connectionHeader = this.requestHeaders.getFirstHeader("connection");
        }
        if (connectionHeader != null) {
            if (connectionHeader.getValue().equalsIgnoreCase("close")) {
                if (LOG.isDebugEnabled()) {
                    Log log = LOG;
                    StringBuffer stringBuffer = new StringBuffer();
                    stringBuffer.append("Should close connection in response to directive: ");
                    stringBuffer.append(connectionHeader.getValue());
                    log.debug(stringBuffer.toString());
                }
                return true;
            } else if (connectionHeader.getValue().equalsIgnoreCase("keep-alive")) {
                if (!LOG.isDebugEnabled()) {
                    return false;
                }
                Log log2 = LOG;
                StringBuffer stringBuffer2 = new StringBuffer();
                stringBuffer2.append("Should NOT close connection in response to directive: ");
                stringBuffer2.append(connectionHeader.getValue());
                log2.debug(stringBuffer2.toString());
                return false;
            } else if (LOG.isDebugEnabled()) {
                Log log3 = LOG;
                StringBuffer stringBuffer3 = new StringBuffer();
                stringBuffer3.append("Unknown directive: ");
                stringBuffer3.append(connectionHeader.toExternalForm());
                log3.debug(stringBuffer3.toString());
            }
        }
        LOG.debug("Resorting to protocol version default close connection policy");
        if (this.effectiveVersion.greaterEquals(HttpVersion.HTTP_1_1)) {
            if (LOG.isDebugEnabled()) {
                Log log4 = LOG;
                StringBuffer stringBuffer4 = new StringBuffer();
                stringBuffer4.append("Should NOT close connection, using ");
                stringBuffer4.append(this.effectiveVersion.toString());
                log4.debug(stringBuffer4.toString());
            }
        } else if (LOG.isDebugEnabled()) {
            Log log5 = LOG;
            StringBuffer stringBuffer5 = new StringBuffer();
            stringBuffer5.append("Should close connection, using ");
            stringBuffer5.append(this.effectiveVersion.toString());
            log5.debug(stringBuffer5.toString());
        }
        return this.effectiveVersion.lessEquals(HttpVersion.HTTP_1_0);
    }

    private void checkExecuteConditions(HttpState state, HttpConnection conn) throws HttpException {
        if (state == null) {
            throw new IllegalArgumentException("HttpState parameter may not be null");
        } else if (conn == null) {
            throw new IllegalArgumentException("HttpConnection parameter may not be null");
        } else if (this.aborted) {
            throw new IllegalStateException("Method has been aborted");
        } else if (!validate()) {
            throw new ProtocolException("HttpMethodBase object not valid");
        }
    }

    public int execute(HttpState state, HttpConnection conn) throws HttpException, IOException {
        LOG.trace("enter HttpMethodBase.execute(HttpState, HttpConnection)");
        this.responseConnection = conn;
        checkExecuteConditions(state, conn);
        this.statusLine = null;
        this.connectionCloseForced = false;
        conn.setLastResponseInputStream((InputStream) null);
        if (this.effectiveVersion == null) {
            this.effectiveVersion = this.params.getVersion();
        }
        writeRequest(state, conn);
        this.requestSent = true;
        readResponse(state, conn);
        this.used = true;
        return this.statusLine.getStatusCode();
    }

    public void abort() {
        if (!this.aborted) {
            this.aborted = true;
            HttpConnection conn = this.responseConnection;
            if (conn != null) {
                conn.close();
            }
        }
    }

    public boolean hasBeenUsed() {
        return this.used;
    }

    public void recycle() {
        LOG.trace("enter HttpMethodBase.recycle()");
        releaseConnection();
        this.path = null;
        this.followRedirects = false;
        this.doAuthentication = true;
        this.queryString = null;
        getRequestHeaderGroup().clear();
        getResponseHeaderGroup().clear();
        getResponseTrailerHeaderGroup().clear();
        this.statusLine = null;
        this.effectiveVersion = null;
        this.aborted = false;
        this.used = false;
        this.params = new HttpMethodParams();
        this.responseBody = null;
        this.recoverableExceptionCount = 0;
        this.connectionCloseForced = false;
        this.hostAuthState.invalidate();
        this.proxyAuthState.invalidate();
        this.cookiespec = null;
        this.requestSent = false;
    }

    public void releaseConnection() {
        try {
            if (this.responseStream != null) {
                try {
                    this.responseStream.close();
                } catch (IOException e) {
                }
            }
        } finally {
            ensureConnectionRelease();
        }
    }

    public void removeRequestHeader(String headerName) {
        Header[] headers = getRequestHeaderGroup().getHeaders(headerName);
        for (Header removeHeader : headers) {
            getRequestHeaderGroup().removeHeader(removeHeader);
        }
    }

    public void removeRequestHeader(Header header) {
        if (header != null) {
            getRequestHeaderGroup().removeHeader(header);
        }
    }

    public boolean validate() {
        return true;
    }

    private CookieSpec getCookieSpec(HttpState state) {
        if (this.cookiespec == null) {
            int i = state.getCookiePolicy();
            if (i == -1) {
                this.cookiespec = CookiePolicy.getCookieSpec(this.params.getCookiePolicy());
            } else {
                this.cookiespec = CookiePolicy.getSpecByPolicy(i);
            }
            this.cookiespec.setValidDateFormats((Collection) this.params.getParameter(HttpMethodParams.DATE_PATTERNS));
        }
        return this.cookiespec;
    }

    /* access modifiers changed from: protected */
    public void addCookieRequestHeader(HttpState state, HttpConnection conn) throws IOException, HttpException {
        LOG.trace("enter HttpMethodBase.addCookieRequestHeader(HttpState, HttpConnection)");
        Header[] cookieheaders = getRequestHeaderGroup().getHeaders("Cookie");
        for (Header cookieheader : cookieheaders) {
            if (cookieheader.isAutogenerated()) {
                getRequestHeaderGroup().removeHeader(cookieheader);
            }
        }
        CookieSpec matcher = getCookieSpec(state);
        String host = this.params.getVirtualHost();
        if (host == null) {
            host = conn.getHost();
        }
        Cookie[] cookies = matcher.match(host, conn.getPort(), getPath(), conn.isSecure(), state.getCookies());
        if (cookies != null && cookies.length > 0) {
            if (getParams().isParameterTrue(HttpMethodParams.SINGLE_COOKIE_HEADER)) {
                getRequestHeaderGroup().addHeader(new Header("Cookie", matcher.formatCookies(cookies), true));
            } else {
                for (Cookie formatCookie : cookies) {
                    getRequestHeaderGroup().addHeader(new Header("Cookie", matcher.formatCookie(formatCookie), true));
                }
            }
            if ((matcher instanceof CookieVersionSupport) != 0) {
                CookieVersionSupport versupport = (CookieVersionSupport) matcher;
                int ver = versupport.getVersion();
                boolean needVersionHeader = false;
                for (Cookie version : cookies) {
                    if (ver != version.getVersion()) {
                        needVersionHeader = true;
                    }
                }
                if (needVersionHeader) {
                    getRequestHeaderGroup().addHeader(versupport.getVersionHeader());
                }
            }
        }
    }

    /* access modifiers changed from: protected */
    public void addHostRequestHeader(HttpState state, HttpConnection conn) throws IOException, HttpException {
        LOG.trace("enter HttpMethodBase.addHostRequestHeader(HttpState, HttpConnection)");
        String host = this.params.getVirtualHost();
        if (host != null) {
            Log log = LOG;
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Using virtual host name: ");
            stringBuffer.append(host);
            log.debug(stringBuffer.toString());
        } else {
            host = conn.getHost();
        }
        int port = conn.getPort();
        if (LOG.isDebugEnabled()) {
            LOG.debug("Adding Host request header");
        }
        if (conn.getProtocol().getDefaultPort() != port) {
            StringBuffer stringBuffer2 = new StringBuffer();
            stringBuffer2.append(host);
            stringBuffer2.append(":");
            stringBuffer2.append(port);
            host = stringBuffer2.toString();
        }
        setRequestHeader("Host", host);
    }

    /* access modifiers changed from: protected */
    public void addProxyConnectionHeader(HttpState state, HttpConnection conn) throws IOException, HttpException {
        LOG.trace("enter HttpMethodBase.addProxyConnectionHeader(HttpState, HttpConnection)");
        if (!conn.isTransparent() && getRequestHeader("Proxy-Connection") == null) {
            addRequestHeader("Proxy-Connection", "Keep-Alive");
        }
    }

    /* access modifiers changed from: protected */
    public void addRequestHeaders(HttpState state, HttpConnection conn) throws IOException, HttpException {
        LOG.trace("enter HttpMethodBase.addRequestHeaders(HttpState, HttpConnection)");
        addUserAgentRequestHeader(state, conn);
        addHostRequestHeader(state, conn);
        addCookieRequestHeader(state, conn);
        addProxyConnectionHeader(state, conn);
    }

    /* access modifiers changed from: protected */
    public void addUserAgentRequestHeader(HttpState state, HttpConnection conn) throws IOException, HttpException {
        LOG.trace("enter HttpMethodBase.addUserAgentRequestHeaders(HttpState, HttpConnection)");
        if (getRequestHeader("User-Agent") == null) {
            String agent = (String) getParams().getParameter(HttpMethodParams.USER_AGENT);
            if (agent == null) {
                agent = "Jakarta Commons-HttpClient";
            }
            setRequestHeader("User-Agent", agent);
        }
    }

    /* access modifiers changed from: protected */
    public void checkNotUsed() throws IllegalStateException {
        if (this.used) {
            throw new IllegalStateException("Already used.");
        }
    }

    /* access modifiers changed from: protected */
    public void checkUsed() throws IllegalStateException {
        if (!this.used) {
            throw new IllegalStateException("Not Used.");
        }
    }

    protected static String generateRequestLine(HttpConnection connection, String name, String requestPath, String query, String version) {
        LOG.trace("enter HttpMethodBase.generateRequestLine(HttpConnection, String, String, String, String)");
        StringBuffer buf = new StringBuffer();
        buf.append(name);
        buf.append(" ");
        if (!connection.isTransparent()) {
            Protocol protocol = connection.getProtocol();
            buf.append(protocol.getScheme().toLowerCase());
            buf.append("://");
            buf.append(connection.getHost());
            if (!(connection.getPort() == -1 || connection.getPort() == protocol.getDefaultPort())) {
                buf.append(":");
                buf.append(connection.getPort());
            }
        }
        if (requestPath == null) {
            buf.append(CookieSpec.PATH_DELIM);
        } else {
            if (!connection.isTransparent() && !requestPath.startsWith(CookieSpec.PATH_DELIM)) {
                buf.append(CookieSpec.PATH_DELIM);
            }
            buf.append(requestPath);
        }
        if (query != null) {
            if (query.indexOf("?") != 0) {
                buf.append("?");
            }
            buf.append(query);
        }
        buf.append(" ");
        buf.append(version);
        buf.append("\r\n");
        return buf.toString();
    }

    /* access modifiers changed from: protected */
    public void processResponseBody(HttpState state, HttpConnection conn) {
    }

    /* access modifiers changed from: protected */
    public void processResponseHeaders(HttpState state, HttpConnection conn) {
        LOG.trace("enter HttpMethodBase.processResponseHeaders(HttpState, HttpConnection)");
        CookieSpec parser = getCookieSpec(state);
        processCookieHeaders(parser, getResponseHeaderGroup().getHeaders(RFC2109Spec.SET_COOKIE_KEY), state, conn);
        if ((parser instanceof CookieVersionSupport) && ((CookieVersionSupport) parser).getVersion() > 0) {
            processCookieHeaders(parser, getResponseHeaderGroup().getHeaders(RFC2965Spec.SET_COOKIE2_KEY), state, conn);
        }
    }

    /* access modifiers changed from: protected */
    /* JADX WARNING: Removed duplicated region for block: B:36:0x00ca  */
    /* JADX WARNING: Removed duplicated region for block: B:45:0x00f0 A[SYNTHETIC] */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public void processCookieHeaders(org.apache.commons.httpclient.cookie.CookieSpec r18, org.apache.commons.httpclient.Header[] r19, org.apache.commons.httpclient.HttpState r20, org.apache.commons.httpclient.HttpConnection r21) {
        /*
            r17 = this;
            r7 = r18
            r8 = r19
            org.apache.commons.logging.Log r0 = LOG
            java.lang.String r1 = "enter HttpMethodBase.processCookieHeaders(Header[], HttpState, HttpConnection)"
            r0.trace(r1)
            r9 = r17
            org.apache.commons.httpclient.params.HttpMethodParams r0 = r9.params
            java.lang.String r0 = r0.getVirtualHost()
            if (r0 != 0) goto L_0x0019
            java.lang.String r0 = r21.getHost()
        L_0x0019:
            r10 = r0
            r11 = 0
            r0 = 0
        L_0x001c:
            r12 = r0
            int r0 = r8.length
            if (r12 >= r0) goto L_0x00fa
            r13 = r8[r12]
            r0 = 0
            r14 = r0
            int r3 = r21.getPort()     // Catch:{ MalformedCookieException -> 0x003a }
            java.lang.String r4 = r17.getPath()     // Catch:{ MalformedCookieException -> 0x003a }
            boolean r5 = r21.isSecure()     // Catch:{ MalformedCookieException -> 0x003a }
            r1 = r18
            r2 = r10
            r6 = r13
            org.apache.commons.httpclient.Cookie[] r0 = r1.parse((java.lang.String) r2, (int) r3, (java.lang.String) r4, (boolean) r5, (org.apache.commons.httpclient.Header) r6)     // Catch:{ MalformedCookieException -> 0x003a }
            r14 = r0
            goto L_0x0069
        L_0x003a:
            r0 = move-exception
            org.apache.commons.logging.Log r1 = LOG
            boolean r1 = r1.isWarnEnabled()
            if (r1 == 0) goto L_0x0069
            org.apache.commons.logging.Log r1 = LOG
            java.lang.StringBuffer r2 = new java.lang.StringBuffer
            r2.<init>()
            java.lang.String r3 = "Invalid cookie header: \""
            r2.append(r3)
            java.lang.String r3 = r13.getValue()
            r2.append(r3)
            java.lang.String r3 = "\". "
            r2.append(r3)
            java.lang.String r3 = r0.getMessage()
            r2.append(r3)
            java.lang.String r2 = r2.toString()
            r1.warn(r2)
        L_0x0069:
            if (r14 == 0) goto L_0x00f4
            r0 = 0
        L_0x006c:
            r15 = r0
            int r0 = r14.length
            if (r15 >= r0) goto L_0x00f4
            r0 = r14[r15]
            r6 = r0
            int r3 = r21.getPort()     // Catch:{ MalformedCookieException -> 0x00be }
            java.lang.String r4 = r17.getPath()     // Catch:{ MalformedCookieException -> 0x00be }
            boolean r5 = r21.isSecure()     // Catch:{ MalformedCookieException -> 0x00be }
            r1 = r18
            r2 = r10
            r16 = r6
            r1.validate(r2, r3, r4, r5, r6)     // Catch:{ MalformedCookieException -> 0x00b8 }
            r1 = r20
            r2 = r16
            r1.addCookie(r2)     // Catch:{ MalformedCookieException -> 0x00b6 }
            org.apache.commons.logging.Log r0 = LOG     // Catch:{ MalformedCookieException -> 0x00b6 }
            boolean r0 = r0.isDebugEnabled()     // Catch:{ MalformedCookieException -> 0x00b6 }
            if (r0 == 0) goto L_0x00b5
            org.apache.commons.logging.Log r0 = LOG     // Catch:{ MalformedCookieException -> 0x00b6 }
            java.lang.StringBuffer r3 = new java.lang.StringBuffer     // Catch:{ MalformedCookieException -> 0x00b6 }
            r3.<init>()     // Catch:{ MalformedCookieException -> 0x00b6 }
            java.lang.String r4 = "Cookie accepted: \""
            r3.append(r4)     // Catch:{ MalformedCookieException -> 0x00b6 }
            java.lang.String r4 = r7.formatCookie(r2)     // Catch:{ MalformedCookieException -> 0x00b6 }
            r3.append(r4)     // Catch:{ MalformedCookieException -> 0x00b6 }
            java.lang.String r4 = "\""
            r3.append(r4)     // Catch:{ MalformedCookieException -> 0x00b6 }
            java.lang.String r3 = r3.toString()     // Catch:{ MalformedCookieException -> 0x00b6 }
            r0.debug(r3)     // Catch:{ MalformedCookieException -> 0x00b6 }
        L_0x00b5:
            goto L_0x00f0
        L_0x00b6:
            r0 = move-exception
            goto L_0x00c2
        L_0x00b8:
            r0 = move-exception
            r1 = r20
            r2 = r16
            goto L_0x00c2
        L_0x00be:
            r0 = move-exception
            r1 = r20
            r2 = r6
        L_0x00c2:
            org.apache.commons.logging.Log r3 = LOG
            boolean r3 = r3.isWarnEnabled()
            if (r3 == 0) goto L_0x00f0
            org.apache.commons.logging.Log r3 = LOG
            java.lang.StringBuffer r4 = new java.lang.StringBuffer
            r4.<init>()
            java.lang.String r5 = "Cookie rejected: \""
            r4.append(r5)
            java.lang.String r5 = r7.formatCookie(r2)
            r4.append(r5)
            java.lang.String r5 = "\". "
            r4.append(r5)
            java.lang.String r5 = r0.getMessage()
            r4.append(r5)
            java.lang.String r4 = r4.toString()
            r3.warn(r4)
        L_0x00f0:
            int r0 = r15 + 1
            goto L_0x006c
        L_0x00f4:
            r1 = r20
            int r0 = r12 + 1
            goto L_0x001c
        L_0x00fa:
            r1 = r20
            return
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.httpclient.HttpMethodBase.processCookieHeaders(org.apache.commons.httpclient.cookie.CookieSpec, org.apache.commons.httpclient.Header[], org.apache.commons.httpclient.HttpState, org.apache.commons.httpclient.HttpConnection):void");
    }

    /* access modifiers changed from: protected */
    public void processStatusLine(HttpState state, HttpConnection conn) {
    }

    /* access modifiers changed from: protected */
    public void readResponse(HttpState state, HttpConnection conn) throws IOException, HttpException {
        LOG.trace("enter HttpMethodBase.readResponse(HttpState, HttpConnection)");
        while (this.statusLine == null) {
            readStatusLine(state, conn);
            processStatusLine(state, conn);
            readResponseHeaders(state, conn);
            processResponseHeaders(state, conn);
            int status = this.statusLine.getStatusCode();
            if (status >= 100 && status < 200) {
                if (LOG.isInfoEnabled()) {
                    Log log = LOG;
                    StringBuffer stringBuffer = new StringBuffer();
                    stringBuffer.append("Discarding unexpected response: ");
                    stringBuffer.append(this.statusLine.toString());
                    log.info(stringBuffer.toString());
                }
                this.statusLine = null;
            }
        }
        readResponseBody(state, conn);
        processResponseBody(state, conn);
    }

    /* access modifiers changed from: protected */
    public void readResponseBody(HttpState state, HttpConnection conn) throws IOException, HttpException {
        LOG.trace("enter HttpMethodBase.readResponseBody(HttpState, HttpConnection)");
        InputStream stream = readResponseBody(conn);
        if (stream == null) {
            responseBodyConsumed();
            return;
        }
        conn.setLastResponseInputStream(stream);
        setResponseStream(stream);
    }

    private InputStream readResponseBody(HttpConnection conn) throws HttpException, IOException {
        LOG.trace("enter HttpMethodBase.readResponseBody(HttpConnection)");
        this.responseBody = null;
        InputStream is = conn.getResponseInputStream();
        if (Wire.CONTENT_WIRE.enabled()) {
            is = new WireLogInputStream(is, Wire.CONTENT_WIRE);
        }
        boolean canHaveBody = canResponseHaveBody(this.statusLine.getStatusCode());
        InputStream result = null;
        Header transferEncodingHeader = this.responseHeaders.getFirstHeader("Transfer-Encoding");
        if (transferEncodingHeader != null) {
            String transferEncoding = transferEncodingHeader.getValue();
            if (!HttpHeaders.Values.CHUNKED.equalsIgnoreCase(transferEncoding) && !"identity".equalsIgnoreCase(transferEncoding) && LOG.isWarnEnabled()) {
                Log log = LOG;
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append("Unsupported transfer encoding: ");
                stringBuffer.append(transferEncoding);
                log.warn(stringBuffer.toString());
            }
            HeaderElement[] encodings = transferEncodingHeader.getElements();
            int len = encodings.length;
            if (len <= 0 || !HttpHeaders.Values.CHUNKED.equalsIgnoreCase(encodings[len - 1].getName())) {
                LOG.info("Response content is not chunk-encoded");
                setConnectionCloseForced(true);
                result = is;
            } else if (conn.isResponseAvailable(conn.getParams().getSoTimeout())) {
                result = new ChunkedInputStream(is, this);
            } else if (!getParams().isParameterTrue(HttpMethodParams.STRICT_TRANSFER_ENCODING)) {
                LOG.warn("Chunk-encoded body missing");
            } else {
                throw new ProtocolException("Chunk-encoded body declared but not sent");
            }
        } else {
            long expectedLength = getResponseContentLength();
            if (expectedLength == -1) {
                if (canHaveBody && this.effectiveVersion.greaterEquals(HttpVersion.HTTP_1_1)) {
                    Header connectionHeader = this.responseHeaders.getFirstHeader("Connection");
                    String connectionDirective = null;
                    if (connectionHeader != null) {
                        connectionDirective = connectionHeader.getValue();
                    }
                    if (!"close".equalsIgnoreCase(connectionDirective)) {
                        LOG.info("Response content length is not known");
                        setConnectionCloseForced(true);
                    }
                }
                result = is;
            } else {
                result = new ContentLengthInputStream(is, expectedLength);
            }
        }
        if (!canHaveBody) {
            result = null;
        }
        if (result != null) {
            return new AutoCloseInputStream(result, new ResponseConsumedWatcher() {
                public void responseConsumed() {
                    HttpMethodBase.this.responseBodyConsumed();
                }
            });
        }
        return result;
    }

    /* access modifiers changed from: protected */
    public void readResponseHeaders(HttpState state, HttpConnection conn) throws IOException, HttpException {
        LOG.trace("enter HttpMethodBase.readResponseHeaders(HttpState,HttpConnection)");
        getResponseHeaderGroup().clear();
        getResponseHeaderGroup().setHeaders(HttpParser.parseHeaders(conn.getResponseInputStream(), getParams().getHttpElementCharset()));
    }

    /* access modifiers changed from: protected */
    public void readStatusLine(HttpState state, HttpConnection conn) throws IOException, HttpException {
        LOG.trace("enter HttpMethodBase.readStatusLine(HttpState, HttpConnection)");
        int maxGarbageLines = getParams().getIntParameter(HttpMethodParams.STATUS_LINE_GARBAGE_LIMIT, Integer.MAX_VALUE);
        int count = 0;
        while (true) {
            String s = conn.readLine(getParams().getHttpElementCharset());
            if (s == null && count == 0) {
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append("The server ");
                stringBuffer.append(conn.getHost());
                stringBuffer.append(" failed to respond");
                throw new NoHttpResponseException(stringBuffer.toString());
            }
            if (Wire.HEADER_WIRE.enabled()) {
                Wire wire = Wire.HEADER_WIRE;
                StringBuffer stringBuffer2 = new StringBuffer();
                stringBuffer2.append(s);
                stringBuffer2.append("\r\n");
                wire.input(stringBuffer2.toString());
            }
            if (s != null && StatusLine.startsWithHTTP(s)) {
                this.statusLine = new StatusLine(s);
                String versionStr = this.statusLine.getHttpVersion();
                if (!getParams().isParameterFalse(HttpMethodParams.UNAMBIGUOUS_STATUS_LINE) || !versionStr.equals("HTTP")) {
                    this.effectiveVersion = HttpVersion.parse(versionStr);
                    return;
                }
                getParams().setVersion(HttpVersion.HTTP_1_0);
                if (LOG.isWarnEnabled()) {
                    Log log = LOG;
                    StringBuffer stringBuffer3 = new StringBuffer();
                    stringBuffer3.append("Ambiguous status line (HTTP protocol version missing):");
                    stringBuffer3.append(this.statusLine.toString());
                    log.warn(stringBuffer3.toString());
                    return;
                }
                return;
            } else if (s == null || count >= maxGarbageLines) {
                StringBuffer stringBuffer4 = new StringBuffer();
                stringBuffer4.append("The server ");
                stringBuffer4.append(conn.getHost());
                stringBuffer4.append(" failed to respond with a valid HTTP response");
            } else {
                count++;
            }
        }
        StringBuffer stringBuffer42 = new StringBuffer();
        stringBuffer42.append("The server ");
        stringBuffer42.append(conn.getHost());
        stringBuffer42.append(" failed to respond with a valid HTTP response");
        throw new ProtocolException(stringBuffer42.toString());
    }

    /* access modifiers changed from: protected */
    public void writeRequest(HttpState state, HttpConnection conn) throws IOException, HttpException {
        LOG.trace("enter HttpMethodBase.writeRequest(HttpState, HttpConnection)");
        writeRequestLine(state, conn);
        writeRequestHeaders(state, conn);
        conn.writeLine();
        if (Wire.HEADER_WIRE.enabled()) {
            Wire.HEADER_WIRE.output("\r\n");
        }
        HttpVersion ver = getParams().getVersion();
        Header expectheader = getRequestHeader("Expect");
        String expectvalue = null;
        if (expectheader != null) {
            expectvalue = expectheader.getValue();
        }
        if (expectvalue != null && expectvalue.compareToIgnoreCase("100-continue") == 0) {
            if (ver.greaterEquals(HttpVersion.HTTP_1_1)) {
                conn.flushRequestOutputStream();
                int readTimeout = conn.getParams().getSoTimeout();
                try {
                    conn.setSocketTimeout(3000);
                    readStatusLine(state, conn);
                    processStatusLine(state, conn);
                    readResponseHeaders(state, conn);
                    processResponseHeaders(state, conn);
                    if (this.statusLine.getStatusCode() == 100) {
                        this.statusLine = null;
                        LOG.debug("OK to continue received");
                        conn.setSocketTimeout(readTimeout);
                    } else {
                        conn.setSocketTimeout(readTimeout);
                        return;
                    }
                } catch (InterruptedIOException e) {
                    if (ExceptionUtil.isSocketTimeoutException(e)) {
                        removeRequestHeader("Expect");
                        LOG.info("100 (continue) read timeout. Resume sending the request");
                    } else {
                        throw e;
                    }
                } catch (Throwable th) {
                    conn.setSocketTimeout(readTimeout);
                    throw th;
                }
            } else {
                removeRequestHeader("Expect");
                LOG.info("'Expect: 100-continue' handshake is only supported by HTTP/1.1 or higher");
            }
        }
        writeRequestBody(state, conn);
        conn.flushRequestOutputStream();
    }

    /* access modifiers changed from: protected */
    public boolean writeRequestBody(HttpState state, HttpConnection conn) throws IOException, HttpException {
        return true;
    }

    /* access modifiers changed from: protected */
    public void writeRequestHeaders(HttpState state, HttpConnection conn) throws IOException, HttpException {
        LOG.trace("enter HttpMethodBase.writeRequestHeaders(HttpState,HttpConnection)");
        addRequestHeaders(state, conn);
        String charset = getParams().getHttpElementCharset();
        Header[] headers = getRequestHeaders();
        for (Header externalForm : headers) {
            String s = externalForm.toExternalForm();
            if (Wire.HEADER_WIRE.enabled()) {
                Wire.HEADER_WIRE.output(s);
            }
            conn.print(s, charset);
        }
    }

    /* access modifiers changed from: protected */
    public void writeRequestLine(HttpState state, HttpConnection conn) throws IOException, HttpException {
        LOG.trace("enter HttpMethodBase.writeRequestLine(HttpState, HttpConnection)");
        String requestLine = getRequestLine(conn);
        if (Wire.HEADER_WIRE.enabled()) {
            Wire.HEADER_WIRE.output(requestLine);
        }
        conn.print(requestLine, getParams().getHttpElementCharset());
    }

    private String getRequestLine(HttpConnection conn) {
        return generateRequestLine(conn, getName(), getPath(), getQueryString(), this.effectiveVersion.toString());
    }

    public HttpMethodParams getParams() {
        return this.params;
    }

    public void setParams(HttpMethodParams params2) {
        if (params2 != null) {
            this.params = params2;
            return;
        }
        throw new IllegalArgumentException("Parameters may not be null");
    }

    public HttpVersion getEffectiveVersion() {
        return this.effectiveVersion;
    }

    private static boolean canResponseHaveBody(int status) {
        LOG.trace("enter HttpMethodBase.canResponseHaveBody(int)");
        if ((status >= 100 && status <= 199) || status == 204 || status == 304) {
            return false;
        }
        return true;
    }

    public String getProxyAuthenticationRealm() {
        return this.proxyAuthState.getRealm();
    }

    public String getAuthenticationRealm() {
        return this.hostAuthState.getRealm();
    }

    /* access modifiers changed from: protected */
    public String getContentCharSet(Header contentheader) {
        NameValuePair param;
        LOG.trace("enter getContentCharSet( Header contentheader )");
        String charset = null;
        if (contentheader != null) {
            HeaderElement[] values = contentheader.getElements();
            if (values.length == 1 && (param = values[0].getParameterByName("charset")) != null) {
                charset = param.getValue();
            }
        }
        if (charset == null) {
            charset = getParams().getContentCharset();
            if (LOG.isDebugEnabled()) {
                Log log = LOG;
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append("Default charset used: ");
                stringBuffer.append(charset);
                log.debug(stringBuffer.toString());
            }
        }
        return charset;
    }

    public String getRequestCharSet() {
        return getContentCharSet(getRequestHeader("Content-Type"));
    }

    public String getResponseCharSet() {
        return getContentCharSet(getResponseHeader("Content-Type"));
    }

    public int getRecoverableExceptionCount() {
        return this.recoverableExceptionCount;
    }

    /* access modifiers changed from: protected */
    public void responseBodyConsumed() {
        this.responseStream = null;
        if (this.responseConnection != null) {
            this.responseConnection.setLastResponseInputStream((InputStream) null);
            if (shouldCloseConnection(this.responseConnection)) {
                this.responseConnection.close();
            } else {
                try {
                    if (this.responseConnection.isResponseAvailable()) {
                        if (getParams().isParameterTrue(HttpMethodParams.WARN_EXTRA_INPUT)) {
                            LOG.warn("Extra response data detected - closing connection");
                        }
                        this.responseConnection.close();
                    }
                } catch (IOException e) {
                    LOG.warn(e.getMessage());
                    this.responseConnection.close();
                }
            }
        }
        this.connectionCloseForced = false;
        ensureConnectionRelease();
    }

    private void ensureConnectionRelease() {
        if (this.responseConnection != null) {
            this.responseConnection.releaseConnection();
            this.responseConnection = null;
        }
    }

    public HostConfiguration getHostConfiguration() {
        HostConfiguration hostconfig = new HostConfiguration();
        hostconfig.setHost(this.httphost);
        return hostconfig;
    }

    public void setHostConfiguration(HostConfiguration hostconfig) {
        if (hostconfig != null) {
            this.httphost = new HttpHost(hostconfig.getHost(), hostconfig.getPort(), hostconfig.getProtocol());
        } else {
            this.httphost = null;
        }
    }

    public MethodRetryHandler getMethodRetryHandler() {
        return this.methodRetryHandler;
    }

    public void setMethodRetryHandler(MethodRetryHandler handler) {
        this.methodRetryHandler = handler;
    }

    /* access modifiers changed from: package-private */
    public void fakeResponse(StatusLine statusline, HeaderGroup responseheaders, InputStream responseStream2) {
        this.used = true;
        this.statusLine = statusline;
        this.responseHeaders = responseheaders;
        this.responseBody = null;
        this.responseStream = responseStream2;
    }

    public AuthState getHostAuthState() {
        return this.hostAuthState;
    }

    public AuthState getProxyAuthState() {
        return this.proxyAuthState;
    }

    public boolean isAborted() {
        return this.aborted;
    }

    public boolean isRequestSent() {
        return this.requestSent;
    }
}
