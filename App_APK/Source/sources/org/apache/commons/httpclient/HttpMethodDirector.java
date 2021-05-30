package org.apache.commons.httpclient;

import java.io.IOException;
import java.util.Collection;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import org.apache.commons.httpclient.auth.AuthChallengeException;
import org.apache.commons.httpclient.auth.AuthChallengeParser;
import org.apache.commons.httpclient.auth.AuthChallengeProcessor;
import org.apache.commons.httpclient.auth.AuthScheme;
import org.apache.commons.httpclient.auth.AuthScope;
import org.apache.commons.httpclient.auth.AuthState;
import org.apache.commons.httpclient.auth.AuthenticationException;
import org.apache.commons.httpclient.auth.CredentialsNotAvailableException;
import org.apache.commons.httpclient.auth.CredentialsProvider;
import org.apache.commons.httpclient.auth.MalformedChallengeException;
import org.apache.commons.httpclient.params.HostParams;
import org.apache.commons.httpclient.params.HttpClientParams;
import org.apache.commons.httpclient.params.HttpParams;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

class HttpMethodDirector {
    private static final Log LOG;
    public static final String PROXY_AUTH_CHALLENGE = "Proxy-Authenticate";
    public static final String PROXY_AUTH_RESP = "Proxy-Authorization";
    public static final String WWW_AUTH_CHALLENGE = "WWW-Authenticate";
    public static final String WWW_AUTH_RESP = "Authorization";
    static /* synthetic */ Class class$org$apache$commons$httpclient$HttpMethodDirector;
    private AuthChallengeProcessor authProcessor = null;
    private HttpConnection conn;
    private ConnectMethod connectMethod;
    private HttpConnectionManager connectionManager;
    private HostConfiguration hostConfiguration;
    private HttpClientParams params;
    private Set redirectLocations = null;
    private boolean releaseConnection = false;
    private HttpState state;

    static {
        Class cls;
        if (class$org$apache$commons$httpclient$HttpMethodDirector == null) {
            cls = class$("org.apache.commons.httpclient.HttpMethodDirector");
            class$org$apache$commons$httpclient$HttpMethodDirector = cls;
        } else {
            cls = class$org$apache$commons$httpclient$HttpMethodDirector;
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

    public HttpMethodDirector(HttpConnectionManager connectionManager2, HostConfiguration hostConfiguration2, HttpClientParams params2, HttpState state2) {
        this.connectionManager = connectionManager2;
        this.hostConfiguration = hostConfiguration2;
        this.params = params2;
        this.state = state2;
        this.authProcessor = new AuthChallengeProcessor(this.params);
    }

    /* JADX INFO: finally extract failed */
    public void executeMethod(HttpMethod method) throws IOException, HttpException {
        if (method != null) {
            this.hostConfiguration.getParams().setDefaults(this.params);
            method.getParams().setDefaults(this.hostConfiguration.getParams());
            Collection<Header> defaults = (Collection) this.hostConfiguration.getParams().getParameter(HostParams.DEFAULT_HEADERS);
            if (defaults != null) {
                for (Header addRequestHeader : defaults) {
                    method.addRequestHeader(addRequestHeader);
                }
            }
            try {
                int maxRedirects = this.params.getIntParameter(HttpClientParams.MAX_REDIRECTS, 100);
                int redirectCount = 0;
                while (true) {
                    if (this.conn != null && !this.hostConfiguration.hostEquals(this.conn)) {
                        this.conn.setLocked(false);
                        this.conn.releaseConnection();
                        this.conn = null;
                    }
                    if (this.conn == null) {
                        this.conn = this.connectionManager.getConnectionWithTimeout(this.hostConfiguration, this.params.getConnectionManagerTimeout());
                        this.conn.setLocked(true);
                        if (this.params.isAuthenticationPreemptive() || this.state.isAuthenticationPreemptive()) {
                            LOG.debug("Preemptively sending default basic credentials");
                            method.getHostAuthState().setPreemptive();
                            method.getHostAuthState().setAuthAttempted(true);
                            if (this.conn.isProxied() && !this.conn.isSecure()) {
                                method.getProxyAuthState().setPreemptive();
                                method.getProxyAuthState().setAuthAttempted(true);
                            }
                        }
                    }
                    authenticate(method);
                    executeWithRetry(method);
                    if (this.connectMethod != null) {
                        fakeResponse(method);
                        break;
                    }
                    boolean retry = false;
                    if (isRedirectNeeded(method) && processRedirectResponse(method)) {
                        retry = true;
                        redirectCount++;
                        if (redirectCount >= maxRedirects) {
                            LOG.error("Narrowly avoided an infinite loop in execute");
                            StringBuffer stringBuffer = new StringBuffer();
                            stringBuffer.append("Maximum redirects (");
                            stringBuffer.append(maxRedirects);
                            stringBuffer.append(") exceeded");
                            throw new RedirectException(stringBuffer.toString());
                        } else if (LOG.isDebugEnabled()) {
                            Log log = LOG;
                            StringBuffer stringBuffer2 = new StringBuffer();
                            stringBuffer2.append("Execute redirect ");
                            stringBuffer2.append(redirectCount);
                            stringBuffer2.append(" of ");
                            stringBuffer2.append(maxRedirects);
                            log.debug(stringBuffer2.toString());
                        }
                    }
                    if (isAuthenticationNeeded(method) && processAuthenticationResponse(method)) {
                        LOG.debug("Retry authentication");
                        retry = true;
                    }
                    if (!retry) {
                        break;
                    } else if (method.getResponseBodyAsStream() != null) {
                        method.getResponseBodyAsStream().close();
                    }
                }
                if (this.conn != null) {
                    this.conn.setLocked(false);
                }
                if ((this.releaseConnection || method.getResponseBodyAsStream() == null) && this.conn != null) {
                    this.conn.releaseConnection();
                }
            } catch (Throwable th) {
                if (this.conn != null) {
                    this.conn.setLocked(false);
                }
                if ((this.releaseConnection || method.getResponseBodyAsStream() == null) && this.conn != null) {
                    this.conn.releaseConnection();
                }
                throw th;
            }
        } else {
            throw new IllegalArgumentException("Method may not be null");
        }
    }

    private void authenticate(HttpMethod method) {
        try {
            if (this.conn.isProxied() && !this.conn.isSecure()) {
                authenticateProxy(method);
            }
            authenticateHost(method);
        } catch (AuthenticationException e) {
            LOG.error(e.getMessage(), e);
        }
    }

    private boolean cleanAuthHeaders(HttpMethod method, String name) {
        Header[] authheaders = method.getRequestHeaders(name);
        boolean clean = true;
        for (Header authheader : authheaders) {
            if (authheader.isAutogenerated()) {
                method.removeRequestHeader(authheader);
            } else {
                clean = false;
            }
        }
        return clean;
    }

    /* JADX WARNING: Code restructure failed: missing block: B:3:0x0009, code lost:
        r0 = r11.getHostAuthState();
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    private void authenticateHost(org.apache.commons.httpclient.HttpMethod r11) throws org.apache.commons.httpclient.auth.AuthenticationException {
        /*
            r10 = this;
            java.lang.String r0 = "Authorization"
            boolean r0 = r10.cleanAuthHeaders(r11, r0)
            if (r0 != 0) goto L_0x0009
            return
        L_0x0009:
            org.apache.commons.httpclient.auth.AuthState r0 = r11.getHostAuthState()
            org.apache.commons.httpclient.auth.AuthScheme r1 = r0.getAuthScheme()
            if (r1 != 0) goto L_0x0014
            return
        L_0x0014:
            boolean r2 = r0.isAuthRequested()
            if (r2 != 0) goto L_0x0020
            boolean r2 = r1.isConnectionBased()
            if (r2 != 0) goto L_0x00aa
        L_0x0020:
            org.apache.commons.httpclient.params.HttpMethodParams r2 = r11.getParams()
            java.lang.String r2 = r2.getVirtualHost()
            if (r2 != 0) goto L_0x0030
            org.apache.commons.httpclient.HttpConnection r3 = r10.conn
            java.lang.String r2 = r3.getHost()
        L_0x0030:
            org.apache.commons.httpclient.HttpConnection r3 = r10.conn
            int r3 = r3.getPort()
            org.apache.commons.httpclient.auth.AuthScope r4 = new org.apache.commons.httpclient.auth.AuthScope
            java.lang.String r5 = r1.getRealm()
            java.lang.String r6 = r1.getSchemeName()
            r4.<init>(r2, r3, r5, r6)
            org.apache.commons.logging.Log r5 = LOG
            boolean r5 = r5.isDebugEnabled()
            if (r5 == 0) goto L_0x0061
            org.apache.commons.logging.Log r5 = LOG
            java.lang.StringBuffer r6 = new java.lang.StringBuffer
            r6.<init>()
            java.lang.String r7 = "Authenticating with "
            r6.append(r7)
            r6.append(r4)
            java.lang.String r6 = r6.toString()
            r5.debug(r6)
        L_0x0061:
            org.apache.commons.httpclient.HttpState r5 = r10.state
            org.apache.commons.httpclient.Credentials r5 = r5.getCredentials(r4)
            if (r5 == 0) goto L_0x007b
            java.lang.String r6 = r1.authenticate(r5, r11)
            if (r6 == 0) goto L_0x007a
            org.apache.commons.httpclient.Header r7 = new org.apache.commons.httpclient.Header
            java.lang.String r8 = "Authorization"
            r9 = 1
            r7.<init>(r8, r6, r9)
            r11.addRequestHeader(r7)
        L_0x007a:
            goto L_0x00aa
        L_0x007b:
            org.apache.commons.logging.Log r6 = LOG
            boolean r6 = r6.isWarnEnabled()
            if (r6 == 0) goto L_0x00aa
            org.apache.commons.logging.Log r6 = LOG
            java.lang.StringBuffer r7 = new java.lang.StringBuffer
            r7.<init>()
            java.lang.String r8 = "Required credentials not available for "
            r7.append(r8)
            r7.append(r4)
            java.lang.String r7 = r7.toString()
            r6.warn(r7)
            org.apache.commons.httpclient.auth.AuthState r6 = r11.getHostAuthState()
            boolean r6 = r6.isPreemptive()
            if (r6 == 0) goto L_0x00aa
            org.apache.commons.logging.Log r6 = LOG
            java.lang.String r7 = "Preemptive authentication requested but no default credentials available"
            r6.warn(r7)
        L_0x00aa:
            return
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.httpclient.HttpMethodDirector.authenticateHost(org.apache.commons.httpclient.HttpMethod):void");
    }

    /* JADX WARNING: Code restructure failed: missing block: B:3:0x0009, code lost:
        r0 = r9.getProxyAuthState();
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    private void authenticateProxy(org.apache.commons.httpclient.HttpMethod r9) throws org.apache.commons.httpclient.auth.AuthenticationException {
        /*
            r8 = this;
            java.lang.String r0 = "Proxy-Authorization"
            boolean r0 = r8.cleanAuthHeaders(r9, r0)
            if (r0 != 0) goto L_0x0009
            return
        L_0x0009:
            org.apache.commons.httpclient.auth.AuthState r0 = r9.getProxyAuthState()
            org.apache.commons.httpclient.auth.AuthScheme r1 = r0.getAuthScheme()
            if (r1 != 0) goto L_0x0014
            return
        L_0x0014:
            boolean r2 = r0.isAuthRequested()
            if (r2 != 0) goto L_0x0020
            boolean r2 = r1.isConnectionBased()
            if (r2 != 0) goto L_0x00a0
        L_0x0020:
            org.apache.commons.httpclient.auth.AuthScope r2 = new org.apache.commons.httpclient.auth.AuthScope
            org.apache.commons.httpclient.HttpConnection r3 = r8.conn
            java.lang.String r3 = r3.getProxyHost()
            org.apache.commons.httpclient.HttpConnection r4 = r8.conn
            int r4 = r4.getProxyPort()
            java.lang.String r5 = r1.getRealm()
            java.lang.String r6 = r1.getSchemeName()
            r2.<init>(r3, r4, r5, r6)
            org.apache.commons.logging.Log r3 = LOG
            boolean r3 = r3.isDebugEnabled()
            if (r3 == 0) goto L_0x0057
            org.apache.commons.logging.Log r3 = LOG
            java.lang.StringBuffer r4 = new java.lang.StringBuffer
            r4.<init>()
            java.lang.String r5 = "Authenticating with "
            r4.append(r5)
            r4.append(r2)
            java.lang.String r4 = r4.toString()
            r3.debug(r4)
        L_0x0057:
            org.apache.commons.httpclient.HttpState r3 = r8.state
            org.apache.commons.httpclient.Credentials r3 = r3.getProxyCredentials(r2)
            if (r3 == 0) goto L_0x0071
            java.lang.String r4 = r1.authenticate(r3, r9)
            if (r4 == 0) goto L_0x0070
            org.apache.commons.httpclient.Header r5 = new org.apache.commons.httpclient.Header
            java.lang.String r6 = "Proxy-Authorization"
            r7 = 1
            r5.<init>(r6, r4, r7)
            r9.addRequestHeader(r5)
        L_0x0070:
            goto L_0x00a0
        L_0x0071:
            org.apache.commons.logging.Log r4 = LOG
            boolean r4 = r4.isWarnEnabled()
            if (r4 == 0) goto L_0x00a0
            org.apache.commons.logging.Log r4 = LOG
            java.lang.StringBuffer r5 = new java.lang.StringBuffer
            r5.<init>()
            java.lang.String r6 = "Required proxy credentials not available for "
            r5.append(r6)
            r5.append(r2)
            java.lang.String r5 = r5.toString()
            r4.warn(r5)
            org.apache.commons.httpclient.auth.AuthState r4 = r9.getProxyAuthState()
            boolean r4 = r4.isPreemptive()
            if (r4 == 0) goto L_0x00a0
            org.apache.commons.logging.Log r4 = LOG
            java.lang.String r5 = "Preemptive authentication requested but no default proxy credentials available"
            r4.warn(r5)
        L_0x00a0:
            return
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.httpclient.HttpMethodDirector.authenticateProxy(org.apache.commons.httpclient.HttpMethod):void");
    }

    private void applyConnectionParams(HttpMethod method) throws IOException {
        int timeout = 0;
        Object param = method.getParams().getParameter("http.socket.timeout");
        if (param == null) {
            param = this.conn.getParams().getParameter("http.socket.timeout");
        }
        if (param != null) {
            timeout = ((Integer) param).intValue();
        }
        this.conn.setSocketTimeout(timeout);
    }

    /* JADX WARNING: Code restructure failed: missing block: B:22:0x006c, code lost:
        r2 = move-exception;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:52:0x0123, code lost:
        if (r10.conn.isOpen() != false) goto L_0x0125;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:53:0x0125, code lost:
        LOG.debug("Closing the connection.");
        r10.conn.close();
     */
    /* JADX WARNING: Code restructure failed: missing block: B:54:0x0131, code lost:
        r10.releaseConnection = true;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:55:0x0133, code lost:
        throw r2;
     */
    /* JADX WARNING: Removed duplicated region for block: B:22:0x006c A[ExcHandler: RuntimeException (r2v18 'e' java.lang.RuntimeException A[CUSTOM_DECLARE]), Splitter:B:2:0x0003] */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    private void executeWithRetry(org.apache.commons.httpclient.HttpMethod r11) throws java.io.IOException, org.apache.commons.httpclient.HttpException {
        /*
            r10 = this;
            r0 = 0
        L_0x0001:
            r1 = 1
            int r0 = r0 + r1
            org.apache.commons.logging.Log r2 = LOG     // Catch:{ HttpException -> 0x011a, IOException -> 0x006f, RuntimeException -> 0x006c }
            boolean r2 = r2.isTraceEnabled()     // Catch:{ HttpException -> 0x011a, IOException -> 0x006f, RuntimeException -> 0x006c }
            if (r2 == 0) goto L_0x0026
            org.apache.commons.logging.Log r2 = LOG     // Catch:{ HttpException -> 0x011a, IOException -> 0x006f, RuntimeException -> 0x006c }
            java.lang.StringBuffer r3 = new java.lang.StringBuffer     // Catch:{ HttpException -> 0x011a, IOException -> 0x006f, RuntimeException -> 0x006c }
            r3.<init>()     // Catch:{ HttpException -> 0x011a, IOException -> 0x006f, RuntimeException -> 0x006c }
            java.lang.String r4 = "Attempt number "
            r3.append(r4)     // Catch:{ HttpException -> 0x011a, IOException -> 0x006f, RuntimeException -> 0x006c }
            r3.append(r0)     // Catch:{ HttpException -> 0x011a, IOException -> 0x006f, RuntimeException -> 0x006c }
            java.lang.String r4 = " to process request"
            r3.append(r4)     // Catch:{ HttpException -> 0x011a, IOException -> 0x006f, RuntimeException -> 0x006c }
            java.lang.String r3 = r3.toString()     // Catch:{ HttpException -> 0x011a, IOException -> 0x006f, RuntimeException -> 0x006c }
            r2.trace(r3)     // Catch:{ HttpException -> 0x011a, IOException -> 0x006f, RuntimeException -> 0x006c }
        L_0x0026:
            org.apache.commons.httpclient.HttpConnection r2 = r10.conn     // Catch:{ HttpException -> 0x011a, IOException -> 0x006f, RuntimeException -> 0x006c }
            org.apache.commons.httpclient.params.HttpConnectionParams r2 = r2.getParams()     // Catch:{ HttpException -> 0x011a, IOException -> 0x006f, RuntimeException -> 0x006c }
            boolean r2 = r2.isStaleCheckingEnabled()     // Catch:{ HttpException -> 0x011a, IOException -> 0x006f, RuntimeException -> 0x006c }
            if (r2 == 0) goto L_0x0037
            org.apache.commons.httpclient.HttpConnection r2 = r10.conn     // Catch:{ HttpException -> 0x011a, IOException -> 0x006f, RuntimeException -> 0x006c }
            r2.closeIfStale()     // Catch:{ HttpException -> 0x011a, IOException -> 0x006f, RuntimeException -> 0x006c }
        L_0x0037:
            org.apache.commons.httpclient.HttpConnection r2 = r10.conn     // Catch:{ HttpException -> 0x011a, IOException -> 0x006f, RuntimeException -> 0x006c }
            boolean r2 = r2.isOpen()     // Catch:{ HttpException -> 0x011a, IOException -> 0x006f, RuntimeException -> 0x006c }
            if (r2 != 0) goto L_0x005f
            org.apache.commons.httpclient.HttpConnection r2 = r10.conn     // Catch:{ HttpException -> 0x011a, IOException -> 0x006f, RuntimeException -> 0x006c }
            r2.open()     // Catch:{ HttpException -> 0x011a, IOException -> 0x006f, RuntimeException -> 0x006c }
            org.apache.commons.httpclient.HttpConnection r2 = r10.conn     // Catch:{ HttpException -> 0x011a, IOException -> 0x006f, RuntimeException -> 0x006c }
            boolean r2 = r2.isProxied()     // Catch:{ HttpException -> 0x011a, IOException -> 0x006f, RuntimeException -> 0x006c }
            if (r2 == 0) goto L_0x005f
            org.apache.commons.httpclient.HttpConnection r2 = r10.conn     // Catch:{ HttpException -> 0x011a, IOException -> 0x006f, RuntimeException -> 0x006c }
            boolean r2 = r2.isSecure()     // Catch:{ HttpException -> 0x011a, IOException -> 0x006f, RuntimeException -> 0x006c }
            if (r2 == 0) goto L_0x005f
            boolean r2 = r11 instanceof org.apache.commons.httpclient.ConnectMethod     // Catch:{ HttpException -> 0x011a, IOException -> 0x006f, RuntimeException -> 0x006c }
            if (r2 != 0) goto L_0x005f
            boolean r2 = r10.executeConnect()     // Catch:{ HttpException -> 0x011a, IOException -> 0x006f, RuntimeException -> 0x006c }
            if (r2 != 0) goto L_0x005f
            return
        L_0x005f:
            r10.applyConnectionParams(r11)     // Catch:{ HttpException -> 0x011a, IOException -> 0x006f, RuntimeException -> 0x006c }
            org.apache.commons.httpclient.HttpState r2 = r10.state     // Catch:{ HttpException -> 0x011a, IOException -> 0x006f, RuntimeException -> 0x006c }
            org.apache.commons.httpclient.HttpConnection r3 = r10.conn     // Catch:{ HttpException -> 0x011a, IOException -> 0x006f, RuntimeException -> 0x006c }
            r11.execute(r2, r3)     // Catch:{ HttpException -> 0x011a, IOException -> 0x006f, RuntimeException -> 0x006c }
            return
        L_0x006c:
            r2 = move-exception
            goto L_0x011c
        L_0x006f:
            r2 = move-exception
            r8 = r2
            org.apache.commons.logging.Log r2 = LOG     // Catch:{ IOException -> 0x0134, RuntimeException -> 0x006c }
            java.lang.String r3 = "Closing the connection."
            r2.debug(r3)     // Catch:{ IOException -> 0x0134, RuntimeException -> 0x006c }
            org.apache.commons.httpclient.HttpConnection r2 = r10.conn     // Catch:{ IOException -> 0x0134, RuntimeException -> 0x006c }
            r2.close()     // Catch:{ IOException -> 0x0134, RuntimeException -> 0x006c }
            boolean r2 = r11 instanceof org.apache.commons.httpclient.HttpMethodBase     // Catch:{ IOException -> 0x0134, RuntimeException -> 0x006c }
            if (r2 == 0) goto L_0x00ac
            r2 = r11
            org.apache.commons.httpclient.HttpMethodBase r2 = (org.apache.commons.httpclient.HttpMethodBase) r2     // Catch:{ IOException -> 0x0134, RuntimeException -> 0x006c }
            org.apache.commons.httpclient.MethodRetryHandler r2 = r2.getMethodRetryHandler()     // Catch:{ IOException -> 0x0134, RuntimeException -> 0x006c }
            r9 = r2
            if (r9 == 0) goto L_0x00ac
            org.apache.commons.httpclient.HttpConnection r4 = r10.conn     // Catch:{ IOException -> 0x0134, RuntimeException -> 0x006c }
            org.apache.commons.httpclient.HttpRecoverableException r5 = new org.apache.commons.httpclient.HttpRecoverableException     // Catch:{ IOException -> 0x0134, RuntimeException -> 0x006c }
            java.lang.String r2 = r8.getMessage()     // Catch:{ IOException -> 0x0134, RuntimeException -> 0x006c }
            r5.<init>(r2)     // Catch:{ IOException -> 0x0134, RuntimeException -> 0x006c }
            boolean r7 = r11.isRequestSent()     // Catch:{ IOException -> 0x0134, RuntimeException -> 0x006c }
            r2 = r9
            r3 = r11
            r6 = r0
            boolean r2 = r2.retryMethod(r3, r4, r5, r6, r7)     // Catch:{ IOException -> 0x0134, RuntimeException -> 0x006c }
            if (r2 == 0) goto L_0x00a4
            goto L_0x00ac
        L_0x00a4:
            org.apache.commons.logging.Log r2 = LOG     // Catch:{ IOException -> 0x0134, RuntimeException -> 0x006c }
            java.lang.String r3 = "Method retry handler returned false. Automatic recovery will not be attempted"
            r2.debug(r3)     // Catch:{ IOException -> 0x0134, RuntimeException -> 0x006c }
            throw r8     // Catch:{ IOException -> 0x0134, RuntimeException -> 0x006c }
        L_0x00ac:
            org.apache.commons.httpclient.params.HttpMethodParams r2 = r11.getParams()     // Catch:{ IOException -> 0x0134, RuntimeException -> 0x006c }
            java.lang.String r3 = "http.method.retry-handler"
            java.lang.Object r2 = r2.getParameter(r3)     // Catch:{ IOException -> 0x0134, RuntimeException -> 0x006c }
            org.apache.commons.httpclient.HttpMethodRetryHandler r2 = (org.apache.commons.httpclient.HttpMethodRetryHandler) r2     // Catch:{ IOException -> 0x0134, RuntimeException -> 0x006c }
            if (r2 != 0) goto L_0x00c0
            org.apache.commons.httpclient.DefaultHttpMethodRetryHandler r3 = new org.apache.commons.httpclient.DefaultHttpMethodRetryHandler     // Catch:{ IOException -> 0x0134, RuntimeException -> 0x006c }
            r3.<init>()     // Catch:{ IOException -> 0x0134, RuntimeException -> 0x006c }
            r2 = r3
        L_0x00c0:
            boolean r3 = r2.retryMethod(r11, r8, r0)     // Catch:{ IOException -> 0x0134, RuntimeException -> 0x006c }
            if (r3 == 0) goto L_0x0112
            org.apache.commons.logging.Log r3 = LOG     // Catch:{ IOException -> 0x0134, RuntimeException -> 0x006c }
            boolean r3 = r3.isInfoEnabled()     // Catch:{ IOException -> 0x0134, RuntimeException -> 0x006c }
            if (r3 == 0) goto L_0x00f8
            org.apache.commons.logging.Log r3 = LOG     // Catch:{ IOException -> 0x0134, RuntimeException -> 0x006c }
            java.lang.StringBuffer r4 = new java.lang.StringBuffer     // Catch:{ IOException -> 0x0134, RuntimeException -> 0x006c }
            r4.<init>()     // Catch:{ IOException -> 0x0134, RuntimeException -> 0x006c }
            java.lang.String r5 = "I/O exception ("
            r4.append(r5)     // Catch:{ IOException -> 0x0134, RuntimeException -> 0x006c }
            java.lang.Class r5 = r8.getClass()     // Catch:{ IOException -> 0x0134, RuntimeException -> 0x006c }
            java.lang.String r5 = r5.getName()     // Catch:{ IOException -> 0x0134, RuntimeException -> 0x006c }
            r4.append(r5)     // Catch:{ IOException -> 0x0134, RuntimeException -> 0x006c }
            java.lang.String r5 = ") caught when processing request: "
            r4.append(r5)     // Catch:{ IOException -> 0x0134, RuntimeException -> 0x006c }
            java.lang.String r5 = r8.getMessage()     // Catch:{ IOException -> 0x0134, RuntimeException -> 0x006c }
            r4.append(r5)     // Catch:{ IOException -> 0x0134, RuntimeException -> 0x006c }
            java.lang.String r4 = r4.toString()     // Catch:{ IOException -> 0x0134, RuntimeException -> 0x006c }
            r3.info(r4)     // Catch:{ IOException -> 0x0134, RuntimeException -> 0x006c }
        L_0x00f8:
            org.apache.commons.logging.Log r3 = LOG     // Catch:{ IOException -> 0x0134, RuntimeException -> 0x006c }
            boolean r3 = r3.isDebugEnabled()     // Catch:{ IOException -> 0x0134, RuntimeException -> 0x006c }
            if (r3 == 0) goto L_0x0109
            org.apache.commons.logging.Log r3 = LOG     // Catch:{ IOException -> 0x0134, RuntimeException -> 0x006c }
            java.lang.String r4 = r8.getMessage()     // Catch:{ IOException -> 0x0134, RuntimeException -> 0x006c }
            r3.debug(r4, r8)     // Catch:{ IOException -> 0x0134, RuntimeException -> 0x006c }
        L_0x0109:
            org.apache.commons.logging.Log r3 = LOG     // Catch:{ IOException -> 0x0134, RuntimeException -> 0x006c }
            java.lang.String r4 = "Retrying request"
            r3.info(r4)     // Catch:{ IOException -> 0x0134, RuntimeException -> 0x006c }
            goto L_0x0001
        L_0x0112:
            org.apache.commons.logging.Log r3 = LOG     // Catch:{ IOException -> 0x0134, RuntimeException -> 0x006c }
            java.lang.String r4 = "Method retry handler returned false. Automatic recovery will not be attempted"
            r3.debug(r4)     // Catch:{ IOException -> 0x0134, RuntimeException -> 0x006c }
            throw r8     // Catch:{ IOException -> 0x0134, RuntimeException -> 0x006c }
        L_0x011a:
            r2 = move-exception
            throw r2     // Catch:{ IOException -> 0x0134, RuntimeException -> 0x006c }
        L_0x011c:
            org.apache.commons.httpclient.HttpConnection r3 = r10.conn
            boolean r3 = r3.isOpen()
            if (r3 == 0) goto L_0x0131
            org.apache.commons.logging.Log r3 = LOG
            java.lang.String r4 = "Closing the connection."
            r3.debug(r4)
            org.apache.commons.httpclient.HttpConnection r3 = r10.conn
            r3.close()
        L_0x0131:
            r10.releaseConnection = r1
            throw r2
        L_0x0134:
            r2 = move-exception
            org.apache.commons.httpclient.HttpConnection r3 = r10.conn
            boolean r3 = r3.isOpen()
            if (r3 == 0) goto L_0x0149
            org.apache.commons.logging.Log r3 = LOG
            java.lang.String r4 = "Closing the connection."
            r3.debug(r4)
            org.apache.commons.httpclient.HttpConnection r3 = r10.conn
            r3.close()
        L_0x0149:
            r10.releaseConnection = r1
            throw r2
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.httpclient.HttpMethodDirector.executeWithRetry(org.apache.commons.httpclient.HttpMethod):void");
    }

    private boolean executeConnect() throws IOException, HttpException {
        int code;
        this.connectMethod = new ConnectMethod(this.hostConfiguration);
        this.connectMethod.getParams().setDefaults(this.hostConfiguration.getParams());
        while (true) {
            if (!this.conn.isOpen()) {
                this.conn.open();
            }
            if (this.params.isAuthenticationPreemptive() || this.state.isAuthenticationPreemptive()) {
                LOG.debug("Preemptively sending default basic credentials");
                this.connectMethod.getProxyAuthState().setPreemptive();
                this.connectMethod.getProxyAuthState().setAuthAttempted(true);
            }
            try {
                authenticateProxy(this.connectMethod);
            } catch (AuthenticationException e) {
                LOG.error(e.getMessage(), e);
            }
            applyConnectionParams(this.connectMethod);
            this.connectMethod.execute(this.state, this.conn);
            code = this.connectMethod.getStatusCode();
            boolean retry = false;
            AuthState authstate = this.connectMethod.getProxyAuthState();
            authstate.setAuthRequested(code == 407);
            if (authstate.isAuthRequested() && processAuthenticationResponse(this.connectMethod)) {
                retry = true;
            }
            if (!retry) {
                break;
            } else if (this.connectMethod.getResponseBodyAsStream() != null) {
                this.connectMethod.getResponseBodyAsStream().close();
            }
        }
        if (code < 200 || code >= 300) {
            this.conn.close();
            return false;
        }
        this.conn.tunnelCreated();
        this.connectMethod = null;
        return true;
    }

    private void fakeResponse(HttpMethod method) throws IOException, HttpException {
        LOG.debug("CONNECT failed, fake the response for the original method");
        if (method instanceof HttpMethodBase) {
            ((HttpMethodBase) method).fakeResponse(this.connectMethod.getStatusLine(), this.connectMethod.getResponseHeaderGroup(), this.connectMethod.getResponseBodyAsStream());
            method.getProxyAuthState().setAuthScheme(this.connectMethod.getProxyAuthState().getAuthScheme());
            this.connectMethod = null;
            return;
        }
        this.releaseConnection = true;
        LOG.warn("Unable to fake response on method as it is not derived from HttpMethodBase.");
    }

    private boolean processRedirectResponse(HttpMethod method) throws RedirectException {
        Header locationHeader = method.getResponseHeader("location");
        if (locationHeader == null) {
            Log log = LOG;
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Received redirect response ");
            stringBuffer.append(method.getStatusCode());
            stringBuffer.append(" but no location header");
            log.error(stringBuffer.toString());
            return false;
        }
        String location = locationHeader.getValue();
        if (LOG.isDebugEnabled()) {
            Log log2 = LOG;
            StringBuffer stringBuffer2 = new StringBuffer();
            stringBuffer2.append("Redirect requested to location '");
            stringBuffer2.append(location);
            stringBuffer2.append("'");
            log2.debug(stringBuffer2.toString());
        }
        try {
            URI currentUri = new URI(this.conn.getProtocol().getScheme(), (String) null, this.conn.getHost(), this.conn.getPort(), method.getPath());
            URI redirectUri = new URI(location, true, method.getParams().getUriCharset());
            if (!redirectUri.isRelativeURI()) {
                method.getParams().setDefaults(this.params);
            } else if (this.params.isParameterTrue(HttpClientParams.REJECT_RELATIVE_REDIRECT)) {
                Log log3 = LOG;
                StringBuffer stringBuffer3 = new StringBuffer();
                stringBuffer3.append("Relative redirect location '");
                stringBuffer3.append(location);
                stringBuffer3.append("' not allowed");
                log3.warn(stringBuffer3.toString());
                return false;
            } else {
                LOG.debug("Redirect URI is not absolute - parsing as relative");
                redirectUri = new URI(currentUri, redirectUri);
            }
            method.setURI(redirectUri);
            this.hostConfiguration.setHost(redirectUri);
            if (this.params.isParameterFalse(HttpClientParams.ALLOW_CIRCULAR_REDIRECTS)) {
                if (this.redirectLocations == null) {
                    this.redirectLocations = new HashSet();
                }
                this.redirectLocations.add(currentUri);
                try {
                    if (redirectUri.hasQuery()) {
                        redirectUri.setQuery((String) null);
                    }
                    if (this.redirectLocations.contains(redirectUri)) {
                        StringBuffer stringBuffer4 = new StringBuffer();
                        stringBuffer4.append("Circular redirect to '");
                        stringBuffer4.append(redirectUri);
                        stringBuffer4.append("'");
                        throw new CircularRedirectException(stringBuffer4.toString());
                    }
                } catch (URIException e) {
                    return false;
                }
            }
            if (LOG.isDebugEnabled()) {
                Log log4 = LOG;
                StringBuffer stringBuffer5 = new StringBuffer();
                stringBuffer5.append("Redirecting from '");
                stringBuffer5.append(currentUri.getEscapedURI());
                stringBuffer5.append("' to '");
                stringBuffer5.append(redirectUri.getEscapedURI());
                log4.debug(stringBuffer5.toString());
            }
            method.getHostAuthState().invalidate();
            return true;
        } catch (URIException ex) {
            StringBuffer stringBuffer6 = new StringBuffer();
            stringBuffer6.append("Invalid redirect location: ");
            stringBuffer6.append(location);
            throw new InvalidRedirectLocationException(stringBuffer6.toString(), location, ex);
        }
    }

    private boolean processAuthenticationResponse(HttpMethod method) {
        LOG.trace("enter HttpMethodBase.processAuthenticationResponse(HttpState, HttpConnection)");
        try {
            int statusCode = method.getStatusCode();
            if (statusCode == 401) {
                return processWWWAuthChallenge(method);
            }
            if (statusCode != 407) {
                return false;
            }
            return processProxyAuthChallenge(method);
        } catch (Exception e) {
            if (LOG.isErrorEnabled()) {
                LOG.error(e.getMessage(), e);
            }
            return false;
        }
    }

    private boolean processWWWAuthChallenge(HttpMethod method) throws MalformedChallengeException, AuthenticationException {
        AuthState authstate = method.getHostAuthState();
        Map challenges = AuthChallengeParser.parseChallenges(method.getResponseHeaders("WWW-Authenticate"));
        if (challenges.isEmpty()) {
            LOG.debug("Authentication challenge(s) not found");
            return false;
        }
        AuthScheme authscheme = null;
        try {
            authscheme = this.authProcessor.processChallenge(authstate, challenges);
        } catch (AuthChallengeException e) {
            if (LOG.isWarnEnabled()) {
                LOG.warn(e.getMessage());
            }
        }
        if (authscheme == null) {
            return false;
        }
        String host = method.getParams().getVirtualHost();
        if (host == null) {
            host = this.conn.getHost();
        }
        AuthScope authscope = new AuthScope(host, this.conn.getPort(), authscheme.getRealm(), authscheme.getSchemeName());
        if (LOG.isDebugEnabled()) {
            Log log = LOG;
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Authentication scope: ");
            stringBuffer.append(authscope);
            log.debug(stringBuffer.toString());
        }
        if (!authstate.isAuthAttempted() || !authscheme.isComplete()) {
            authstate.setAuthAttempted(true);
            Credentials credentials = this.state.getCredentials(authscope);
            if (credentials == null) {
                credentials = promptForCredentials(authscheme, method.getParams(), authscope);
            }
            if (credentials != null) {
                return true;
            }
            if (LOG.isInfoEnabled()) {
                Log log2 = LOG;
                StringBuffer stringBuffer2 = new StringBuffer();
                stringBuffer2.append("No credentials available for ");
                stringBuffer2.append(authscope);
                log2.info(stringBuffer2.toString());
            }
            return false;
        } else if (promptForCredentials(authscheme, method.getParams(), authscope) != null) {
            return true;
        } else {
            if (LOG.isInfoEnabled()) {
                Log log3 = LOG;
                StringBuffer stringBuffer3 = new StringBuffer();
                stringBuffer3.append("Failure authenticating with ");
                stringBuffer3.append(authscope);
                log3.info(stringBuffer3.toString());
            }
            return false;
        }
    }

    private boolean processProxyAuthChallenge(HttpMethod method) throws MalformedChallengeException, AuthenticationException {
        AuthState authstate = method.getProxyAuthState();
        Map proxyChallenges = AuthChallengeParser.parseChallenges(method.getResponseHeaders("Proxy-Authenticate"));
        if (proxyChallenges.isEmpty()) {
            LOG.debug("Proxy authentication challenge(s) not found");
            return false;
        }
        AuthScheme authscheme = null;
        try {
            authscheme = this.authProcessor.processChallenge(authstate, proxyChallenges);
        } catch (AuthChallengeException e) {
            if (LOG.isWarnEnabled()) {
                LOG.warn(e.getMessage());
            }
        }
        if (authscheme == null) {
            return false;
        }
        AuthScope authscope = new AuthScope(this.conn.getProxyHost(), this.conn.getProxyPort(), authscheme.getRealm(), authscheme.getSchemeName());
        if (LOG.isDebugEnabled()) {
            Log log = LOG;
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Proxy authentication scope: ");
            stringBuffer.append(authscope);
            log.debug(stringBuffer.toString());
        }
        if (!authstate.isAuthAttempted() || !authscheme.isComplete()) {
            authstate.setAuthAttempted(true);
            Credentials credentials = this.state.getProxyCredentials(authscope);
            if (credentials == null) {
                credentials = promptForProxyCredentials(authscheme, method.getParams(), authscope);
            }
            if (credentials != null) {
                return true;
            }
            if (LOG.isInfoEnabled()) {
                Log log2 = LOG;
                StringBuffer stringBuffer2 = new StringBuffer();
                stringBuffer2.append("No credentials available for ");
                stringBuffer2.append(authscope);
                log2.info(stringBuffer2.toString());
            }
            return false;
        } else if (promptForProxyCredentials(authscheme, method.getParams(), authscope) != null) {
            return true;
        } else {
            if (LOG.isInfoEnabled()) {
                Log log3 = LOG;
                StringBuffer stringBuffer3 = new StringBuffer();
                stringBuffer3.append("Failure authenticating with ");
                stringBuffer3.append(authscope);
                log3.info(stringBuffer3.toString());
            }
            return false;
        }
    }

    private boolean isRedirectNeeded(HttpMethod method) {
        int statusCode = method.getStatusCode();
        if (statusCode != 307) {
            switch (statusCode) {
                case 301:
                case 302:
                case 303:
                    break;
                default:
                    return false;
            }
        }
        LOG.debug("Redirect required");
        if (method.getFollowRedirects()) {
            return true;
        }
        return false;
    }

    private boolean isAuthenticationNeeded(HttpMethod method) {
        method.getHostAuthState().setAuthRequested(method.getStatusCode() == 401);
        method.getProxyAuthState().setAuthRequested(method.getStatusCode() == 407);
        if (!method.getHostAuthState().isAuthRequested() && !method.getProxyAuthState().isAuthRequested()) {
            return false;
        }
        LOG.debug("Authorization required");
        if (method.getDoAuthentication()) {
            return true;
        }
        LOG.info("Authentication requested but doAuthentication is disabled");
        return false;
    }

    private Credentials promptForCredentials(AuthScheme authScheme, HttpParams params2, AuthScope authscope) {
        LOG.debug("Credentials required");
        Credentials creds = null;
        CredentialsProvider credProvider = (CredentialsProvider) params2.getParameter(CredentialsProvider.PROVIDER);
        if (credProvider != null) {
            try {
                creds = credProvider.getCredentials(authScheme, authscope.getHost(), authscope.getPort(), false);
            } catch (CredentialsNotAvailableException e) {
                LOG.warn(e.getMessage());
            }
            if (creds != null) {
                this.state.setCredentials(authscope, creds);
                if (LOG.isDebugEnabled()) {
                    Log log = LOG;
                    StringBuffer stringBuffer = new StringBuffer();
                    stringBuffer.append(authscope);
                    stringBuffer.append(" new credentials given");
                    log.debug(stringBuffer.toString());
                }
            }
        } else {
            LOG.debug("Credentials provider not available");
        }
        return creds;
    }

    private Credentials promptForProxyCredentials(AuthScheme authScheme, HttpParams params2, AuthScope authscope) {
        LOG.debug("Proxy credentials required");
        Credentials creds = null;
        CredentialsProvider credProvider = (CredentialsProvider) params2.getParameter(CredentialsProvider.PROVIDER);
        if (credProvider != null) {
            try {
                creds = credProvider.getCredentials(authScheme, authscope.getHost(), authscope.getPort(), true);
            } catch (CredentialsNotAvailableException e) {
                LOG.warn(e.getMessage());
            }
            if (creds != null) {
                this.state.setProxyCredentials(authscope, creds);
                if (LOG.isDebugEnabled()) {
                    Log log = LOG;
                    StringBuffer stringBuffer = new StringBuffer();
                    stringBuffer.append(authscope);
                    stringBuffer.append(" new credentials given");
                    log.debug(stringBuffer.toString());
                }
            }
        } else {
            LOG.debug("Proxy credentials provider not available");
        }
        return creds;
    }

    public HostConfiguration getHostConfiguration() {
        return this.hostConfiguration;
    }

    public HttpState getState() {
        return this.state;
    }

    public HttpConnectionManager getConnectionManager() {
        return this.connectionManager;
    }

    public HttpParams getParams() {
        return this.params;
    }
}
