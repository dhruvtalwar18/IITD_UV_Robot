package org.apache.commons.httpclient.auth;

import java.util.HashMap;
import java.util.Map;
import org.apache.commons.httpclient.Credentials;
import org.apache.commons.httpclient.Header;
import org.apache.commons.httpclient.HttpConnection;
import org.apache.commons.httpclient.HttpMethod;
import org.apache.commons.httpclient.HttpState;
import org.apache.commons.httpclient.UsernamePasswordCredentials;
import org.apache.commons.httpclient.cookie.CookiePolicy;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

public final class HttpAuthenticator {
    private static final Log LOG;
    public static final String PROXY_AUTH = "Proxy-Authenticate";
    public static final String PROXY_AUTH_RESP = "Proxy-Authorization";
    public static final String WWW_AUTH = "WWW-Authenticate";
    public static final String WWW_AUTH_RESP = "Authorization";
    static /* synthetic */ Class class$org$apache$commons$httpclient$auth$HttpAuthenticator;

    static {
        Class cls;
        if (class$org$apache$commons$httpclient$auth$HttpAuthenticator == null) {
            cls = class$("org.apache.commons.httpclient.auth.HttpAuthenticator");
            class$org$apache$commons$httpclient$auth$HttpAuthenticator = cls;
        } else {
            cls = class$org$apache$commons$httpclient$auth$HttpAuthenticator;
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

    public static AuthScheme selectAuthScheme(Header[] challenges) throws MalformedChallengeException {
        LOG.trace("enter HttpAuthenticator.selectAuthScheme(Header[])");
        if (challenges == null) {
            throw new IllegalArgumentException("Array of challenges may not be null");
        } else if (challenges.length != 0) {
            Map challengemap = new HashMap(challenges.length);
            for (Header value : challenges) {
                String challenge = value.getValue();
                challengemap.put(AuthChallengeParser.extractScheme(challenge), challenge);
            }
            String challenge2 = (String) challengemap.get("ntlm");
            if (challenge2 != null) {
                return new NTLMScheme(challenge2);
            }
            String challenge3 = (String) challengemap.get("digest");
            if (challenge3 != null) {
                return new DigestScheme(challenge3);
            }
            String challenge4 = (String) challengemap.get(AuthState.PREEMPTIVE_AUTH_SCHEME);
            if (challenge4 != null) {
                return new BasicScheme(challenge4);
            }
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Authentication scheme(s) not supported: ");
            stringBuffer.append(challengemap.toString());
            throw new UnsupportedOperationException(stringBuffer.toString());
        } else {
            throw new IllegalArgumentException("Array of challenges may not be empty");
        }
    }

    private static boolean doAuthenticateDefault(HttpMethod method, HttpConnection conn, HttpState state, boolean proxy) throws AuthenticationException {
        if (method == null) {
            throw new IllegalArgumentException("HTTP method may not be null");
        } else if (state != null) {
            String host = null;
            if (conn != null) {
                host = proxy ? conn.getProxyHost() : conn.getHost();
            }
            Credentials credentials = proxy ? state.getProxyCredentials((String) null, host) : state.getCredentials((String) null, host);
            if (credentials == null) {
                return false;
            }
            if (credentials instanceof UsernamePasswordCredentials) {
                String auth = BasicScheme.authenticate((UsernamePasswordCredentials) credentials, method.getParams().getCredentialCharset());
                if (auth == null) {
                    return false;
                }
                method.addRequestHeader(new Header(proxy ? "Proxy-Authorization" : "Authorization", auth, true));
                return true;
            }
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Credentials cannot be used for basic authentication: ");
            stringBuffer.append(credentials.toString());
            throw new InvalidCredentialsException(stringBuffer.toString());
        } else {
            throw new IllegalArgumentException("HTTP state may not be null");
        }
    }

    public static boolean authenticateDefault(HttpMethod method, HttpConnection conn, HttpState state) throws AuthenticationException {
        LOG.trace("enter HttpAuthenticator.authenticateDefault(HttpMethod, HttpConnection, HttpState)");
        return doAuthenticateDefault(method, conn, state, false);
    }

    public static boolean authenticateProxyDefault(HttpMethod method, HttpConnection conn, HttpState state) throws AuthenticationException {
        LOG.trace("enter HttpAuthenticator.authenticateProxyDefault(HttpMethod, HttpState)");
        return doAuthenticateDefault(method, conn, state, true);
    }

    private static boolean doAuthenticate(AuthScheme authscheme, HttpMethod method, HttpConnection conn, HttpState state, boolean proxy) throws AuthenticationException {
        if (authscheme == null) {
            throw new IllegalArgumentException("Authentication scheme may not be null");
        } else if (method == null) {
            throw new IllegalArgumentException("HTTP method may not be null");
        } else if (state != null) {
            String host = null;
            if (conn != null) {
                if (proxy) {
                    host = conn.getProxyHost();
                } else {
                    host = method.getParams().getVirtualHost();
                    if (host == null) {
                        host = conn.getHost();
                    }
                }
            }
            String realm = authscheme.getRealm();
            if (LOG.isDebugEnabled()) {
                StringBuffer buffer = new StringBuffer();
                buffer.append("Using credentials for ");
                if (realm == null) {
                    buffer.append(CookiePolicy.DEFAULT);
                } else {
                    buffer.append('\'');
                    buffer.append(realm);
                    buffer.append('\'');
                }
                buffer.append(" authentication realm at ");
                buffer.append(host);
                LOG.debug(buffer.toString());
            }
            Credentials credentials = proxy ? state.getProxyCredentials(realm, host) : state.getCredentials(realm, host);
            if (credentials == null) {
                StringBuffer buffer2 = new StringBuffer();
                buffer2.append("No credentials available for the ");
                if (realm == null) {
                    buffer2.append(CookiePolicy.DEFAULT);
                } else {
                    buffer2.append('\'');
                    buffer2.append(realm);
                    buffer2.append('\'');
                }
                buffer2.append(" authentication realm at ");
                buffer2.append(host);
                throw new CredentialsNotAvailableException(buffer2.toString());
            }
            String auth = authscheme.authenticate(credentials, method);
            if (auth == null) {
                return false;
            }
            method.addRequestHeader(new Header(proxy ? "Proxy-Authorization" : "Authorization", auth, true));
            return true;
        } else {
            throw new IllegalArgumentException("HTTP state may not be null");
        }
    }

    public static boolean authenticate(AuthScheme authscheme, HttpMethod method, HttpConnection conn, HttpState state) throws AuthenticationException {
        LOG.trace("enter HttpAuthenticator.authenticate(AuthScheme, HttpMethod, HttpConnection, HttpState)");
        return doAuthenticate(authscheme, method, conn, state, false);
    }

    public static boolean authenticateProxy(AuthScheme authscheme, HttpMethod method, HttpConnection conn, HttpState state) throws AuthenticationException {
        LOG.trace("enter HttpAuthenticator.authenticateProxy(AuthScheme, HttpMethod, HttpState)");
        return doAuthenticate(authscheme, method, conn, state, true);
    }
}
