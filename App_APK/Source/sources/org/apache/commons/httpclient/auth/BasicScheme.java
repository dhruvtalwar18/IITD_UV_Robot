package org.apache.commons.httpclient.auth;

import org.apache.commons.codec.binary.Base64;
import org.apache.commons.httpclient.Credentials;
import org.apache.commons.httpclient.HttpMethod;
import org.apache.commons.httpclient.UsernamePasswordCredentials;
import org.apache.commons.httpclient.util.EncodingUtil;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

public class BasicScheme extends RFC2617Scheme {
    private static final Log LOG;
    static /* synthetic */ Class class$org$apache$commons$httpclient$auth$BasicScheme;
    private boolean complete = false;

    static {
        Class cls;
        if (class$org$apache$commons$httpclient$auth$BasicScheme == null) {
            cls = class$("org.apache.commons.httpclient.auth.BasicScheme");
            class$org$apache$commons$httpclient$auth$BasicScheme = cls;
        } else {
            cls = class$org$apache$commons$httpclient$auth$BasicScheme;
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

    public BasicScheme() {
    }

    public BasicScheme(String challenge) throws MalformedChallengeException {
        super(challenge);
    }

    public String getSchemeName() {
        return AuthState.PREEMPTIVE_AUTH_SCHEME;
    }

    public void processChallenge(String challenge) throws MalformedChallengeException {
        super.processChallenge(challenge);
        this.complete = true;
    }

    public boolean isComplete() {
        return this.complete;
    }

    public String authenticate(Credentials credentials, String method, String uri) throws AuthenticationException {
        LOG.trace("enter BasicScheme.authenticate(Credentials, String, String)");
        try {
            return authenticate((UsernamePasswordCredentials) credentials);
        } catch (ClassCastException e) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Credentials cannot be used for basic authentication: ");
            stringBuffer.append(credentials.getClass().getName());
            throw new InvalidCredentialsException(stringBuffer.toString());
        }
    }

    public boolean isConnectionBased() {
        return false;
    }

    public String authenticate(Credentials credentials, HttpMethod method) throws AuthenticationException {
        LOG.trace("enter BasicScheme.authenticate(Credentials, HttpMethod)");
        if (method != null) {
            try {
                return authenticate((UsernamePasswordCredentials) credentials, method.getParams().getCredentialCharset());
            } catch (ClassCastException e) {
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append("Credentials cannot be used for basic authentication: ");
                stringBuffer.append(credentials.getClass().getName());
                throw new InvalidCredentialsException(stringBuffer.toString());
            }
        } else {
            throw new IllegalArgumentException("Method may not be null");
        }
    }

    public static String authenticate(UsernamePasswordCredentials credentials) {
        return authenticate(credentials, "ISO-8859-1");
    }

    public static String authenticate(UsernamePasswordCredentials credentials, String charset) {
        LOG.trace("enter BasicScheme.authenticate(UsernamePasswordCredentials, String)");
        if (credentials == null) {
            throw new IllegalArgumentException("Credentials may not be null");
        } else if (charset == null || charset.length() == 0) {
            throw new IllegalArgumentException("charset may not be null or empty");
        } else {
            StringBuffer buffer = new StringBuffer();
            buffer.append(credentials.getUserName());
            buffer.append(":");
            buffer.append(credentials.getPassword());
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Basic ");
            stringBuffer.append(EncodingUtil.getAsciiString(Base64.encodeBase64(EncodingUtil.getBytes(buffer.toString(), charset))));
            return stringBuffer.toString();
        }
    }
}
