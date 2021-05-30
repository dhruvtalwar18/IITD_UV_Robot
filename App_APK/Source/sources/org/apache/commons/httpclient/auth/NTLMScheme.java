package org.apache.commons.httpclient.auth;

import org.apache.commons.httpclient.Credentials;
import org.apache.commons.httpclient.HttpMethod;
import org.apache.commons.httpclient.NTCredentials;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

public class NTLMScheme implements AuthScheme {
    private static final int FAILED = Integer.MAX_VALUE;
    private static final int INITIATED = 1;
    private static final Log LOG;
    private static final int TYPE1_MSG_GENERATED = 2;
    private static final int TYPE2_MSG_RECEIVED = 3;
    private static final int TYPE3_MSG_GENERATED = 4;
    private static final int UNINITIATED = 0;
    static /* synthetic */ Class class$org$apache$commons$httpclient$auth$NTLMScheme;
    private String ntlmchallenge;
    private int state;

    static {
        Class cls;
        if (class$org$apache$commons$httpclient$auth$NTLMScheme == null) {
            cls = class$("org.apache.commons.httpclient.auth.NTLMScheme");
            class$org$apache$commons$httpclient$auth$NTLMScheme = cls;
        } else {
            cls = class$org$apache$commons$httpclient$auth$NTLMScheme;
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

    public NTLMScheme() {
        this.ntlmchallenge = null;
        this.state = 0;
    }

    public NTLMScheme(String challenge) throws MalformedChallengeException {
        this.ntlmchallenge = null;
        processChallenge(challenge);
    }

    public void processChallenge(String challenge) throws MalformedChallengeException {
        if (AuthChallengeParser.extractScheme(challenge).equalsIgnoreCase(getSchemeName())) {
            int i = challenge.indexOf(32);
            if (i != -1) {
                this.ntlmchallenge = challenge.substring(i, challenge.length()).trim();
                this.state = 3;
                return;
            }
            this.ntlmchallenge = "";
            if (this.state == 0) {
                this.state = 1;
            } else {
                this.state = Integer.MAX_VALUE;
            }
        } else {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Invalid NTLM challenge: ");
            stringBuffer.append(challenge);
            throw new MalformedChallengeException(stringBuffer.toString());
        }
    }

    public boolean isComplete() {
        return this.state == 4 || this.state == Integer.MAX_VALUE;
    }

    public String getSchemeName() {
        return "ntlm";
    }

    public String getRealm() {
        return null;
    }

    public String getID() {
        return this.ntlmchallenge;
    }

    public String getParameter(String name) {
        if (name != null) {
            return null;
        }
        throw new IllegalArgumentException("Parameter name may not be null");
    }

    public boolean isConnectionBased() {
        return true;
    }

    public static String authenticate(NTCredentials credentials, String challenge) throws AuthenticationException {
        LOG.trace("enter NTLMScheme.authenticate(NTCredentials, String)");
        if (credentials != null) {
            String s = new NTLM().getResponseFor(challenge, credentials.getUserName(), credentials.getPassword(), credentials.getHost(), credentials.getDomain());
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("NTLM ");
            stringBuffer.append(s);
            return stringBuffer.toString();
        }
        throw new IllegalArgumentException("Credentials may not be null");
    }

    public static String authenticate(NTCredentials credentials, String challenge, String charset) throws AuthenticationException {
        LOG.trace("enter NTLMScheme.authenticate(NTCredentials, String)");
        if (credentials != null) {
            NTLM ntlm = new NTLM();
            ntlm.setCredentialCharset(charset);
            String s = ntlm.getResponseFor(challenge, credentials.getUserName(), credentials.getPassword(), credentials.getHost(), credentials.getDomain());
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("NTLM ");
            stringBuffer.append(s);
            return stringBuffer.toString();
        }
        throw new IllegalArgumentException("Credentials may not be null");
    }

    public String authenticate(Credentials credentials, String method, String uri) throws AuthenticationException {
        LOG.trace("enter NTLMScheme.authenticate(Credentials, String, String)");
        try {
            return authenticate((NTCredentials) credentials, this.ntlmchallenge);
        } catch (ClassCastException e) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Credentials cannot be used for NTLM authentication: ");
            stringBuffer.append(credentials.getClass().getName());
            throw new InvalidCredentialsException(stringBuffer.toString());
        }
    }

    public String authenticate(Credentials credentials, HttpMethod method) throws AuthenticationException {
        String response;
        LOG.trace("enter NTLMScheme.authenticate(Credentials, HttpMethod)");
        if (this.state != 0) {
            try {
                NTCredentials ntcredentials = (NTCredentials) credentials;
                NTLM ntlm = new NTLM();
                ntlm.setCredentialCharset(method.getParams().getCredentialCharset());
                if (this.state == 1 || this.state == Integer.MAX_VALUE) {
                    response = ntlm.getType1Message(ntcredentials.getHost(), ntcredentials.getDomain());
                    this.state = 2;
                } else {
                    response = ntlm.getType3Message(ntcredentials.getUserName(), ntcredentials.getPassword(), ntcredentials.getHost(), ntcredentials.getDomain(), ntlm.parseType2Message(this.ntlmchallenge));
                    this.state = 4;
                }
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append("NTLM ");
                stringBuffer.append(response);
                return stringBuffer.toString();
            } catch (ClassCastException e) {
                StringBuffer stringBuffer2 = new StringBuffer();
                stringBuffer2.append("Credentials cannot be used for NTLM authentication: ");
                stringBuffer2.append(credentials.getClass().getName());
                throw new InvalidCredentialsException(stringBuffer2.toString());
            }
        } else {
            throw new IllegalStateException("NTLM authentication process has not been initiated");
        }
    }
}
