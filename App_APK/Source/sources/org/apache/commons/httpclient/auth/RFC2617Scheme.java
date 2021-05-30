package org.apache.commons.httpclient.auth;

import java.util.Map;

public abstract class RFC2617Scheme implements AuthScheme {
    private Map params = null;

    public RFC2617Scheme() {
    }

    public RFC2617Scheme(String challenge) throws MalformedChallengeException {
        processChallenge(challenge);
    }

    public void processChallenge(String challenge) throws MalformedChallengeException {
        if (AuthChallengeParser.extractScheme(challenge).equalsIgnoreCase(getSchemeName())) {
            this.params = AuthChallengeParser.extractParams(challenge);
            return;
        }
        StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append("Invalid ");
        stringBuffer.append(getSchemeName());
        stringBuffer.append(" challenge: ");
        stringBuffer.append(challenge);
        throw new MalformedChallengeException(stringBuffer.toString());
    }

    /* access modifiers changed from: protected */
    public Map getParameters() {
        return this.params;
    }

    public String getParameter(String name) {
        if (name == null) {
            throw new IllegalArgumentException("Parameter name may not be null");
        } else if (this.params == null) {
            return null;
        } else {
            return (String) this.params.get(name.toLowerCase());
        }
    }

    public String getRealm() {
        return getParameter("realm");
    }

    public String getID() {
        return getRealm();
    }
}
