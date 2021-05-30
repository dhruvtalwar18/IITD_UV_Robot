package org.apache.commons.httpclient.auth;

public abstract class AuthSchemeBase implements AuthScheme {
    private String challenge = null;

    public AuthSchemeBase(String challenge2) throws MalformedChallengeException {
        if (challenge2 != null) {
            this.challenge = challenge2;
            return;
        }
        throw new IllegalArgumentException("Challenge may not be null");
    }

    public boolean equals(Object obj) {
        if (obj instanceof AuthSchemeBase) {
            return this.challenge.equals(((AuthSchemeBase) obj).challenge);
        }
        return super.equals(obj);
    }

    public int hashCode() {
        return this.challenge.hashCode();
    }

    public String toString() {
        return this.challenge;
    }
}
