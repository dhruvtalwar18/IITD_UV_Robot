package org.apache.commons.httpclient.auth;

public class AuthState {
    public static final String PREEMPTIVE_AUTH_SCHEME = "basic";
    private boolean authAttempted = false;
    private boolean authRequested = false;
    private AuthScheme authScheme = null;
    private boolean preemptive = false;

    public void invalidate() {
        this.authScheme = null;
        this.authRequested = false;
        this.authAttempted = false;
        this.preemptive = false;
    }

    public boolean isAuthRequested() {
        return this.authRequested;
    }

    public void setAuthRequested(boolean challengeReceived) {
        this.authRequested = challengeReceived;
    }

    public boolean isAuthAttempted() {
        return this.authAttempted;
    }

    public void setAuthAttempted(boolean challengeResponded) {
        this.authAttempted = challengeResponded;
    }

    public void setPreemptive() {
        if (this.preemptive) {
            return;
        }
        if (this.authScheme == null) {
            this.authScheme = AuthPolicy.getAuthScheme(PREEMPTIVE_AUTH_SCHEME);
            this.preemptive = true;
            return;
        }
        throw new IllegalStateException("Authentication state already initialized");
    }

    public boolean isPreemptive() {
        return this.preemptive;
    }

    public void setAuthScheme(AuthScheme authScheme2) {
        if (authScheme2 == null) {
            invalidate();
            return;
        }
        if (this.preemptive && !this.authScheme.getClass().isInstance(authScheme2)) {
            this.preemptive = false;
            this.authAttempted = false;
        }
        this.authScheme = authScheme2;
    }

    public AuthScheme getAuthScheme() {
        return this.authScheme;
    }

    public String getRealm() {
        if (this.authScheme != null) {
            return this.authScheme.getRealm();
        }
        return null;
    }

    public String toString() {
        StringBuffer buffer = new StringBuffer();
        buffer.append("Auth state: auth requested [");
        buffer.append(this.authRequested);
        buffer.append("]; auth attempted [");
        buffer.append(this.authAttempted);
        if (this.authScheme != null) {
            buffer.append("]; auth scheme [");
            buffer.append(this.authScheme.getSchemeName());
            buffer.append("]; realm [");
            buffer.append(this.authScheme.getRealm());
        }
        buffer.append("] preemptive [");
        buffer.append(this.preemptive);
        buffer.append("]");
        return buffer.toString();
    }
}
