package org.apache.commons.httpclient.auth;

import java.util.Collection;
import java.util.Iterator;
import java.util.Map;
import org.apache.commons.httpclient.params.HttpParams;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

public final class AuthChallengeProcessor {
    private static final Log LOG;
    static /* synthetic */ Class class$org$apache$commons$httpclient$auth$AuthChallengeProcessor;
    private HttpParams params = null;

    static {
        Class cls;
        if (class$org$apache$commons$httpclient$auth$AuthChallengeProcessor == null) {
            cls = class$("org.apache.commons.httpclient.auth.AuthChallengeProcessor");
            class$org$apache$commons$httpclient$auth$AuthChallengeProcessor = cls;
        } else {
            cls = class$org$apache$commons$httpclient$auth$AuthChallengeProcessor;
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

    public AuthChallengeProcessor(HttpParams params2) {
        if (params2 != null) {
            this.params = params2;
            return;
        }
        throw new IllegalArgumentException("Parameter collection may not be null");
    }

    public AuthScheme selectAuthScheme(Map challenges) throws AuthChallengeException {
        if (challenges != null) {
            Collection authPrefs = (Collection) this.params.getParameter(AuthPolicy.AUTH_SCHEME_PRIORITY);
            if (authPrefs == null || authPrefs.isEmpty()) {
                authPrefs = AuthPolicy.getDefaultAuthPrefs();
            }
            if (LOG.isDebugEnabled()) {
                Log log = LOG;
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append("Supported authentication schemes in the order of preference: ");
                stringBuffer.append(authPrefs);
                log.debug(stringBuffer.toString());
            }
            AuthScheme authscheme = null;
            Iterator item = authPrefs.iterator();
            while (true) {
                if (!item.hasNext()) {
                    break;
                }
                String id = (String) item.next();
                if (((String) challenges.get(id.toLowerCase())) != null) {
                    if (LOG.isInfoEnabled()) {
                        Log log2 = LOG;
                        StringBuffer stringBuffer2 = new StringBuffer();
                        stringBuffer2.append(id);
                        stringBuffer2.append(" authentication scheme selected");
                        log2.info(stringBuffer2.toString());
                    }
                    try {
                        authscheme = AuthPolicy.getAuthScheme(id);
                    } catch (IllegalStateException e) {
                        throw new AuthChallengeException(e.getMessage());
                    }
                } else if (LOG.isDebugEnabled()) {
                    Log log3 = LOG;
                    StringBuffer stringBuffer3 = new StringBuffer();
                    stringBuffer3.append("Challenge for ");
                    stringBuffer3.append(id);
                    stringBuffer3.append(" authentication scheme not available");
                    log3.debug(stringBuffer3.toString());
                }
            }
            if (authscheme != null) {
                return authscheme;
            }
            StringBuffer stringBuffer4 = new StringBuffer();
            stringBuffer4.append("Unable to respond to any of these challenges: ");
            stringBuffer4.append(challenges);
            throw new AuthChallengeException(stringBuffer4.toString());
        }
        throw new IllegalArgumentException("Challenge map may not be null");
    }

    public AuthScheme processChallenge(AuthState state, Map challenges) throws MalformedChallengeException, AuthenticationException {
        if (state == null) {
            throw new IllegalArgumentException("Authentication state may not be null");
        } else if (challenges != null) {
            if (state.isPreemptive() || state.getAuthScheme() == null) {
                state.setAuthScheme(selectAuthScheme(challenges));
            }
            AuthScheme authscheme = state.getAuthScheme();
            String id = authscheme.getSchemeName();
            if (LOG.isDebugEnabled()) {
                Log log = LOG;
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append("Using authentication scheme: ");
                stringBuffer.append(id);
                log.debug(stringBuffer.toString());
            }
            String challenge = (String) challenges.get(id.toLowerCase());
            if (challenge != null) {
                authscheme.processChallenge(challenge);
                LOG.debug("Authorization challenge processed");
                return authscheme;
            }
            StringBuffer stringBuffer2 = new StringBuffer();
            stringBuffer2.append(id);
            stringBuffer2.append(" authorization challenge expected, but not found");
            throw new AuthenticationException(stringBuffer2.toString());
        } else {
            throw new IllegalArgumentException("Challenge map may not be null");
        }
    }
}
