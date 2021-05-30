package org.apache.commons.httpclient.auth;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

public abstract class AuthPolicy {
    public static final String AUTH_SCHEME_PRIORITY = "http.auth.scheme-priority";
    public static final String BASIC = "Basic";
    public static final String DIGEST = "Digest";
    protected static final Log LOG;
    public static final String NTLM = "NTLM";
    private static final HashMap SCHEMES = new HashMap();
    private static final ArrayList SCHEME_LIST = new ArrayList();
    static /* synthetic */ Class class$org$apache$commons$httpclient$auth$AuthPolicy;
    static /* synthetic */ Class class$org$apache$commons$httpclient$auth$BasicScheme;
    static /* synthetic */ Class class$org$apache$commons$httpclient$auth$DigestScheme;
    static /* synthetic */ Class class$org$apache$commons$httpclient$auth$NTLMScheme;

    static {
        Class cls;
        Class cls2;
        Class cls3;
        Class cls4;
        if (class$org$apache$commons$httpclient$auth$NTLMScheme == null) {
            cls = class$("org.apache.commons.httpclient.auth.NTLMScheme");
            class$org$apache$commons$httpclient$auth$NTLMScheme = cls;
        } else {
            cls = class$org$apache$commons$httpclient$auth$NTLMScheme;
        }
        registerAuthScheme(NTLM, cls);
        if (class$org$apache$commons$httpclient$auth$DigestScheme == null) {
            cls2 = class$("org.apache.commons.httpclient.auth.DigestScheme");
            class$org$apache$commons$httpclient$auth$DigestScheme = cls2;
        } else {
            cls2 = class$org$apache$commons$httpclient$auth$DigestScheme;
        }
        registerAuthScheme(DIGEST, cls2);
        if (class$org$apache$commons$httpclient$auth$BasicScheme == null) {
            cls3 = class$("org.apache.commons.httpclient.auth.BasicScheme");
            class$org$apache$commons$httpclient$auth$BasicScheme = cls3;
        } else {
            cls3 = class$org$apache$commons$httpclient$auth$BasicScheme;
        }
        registerAuthScheme(BASIC, cls3);
        if (class$org$apache$commons$httpclient$auth$AuthPolicy == null) {
            cls4 = class$("org.apache.commons.httpclient.auth.AuthPolicy");
            class$org$apache$commons$httpclient$auth$AuthPolicy = cls4;
        } else {
            cls4 = class$org$apache$commons$httpclient$auth$AuthPolicy;
        }
        LOG = LogFactory.getLog(cls4);
    }

    static /* synthetic */ Class class$(String x0) {
        try {
            return Class.forName(x0);
        } catch (ClassNotFoundException x1) {
            throw new NoClassDefFoundError(x1.getMessage());
        }
    }

    public static synchronized void registerAuthScheme(String id, Class clazz) {
        synchronized (AuthPolicy.class) {
            if (id == null) {
                throw new IllegalArgumentException("Id may not be null");
            } else if (clazz != null) {
                SCHEMES.put(id.toLowerCase(), clazz);
                SCHEME_LIST.add(id.toLowerCase());
            } else {
                throw new IllegalArgumentException("Authentication scheme class may not be null");
            }
        }
    }

    public static synchronized void unregisterAuthScheme(String id) {
        synchronized (AuthPolicy.class) {
            if (id != null) {
                SCHEMES.remove(id.toLowerCase());
                SCHEME_LIST.remove(id.toLowerCase());
            } else {
                throw new IllegalArgumentException("Id may not be null");
            }
        }
    }

    public static synchronized AuthScheme getAuthScheme(String id) throws IllegalStateException {
        AuthScheme authScheme;
        synchronized (AuthPolicy.class) {
            if (id != null) {
                Class clazz = (Class) SCHEMES.get(id.toLowerCase());
                if (clazz != null) {
                    try {
                        authScheme = (AuthScheme) clazz.newInstance();
                    } catch (Exception e) {
                        Log log = LOG;
                        StringBuffer stringBuffer = new StringBuffer();
                        stringBuffer.append("Error initializing authentication scheme: ");
                        stringBuffer.append(id);
                        log.error(stringBuffer.toString(), e);
                        StringBuffer stringBuffer2 = new StringBuffer();
                        stringBuffer2.append(id);
                        stringBuffer2.append(" authentication scheme implemented by ");
                        stringBuffer2.append(clazz.getName());
                        stringBuffer2.append(" could not be initialized");
                        throw new IllegalStateException(stringBuffer2.toString());
                    }
                } else {
                    StringBuffer stringBuffer3 = new StringBuffer();
                    stringBuffer3.append("Unsupported authentication scheme ");
                    stringBuffer3.append(id);
                    throw new IllegalStateException(stringBuffer3.toString());
                }
            } else {
                throw new IllegalArgumentException("Id may not be null");
            }
        }
        return authScheme;
    }

    public static synchronized List getDefaultAuthPrefs() {
        List list;
        synchronized (AuthPolicy.class) {
            list = (List) SCHEME_LIST.clone();
        }
        return list;
    }
}
