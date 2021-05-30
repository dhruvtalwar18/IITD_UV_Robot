package org.apache.commons.httpclient.cookie;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

public abstract class CookiePolicy {
    public static final String BROWSER_COMPATIBILITY = "compatibility";
    public static final int COMPATIBILITY = 0;
    public static final String DEFAULT = "default";
    public static final String IGNORE_COOKIES = "ignoreCookies";
    protected static final Log LOG;
    public static final String NETSCAPE = "netscape";
    public static final int NETSCAPE_DRAFT = 1;
    public static final int RFC2109 = 2;
    public static final int RFC2965 = 3;
    public static final String RFC_2109 = "rfc2109";
    public static final String RFC_2965 = "rfc2965";
    private static Map SPECS = Collections.synchronizedMap(new HashMap());
    static /* synthetic */ Class class$org$apache$commons$httpclient$cookie$CookiePolicy;
    static /* synthetic */ Class class$org$apache$commons$httpclient$cookie$CookieSpecBase;
    static /* synthetic */ Class class$org$apache$commons$httpclient$cookie$IgnoreCookiesSpec;
    static /* synthetic */ Class class$org$apache$commons$httpclient$cookie$NetscapeDraftSpec;
    static /* synthetic */ Class class$org$apache$commons$httpclient$cookie$RFC2109Spec;
    static /* synthetic */ Class class$org$apache$commons$httpclient$cookie$RFC2965Spec;
    private static int defaultPolicy = 2;

    static {
        Class cls;
        Class cls2;
        Class cls3;
        Class cls4;
        Class cls5;
        Class cls6;
        Class cls7;
        if (class$org$apache$commons$httpclient$cookie$RFC2109Spec == null) {
            cls = class$("org.apache.commons.httpclient.cookie.RFC2109Spec");
            class$org$apache$commons$httpclient$cookie$RFC2109Spec = cls;
        } else {
            cls = class$org$apache$commons$httpclient$cookie$RFC2109Spec;
        }
        registerCookieSpec(DEFAULT, cls);
        if (class$org$apache$commons$httpclient$cookie$RFC2109Spec == null) {
            cls2 = class$("org.apache.commons.httpclient.cookie.RFC2109Spec");
            class$org$apache$commons$httpclient$cookie$RFC2109Spec = cls2;
        } else {
            cls2 = class$org$apache$commons$httpclient$cookie$RFC2109Spec;
        }
        registerCookieSpec(RFC_2109, cls2);
        if (class$org$apache$commons$httpclient$cookie$RFC2965Spec == null) {
            cls3 = class$("org.apache.commons.httpclient.cookie.RFC2965Spec");
            class$org$apache$commons$httpclient$cookie$RFC2965Spec = cls3;
        } else {
            cls3 = class$org$apache$commons$httpclient$cookie$RFC2965Spec;
        }
        registerCookieSpec(RFC_2965, cls3);
        if (class$org$apache$commons$httpclient$cookie$CookieSpecBase == null) {
            cls4 = class$("org.apache.commons.httpclient.cookie.CookieSpecBase");
            class$org$apache$commons$httpclient$cookie$CookieSpecBase = cls4;
        } else {
            cls4 = class$org$apache$commons$httpclient$cookie$CookieSpecBase;
        }
        registerCookieSpec(BROWSER_COMPATIBILITY, cls4);
        if (class$org$apache$commons$httpclient$cookie$NetscapeDraftSpec == null) {
            cls5 = class$("org.apache.commons.httpclient.cookie.NetscapeDraftSpec");
            class$org$apache$commons$httpclient$cookie$NetscapeDraftSpec = cls5;
        } else {
            cls5 = class$org$apache$commons$httpclient$cookie$NetscapeDraftSpec;
        }
        registerCookieSpec(NETSCAPE, cls5);
        if (class$org$apache$commons$httpclient$cookie$IgnoreCookiesSpec == null) {
            cls6 = class$("org.apache.commons.httpclient.cookie.IgnoreCookiesSpec");
            class$org$apache$commons$httpclient$cookie$IgnoreCookiesSpec = cls6;
        } else {
            cls6 = class$org$apache$commons$httpclient$cookie$IgnoreCookiesSpec;
        }
        registerCookieSpec(IGNORE_COOKIES, cls6);
        if (class$org$apache$commons$httpclient$cookie$CookiePolicy == null) {
            cls7 = class$("org.apache.commons.httpclient.cookie.CookiePolicy");
            class$org$apache$commons$httpclient$cookie$CookiePolicy = cls7;
        } else {
            cls7 = class$org$apache$commons$httpclient$cookie$CookiePolicy;
        }
        LOG = LogFactory.getLog(cls7);
    }

    static /* synthetic */ Class class$(String x0) {
        try {
            return Class.forName(x0);
        } catch (ClassNotFoundException x1) {
            throw new NoClassDefFoundError(x1.getMessage());
        }
    }

    public static void registerCookieSpec(String id, Class clazz) {
        if (id == null) {
            throw new IllegalArgumentException("Id may not be null");
        } else if (clazz != null) {
            SPECS.put(id.toLowerCase(), clazz);
        } else {
            throw new IllegalArgumentException("Cookie spec class may not be null");
        }
    }

    public static void unregisterCookieSpec(String id) {
        if (id != null) {
            SPECS.remove(id.toLowerCase());
            return;
        }
        throw new IllegalArgumentException("Id may not be null");
    }

    public static CookieSpec getCookieSpec(String id) throws IllegalStateException {
        if (id != null) {
            Class clazz = (Class) SPECS.get(id.toLowerCase());
            if (clazz != null) {
                try {
                    return (CookieSpec) clazz.newInstance();
                } catch (Exception e) {
                    Log log = LOG;
                    StringBuffer stringBuffer = new StringBuffer();
                    stringBuffer.append("Error initializing cookie spec: ");
                    stringBuffer.append(id);
                    log.error(stringBuffer.toString(), e);
                    StringBuffer stringBuffer2 = new StringBuffer();
                    stringBuffer2.append(id);
                    stringBuffer2.append(" cookie spec implemented by ");
                    stringBuffer2.append(clazz.getName());
                    stringBuffer2.append(" could not be initialized");
                    throw new IllegalStateException(stringBuffer2.toString());
                }
            } else {
                StringBuffer stringBuffer3 = new StringBuffer();
                stringBuffer3.append("Unsupported cookie spec ");
                stringBuffer3.append(id);
                throw new IllegalStateException(stringBuffer3.toString());
            }
        } else {
            throw new IllegalArgumentException("Id may not be null");
        }
    }

    public static int getDefaultPolicy() {
        return defaultPolicy;
    }

    public static void setDefaultPolicy(int policy) {
        defaultPolicy = policy;
    }

    public static CookieSpec getSpecByPolicy(int policy) {
        switch (policy) {
            case 0:
                return new CookieSpecBase();
            case 1:
                return new NetscapeDraftSpec();
            case 2:
                return new RFC2109Spec();
            case 3:
                return new RFC2965Spec();
            default:
                return getDefaultSpec();
        }
    }

    public static CookieSpec getDefaultSpec() {
        try {
            return getCookieSpec(DEFAULT);
        } catch (IllegalStateException e) {
            LOG.warn("Default cookie policy is not registered");
            return new RFC2109Spec();
        }
    }

    public static CookieSpec getSpecByVersion(int ver) {
        switch (ver) {
            case 0:
                return new NetscapeDraftSpec();
            case 1:
                return new RFC2109Spec();
            default:
                return getDefaultSpec();
        }
    }

    public static CookieSpec getCompatibilitySpec() {
        return getSpecByPolicy(0);
    }

    public static String[] getRegisteredCookieSpecs() {
        return (String[]) SPECS.keySet().toArray(new String[SPECS.size()]);
    }
}
