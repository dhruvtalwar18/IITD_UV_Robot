package org.apache.commons.httpclient;

import java.util.ArrayList;
import java.util.Date;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import org.apache.commons.httpclient.auth.AuthScope;
import org.apache.commons.httpclient.cookie.CookiePolicy;
import org.apache.commons.httpclient.cookie.CookieSpec;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

public class HttpState {
    private static final Log LOG;
    public static final String PREEMPTIVE_DEFAULT = "false";
    public static final String PREEMPTIVE_PROPERTY = "httpclient.authentication.preemptive";
    static /* synthetic */ Class class$org$apache$commons$httpclient$HttpState;
    private int cookiePolicy = -1;
    protected ArrayList cookies = new ArrayList();
    protected HashMap credMap = new HashMap();
    private boolean preemptive = false;
    protected HashMap proxyCred = new HashMap();

    static {
        Class cls;
        if (class$org$apache$commons$httpclient$HttpState == null) {
            cls = class$("org.apache.commons.httpclient.HttpState");
            class$org$apache$commons$httpclient$HttpState = cls;
        } else {
            cls = class$org$apache$commons$httpclient$HttpState;
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

    public synchronized void addCookie(Cookie cookie) {
        LOG.trace("enter HttpState.addCookie(Cookie)");
        if (cookie != null) {
            Iterator it = this.cookies.iterator();
            while (true) {
                if (!it.hasNext()) {
                    break;
                } else if (cookie.equals((Cookie) it.next())) {
                    it.remove();
                    break;
                }
            }
            if (!cookie.isExpired()) {
                this.cookies.add(cookie);
            }
        }
    }

    public synchronized void addCookies(Cookie[] cookies2) {
        LOG.trace("enter HttpState.addCookies(Cookie[])");
        if (cookies2 != null) {
            for (Cookie addCookie : cookies2) {
                addCookie(addCookie);
            }
        }
    }

    public synchronized Cookie[] getCookies() {
        LOG.trace("enter HttpState.getCookies()");
        return (Cookie[]) this.cookies.toArray(new Cookie[this.cookies.size()]);
    }

    public synchronized Cookie[] getCookies(String domain, int port, String path, boolean secure) {
        ArrayList list;
        LOG.trace("enter HttpState.getCookies(String, int, String, boolean)");
        CookieSpec matcher = CookiePolicy.getDefaultSpec();
        list = new ArrayList(this.cookies.size());
        int i = 0;
        int m = this.cookies.size();
        while (true) {
            int m2 = m;
            if (i < m2) {
                Cookie cookie = (Cookie) this.cookies.get(i);
                if (matcher.match(domain, port, path, secure, cookie)) {
                    list.add(cookie);
                }
                i++;
                m = m2;
            }
        }
        return (Cookie[]) list.toArray(new Cookie[list.size()]);
    }

    public synchronized boolean purgeExpiredCookies() {
        LOG.trace("enter HttpState.purgeExpiredCookies()");
        return purgeExpiredCookies(new Date());
    }

    public synchronized boolean purgeExpiredCookies(Date date) {
        boolean removed;
        LOG.trace("enter HttpState.purgeExpiredCookies(Date)");
        removed = false;
        Iterator it = this.cookies.iterator();
        while (it.hasNext()) {
            if (((Cookie) it.next()).isExpired(date)) {
                it.remove();
                removed = true;
            }
        }
        return removed;
    }

    public int getCookiePolicy() {
        return this.cookiePolicy;
    }

    public void setAuthenticationPreemptive(boolean value) {
        this.preemptive = value;
    }

    public boolean isAuthenticationPreemptive() {
        return this.preemptive;
    }

    public void setCookiePolicy(int policy) {
        this.cookiePolicy = policy;
    }

    public synchronized void setCredentials(String realm, String host, Credentials credentials) {
        LOG.trace("enter HttpState.setCredentials(String, String, Credentials)");
        this.credMap.put(new AuthScope(host, -1, realm, AuthScope.ANY_SCHEME), credentials);
    }

    public synchronized void setCredentials(AuthScope authscope, Credentials credentials) {
        if (authscope != null) {
            LOG.trace("enter HttpState.setCredentials(AuthScope, Credentials)");
            this.credMap.put(authscope, credentials);
        } else {
            throw new IllegalArgumentException("Authentication scope may not be null");
        }
    }

    private static Credentials matchCredentials(HashMap map, AuthScope authscope) {
        Credentials creds = (Credentials) map.get(authscope);
        if (creds != null) {
            return creds;
        }
        int bestMatchFactor = -1;
        AuthScope bestMatch = null;
        for (AuthScope current : map.keySet()) {
            int factor = authscope.match(current);
            if (factor > bestMatchFactor) {
                bestMatchFactor = factor;
                bestMatch = current;
            }
        }
        if (bestMatch != null) {
            return (Credentials) map.get(bestMatch);
        }
        return creds;
    }

    public synchronized Credentials getCredentials(String realm, String host) {
        LOG.trace("enter HttpState.getCredentials(String, String");
        return matchCredentials(this.credMap, new AuthScope(host, -1, realm, AuthScope.ANY_SCHEME));
    }

    public synchronized Credentials getCredentials(AuthScope authscope) {
        if (authscope != null) {
            LOG.trace("enter HttpState.getCredentials(AuthScope)");
        } else {
            throw new IllegalArgumentException("Authentication scope may not be null");
        }
        return matchCredentials(this.credMap, authscope);
    }

    public synchronized void setProxyCredentials(String realm, String proxyHost, Credentials credentials) {
        LOG.trace("enter HttpState.setProxyCredentials(String, String, Credentials");
        this.proxyCred.put(new AuthScope(proxyHost, -1, realm, AuthScope.ANY_SCHEME), credentials);
    }

    public synchronized void setProxyCredentials(AuthScope authscope, Credentials credentials) {
        if (authscope != null) {
            LOG.trace("enter HttpState.setProxyCredentials(AuthScope, Credentials)");
            this.proxyCred.put(authscope, credentials);
        } else {
            throw new IllegalArgumentException("Authentication scope may not be null");
        }
    }

    public synchronized Credentials getProxyCredentials(String realm, String proxyHost) {
        LOG.trace("enter HttpState.getCredentials(String, String");
        return matchCredentials(this.proxyCred, new AuthScope(proxyHost, -1, realm, AuthScope.ANY_SCHEME));
    }

    public synchronized Credentials getProxyCredentials(AuthScope authscope) {
        if (authscope != null) {
            LOG.trace("enter HttpState.getProxyCredentials(AuthScope)");
        } else {
            throw new IllegalArgumentException("Authentication scope may not be null");
        }
        return matchCredentials(this.proxyCred, authscope);
    }

    public synchronized String toString() {
        StringBuffer sbResult;
        sbResult = new StringBuffer();
        sbResult.append("[");
        sbResult.append(getCredentialsStringRepresentation(this.proxyCred));
        sbResult.append(" | ");
        sbResult.append(getCredentialsStringRepresentation(this.credMap));
        sbResult.append(" | ");
        sbResult.append(getCookiesStringRepresentation(this.cookies));
        sbResult.append("]");
        return sbResult.toString();
    }

    private static String getCredentialsStringRepresentation(Map credMap2) {
        StringBuffer sbResult = new StringBuffer();
        for (Object key : credMap2.keySet()) {
            Credentials cred = (Credentials) credMap2.get(key);
            if (sbResult.length() > 0) {
                sbResult.append(", ");
            }
            sbResult.append(key);
            sbResult.append("#");
            sbResult.append(cred.toString());
        }
        return sbResult.toString();
    }

    private static String getCookiesStringRepresentation(List cookies2) {
        StringBuffer sbResult = new StringBuffer();
        Iterator iter = cookies2.iterator();
        while (iter.hasNext()) {
            Cookie ck = (Cookie) iter.next();
            if (sbResult.length() > 0) {
                sbResult.append("#");
            }
            sbResult.append(ck.toExternalForm());
        }
        return sbResult.toString();
    }

    public void clearCredentials() {
        this.credMap.clear();
    }

    public void clearProxyCredentials() {
        this.proxyCred.clear();
    }

    public synchronized void clearCookies() {
        this.cookies.clear();
    }

    public void clear() {
        clearCookies();
        clearCredentials();
        clearProxyCredentials();
    }
}
