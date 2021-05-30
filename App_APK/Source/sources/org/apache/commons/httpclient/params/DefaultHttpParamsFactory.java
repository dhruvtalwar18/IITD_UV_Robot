package org.apache.commons.httpclient.params;

import java.util.ArrayList;
import java.util.Arrays;
import org.apache.commons.httpclient.DefaultHttpMethodRetryHandler;
import org.apache.commons.httpclient.HttpState;
import org.apache.commons.httpclient.HttpVersion;
import org.apache.commons.httpclient.cookie.CookiePolicy;

public class DefaultHttpParamsFactory implements HttpParamsFactory {
    static /* synthetic */ Class class$org$apache$commons$httpclient$SimpleHttpConnectionManager;
    private HttpParams httpParams;

    public synchronized HttpParams getDefaultParams() {
        if (this.httpParams == null) {
            this.httpParams = createParams();
        }
        return this.httpParams;
    }

    /* access modifiers changed from: protected */
    public HttpParams createParams() {
        Class cls;
        String defaultCookiePolicy = null;
        HttpClientParams params = new HttpClientParams((HttpParams) null);
        params.setParameter(HttpMethodParams.USER_AGENT, "Jakarta Commons-HttpClient/3.1");
        params.setVersion(HttpVersion.HTTP_1_1);
        if (class$org$apache$commons$httpclient$SimpleHttpConnectionManager == null) {
            cls = class$("org.apache.commons.httpclient.SimpleHttpConnectionManager");
            class$org$apache$commons$httpclient$SimpleHttpConnectionManager = cls;
        } else {
            cls = class$org$apache$commons$httpclient$SimpleHttpConnectionManager;
        }
        params.setConnectionManagerClass(cls);
        params.setCookiePolicy(CookiePolicy.DEFAULT);
        params.setHttpElementCharset("US-ASCII");
        params.setContentCharset("ISO-8859-1");
        params.setParameter(HttpMethodParams.RETRY_HANDLER, new DefaultHttpMethodRetryHandler());
        ArrayList datePatterns = new ArrayList();
        datePatterns.addAll(Arrays.asList(new String[]{"EEE, dd MMM yyyy HH:mm:ss zzz", "EEEE, dd-MMM-yy HH:mm:ss zzz", "EEE MMM d HH:mm:ss yyyy", "EEE, dd-MMM-yyyy HH:mm:ss z", "EEE, dd-MMM-yyyy HH-mm-ss z", "EEE, dd MMM yy HH:mm:ss z", "EEE dd-MMM-yyyy HH:mm:ss z", "EEE dd MMM yyyy HH:mm:ss z", "EEE dd-MMM-yyyy HH-mm-ss z", "EEE dd-MMM-yy HH:mm:ss z", "EEE dd MMM yy HH:mm:ss z", "EEE,dd-MMM-yy HH:mm:ss z", "EEE,dd-MMM-yyyy HH:mm:ss z", "EEE, dd-MM-yyyy HH:mm:ss z"}));
        params.setParameter(HttpMethodParams.DATE_PATTERNS, datePatterns);
        String agent = null;
        try {
            agent = System.getProperty("httpclient.useragent");
        } catch (SecurityException e) {
        }
        if (agent != null) {
            params.setParameter(HttpMethodParams.USER_AGENT, agent);
        }
        String preemptiveDefault = null;
        try {
            preemptiveDefault = System.getProperty(HttpState.PREEMPTIVE_PROPERTY);
        } catch (SecurityException e2) {
        }
        if (preemptiveDefault != null) {
            String preemptiveDefault2 = preemptiveDefault.trim().toLowerCase();
            if (preemptiveDefault2.equals("true")) {
                params.setParameter(HttpClientParams.PREEMPTIVE_AUTHENTICATION, Boolean.TRUE);
            } else if (preemptiveDefault2.equals(HttpState.PREEMPTIVE_DEFAULT)) {
                params.setParameter(HttpClientParams.PREEMPTIVE_AUTHENTICATION, Boolean.FALSE);
            }
        }
        try {
            defaultCookiePolicy = System.getProperty("apache.commons.httpclient.cookiespec");
        } catch (SecurityException e3) {
        }
        if (defaultCookiePolicy != null) {
            if ("COMPATIBILITY".equalsIgnoreCase(defaultCookiePolicy)) {
                params.setCookiePolicy(CookiePolicy.BROWSER_COMPATIBILITY);
            } else if ("NETSCAPE_DRAFT".equalsIgnoreCase(defaultCookiePolicy)) {
                params.setCookiePolicy(CookiePolicy.NETSCAPE);
            } else if ("RFC2109".equalsIgnoreCase(defaultCookiePolicy)) {
                params.setCookiePolicy(CookiePolicy.RFC_2109);
            }
        }
        return params;
    }

    static /* synthetic */ Class class$(String x0) {
        try {
            return Class.forName(x0);
        } catch (ClassNotFoundException x1) {
            throw new NoClassDefFoundError(x1.getMessage());
        }
    }
}
