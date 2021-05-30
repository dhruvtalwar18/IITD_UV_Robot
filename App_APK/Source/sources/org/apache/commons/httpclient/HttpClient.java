package org.apache.commons.httpclient;

import java.io.IOException;
import java.security.Provider;
import java.security.Security;
import org.apache.commons.httpclient.params.HttpClientParams;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

public class HttpClient {
    private static final Log LOG;
    static /* synthetic */ Class class$org$apache$commons$httpclient$HttpClient;
    private HostConfiguration hostConfiguration;
    private HttpConnectionManager httpConnectionManager;
    private HttpClientParams params;
    private HttpState state;

    static {
        Class cls;
        if (class$org$apache$commons$httpclient$HttpClient == null) {
            cls = class$("org.apache.commons.httpclient.HttpClient");
            class$org$apache$commons$httpclient$HttpClient = cls;
        } else {
            cls = class$org$apache$commons$httpclient$HttpClient;
        }
        LOG = LogFactory.getLog(cls);
        if (LOG.isDebugEnabled()) {
            try {
                Log log = LOG;
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append("Java version: ");
                stringBuffer.append(System.getProperty("java.version"));
                log.debug(stringBuffer.toString());
                Log log2 = LOG;
                StringBuffer stringBuffer2 = new StringBuffer();
                stringBuffer2.append("Java vendor: ");
                stringBuffer2.append(System.getProperty("java.vendor"));
                log2.debug(stringBuffer2.toString());
                Log log3 = LOG;
                StringBuffer stringBuffer3 = new StringBuffer();
                stringBuffer3.append("Java class path: ");
                stringBuffer3.append(System.getProperty("java.class.path"));
                log3.debug(stringBuffer3.toString());
                Log log4 = LOG;
                StringBuffer stringBuffer4 = new StringBuffer();
                stringBuffer4.append("Operating system name: ");
                stringBuffer4.append(System.getProperty("os.name"));
                log4.debug(stringBuffer4.toString());
                Log log5 = LOG;
                StringBuffer stringBuffer5 = new StringBuffer();
                stringBuffer5.append("Operating system architecture: ");
                stringBuffer5.append(System.getProperty("os.arch"));
                log5.debug(stringBuffer5.toString());
                Log log6 = LOG;
                StringBuffer stringBuffer6 = new StringBuffer();
                stringBuffer6.append("Operating system version: ");
                stringBuffer6.append(System.getProperty("os.version"));
                log6.debug(stringBuffer6.toString());
                Provider[] providers = Security.getProviders();
                for (Provider provider : providers) {
                    Log log7 = LOG;
                    StringBuffer stringBuffer7 = new StringBuffer();
                    stringBuffer7.append(provider.getName());
                    stringBuffer7.append(" ");
                    stringBuffer7.append(provider.getVersion());
                    stringBuffer7.append(": ");
                    stringBuffer7.append(provider.getInfo());
                    log7.debug(stringBuffer7.toString());
                }
            } catch (SecurityException e) {
            }
        }
    }

    static /* synthetic */ Class class$(String x0) {
        try {
            return Class.forName(x0);
        } catch (ClassNotFoundException x1) {
            throw new NoClassDefFoundError(x1.getMessage());
        }
    }

    public HttpClient() {
        this(new HttpClientParams());
    }

    public HttpClient(HttpClientParams params2) {
        this.state = new HttpState();
        this.params = null;
        this.hostConfiguration = new HostConfiguration();
        if (params2 != null) {
            this.params = params2;
            this.httpConnectionManager = null;
            Class clazz = params2.getConnectionManagerClass();
            if (clazz != null) {
                try {
                    this.httpConnectionManager = (HttpConnectionManager) clazz.newInstance();
                } catch (Exception e) {
                    LOG.warn("Error instantiating connection manager class, defaulting to SimpleHttpConnectionManager", e);
                }
            }
            if (this.httpConnectionManager == null) {
                this.httpConnectionManager = new SimpleHttpConnectionManager();
            }
            if (this.httpConnectionManager != null) {
                this.httpConnectionManager.getParams().setDefaults(this.params);
                return;
            }
            return;
        }
        throw new IllegalArgumentException("Params may not be null");
    }

    public HttpClient(HttpClientParams params2, HttpConnectionManager httpConnectionManager2) {
        this.state = new HttpState();
        this.params = null;
        this.hostConfiguration = new HostConfiguration();
        if (httpConnectionManager2 == null) {
            throw new IllegalArgumentException("httpConnectionManager cannot be null");
        } else if (params2 != null) {
            this.params = params2;
            this.httpConnectionManager = httpConnectionManager2;
            this.httpConnectionManager.getParams().setDefaults(this.params);
        } else {
            throw new IllegalArgumentException("Params may not be null");
        }
    }

    public HttpClient(HttpConnectionManager httpConnectionManager2) {
        this(new HttpClientParams(), httpConnectionManager2);
    }

    public synchronized HttpState getState() {
        return this.state;
    }

    public synchronized void setState(HttpState state2) {
        this.state = state2;
    }

    public synchronized void setStrictMode(boolean strictMode) {
        if (strictMode) {
            try {
                this.params.makeStrict();
            } catch (Throwable th) {
                throw th;
            }
        } else {
            this.params.makeLenient();
        }
    }

    public synchronized boolean isStrictMode() {
        return false;
    }

    public synchronized void setTimeout(int newTimeoutInMilliseconds) {
        this.params.setSoTimeout(newTimeoutInMilliseconds);
    }

    public synchronized void setHttpConnectionFactoryTimeout(long timeout) {
        this.params.setConnectionManagerTimeout(timeout);
    }

    public synchronized void setConnectionTimeout(int newTimeoutInMilliseconds) {
        this.httpConnectionManager.getParams().setConnectionTimeout(newTimeoutInMilliseconds);
    }

    public int executeMethod(HttpMethod method) throws IOException, HttpException {
        LOG.trace("enter HttpClient.executeMethod(HttpMethod)");
        return executeMethod((HostConfiguration) null, method, (HttpState) null);
    }

    public int executeMethod(HostConfiguration hostConfiguration2, HttpMethod method) throws IOException, HttpException {
        LOG.trace("enter HttpClient.executeMethod(HostConfiguration,HttpMethod)");
        return executeMethod(hostConfiguration2, method, (HttpState) null);
    }

    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r2v1, resolved type: java.lang.Object} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r7v3, resolved type: org.apache.commons.httpclient.HostConfiguration} */
    /* JADX WARNING: Multi-variable type inference failed */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public int executeMethod(org.apache.commons.httpclient.HostConfiguration r7, org.apache.commons.httpclient.HttpMethod r8, org.apache.commons.httpclient.HttpState r9) throws java.io.IOException, org.apache.commons.httpclient.HttpException {
        /*
            r6 = this;
            org.apache.commons.logging.Log r0 = LOG
            java.lang.String r1 = "enter HttpClient.executeMethod(HostConfiguration,HttpMethod,HttpState)"
            r0.trace(r1)
            if (r8 == 0) goto L_0x0047
            org.apache.commons.httpclient.HostConfiguration r0 = r6.getHostConfiguration()
            if (r7 != 0) goto L_0x0010
            r7 = r0
        L_0x0010:
            org.apache.commons.httpclient.URI r1 = r8.getURI()
            if (r7 == r0) goto L_0x001c
            boolean r2 = r1.isAbsoluteURI()
            if (r2 == 0) goto L_0x002c
        L_0x001c:
            java.lang.Object r2 = r7.clone()
            r7 = r2
            org.apache.commons.httpclient.HostConfiguration r7 = (org.apache.commons.httpclient.HostConfiguration) r7
            boolean r2 = r1.isAbsoluteURI()
            if (r2 == 0) goto L_0x002c
            r7.setHost((org.apache.commons.httpclient.URI) r1)
        L_0x002c:
            org.apache.commons.httpclient.HttpMethodDirector r2 = new org.apache.commons.httpclient.HttpMethodDirector
            org.apache.commons.httpclient.HttpConnectionManager r3 = r6.getHttpConnectionManager()
            org.apache.commons.httpclient.params.HttpClientParams r4 = r6.params
            if (r9 != 0) goto L_0x003b
            org.apache.commons.httpclient.HttpState r5 = r6.getState()
            goto L_0x003c
        L_0x003b:
            r5 = r9
        L_0x003c:
            r2.<init>(r3, r7, r4, r5)
            r2.executeMethod(r8)
            int r3 = r8.getStatusCode()
            return r3
        L_0x0047:
            java.lang.IllegalArgumentException r0 = new java.lang.IllegalArgumentException
            java.lang.String r1 = "HttpMethod parameter may not be null"
            r0.<init>(r1)
            throw r0
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.httpclient.HttpClient.executeMethod(org.apache.commons.httpclient.HostConfiguration, org.apache.commons.httpclient.HttpMethod, org.apache.commons.httpclient.HttpState):int");
    }

    public String getHost() {
        return this.hostConfiguration.getHost();
    }

    public int getPort() {
        return this.hostConfiguration.getPort();
    }

    public synchronized HostConfiguration getHostConfiguration() {
        return this.hostConfiguration;
    }

    public synchronized void setHostConfiguration(HostConfiguration hostConfiguration2) {
        this.hostConfiguration = hostConfiguration2;
    }

    public synchronized HttpConnectionManager getHttpConnectionManager() {
        return this.httpConnectionManager;
    }

    public synchronized void setHttpConnectionManager(HttpConnectionManager httpConnectionManager2) {
        this.httpConnectionManager = httpConnectionManager2;
        if (this.httpConnectionManager != null) {
            this.httpConnectionManager.getParams().setDefaults(this.params);
        }
    }

    public HttpClientParams getParams() {
        return this.params;
    }

    public void setParams(HttpClientParams params2) {
        if (params2 != null) {
            this.params = params2;
            return;
        }
        throw new IllegalArgumentException("Parameters may not be null");
    }
}
