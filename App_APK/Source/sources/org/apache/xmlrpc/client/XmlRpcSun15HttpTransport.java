package org.apache.xmlrpc.client;

import java.io.IOException;
import java.net.Proxy;
import java.net.URL;
import java.net.URLConnection;
import javax.net.ssl.HttpsURLConnection;
import javax.net.ssl.SSLSocketFactory;
import org.apache.xmlrpc.XmlRpcRequest;

public class XmlRpcSun15HttpTransport extends XmlRpcSun14HttpTransport {
    private Proxy proxy;

    public XmlRpcSun15HttpTransport(XmlRpcClient pClient) {
        super(pClient);
    }

    public void setProxy(Proxy pProxy) {
        this.proxy = pProxy;
    }

    public Proxy getProxy() {
        return this.proxy;
    }

    /* access modifiers changed from: protected */
    public void initHttpHeaders(XmlRpcRequest pRequest) throws XmlRpcClientException {
        XmlRpcHttpClientConfig config = (XmlRpcHttpClientConfig) pRequest.getConfig();
        int connectionTimeout = config.getConnectionTimeout();
        if (connectionTimeout > 0) {
            getURLConnection().setConnectTimeout(connectionTimeout);
        }
        int replyTimeout = config.getReplyTimeout();
        if (replyTimeout > 0) {
            getURLConnection().setReadTimeout(replyTimeout);
        }
        super.initHttpHeaders(pRequest);
    }

    /* access modifiers changed from: protected */
    public URLConnection newURLConnection(URL pURL) throws IOException {
        Proxy prox = getProxy();
        URLConnection conn = prox == null ? pURL.openConnection() : pURL.openConnection(prox);
        SSLSocketFactory sslSockFactory = getSSLSocketFactory();
        if (sslSockFactory != null && (conn instanceof HttpsURLConnection)) {
            ((HttpsURLConnection) conn).setSSLSocketFactory(sslSockFactory);
        }
        return conn;
    }
}
