package org.apache.xmlrpc.client;

import java.io.IOException;
import java.net.URL;
import java.net.URLConnection;
import javax.net.ssl.HttpsURLConnection;
import javax.net.ssl.SSLSocketFactory;

public class XmlRpcSun14HttpTransport extends XmlRpcSunHttpTransport {
    private SSLSocketFactory sslSocketFactory;

    public XmlRpcSun14HttpTransport(XmlRpcClient pClient) {
        super(pClient);
    }

    public void setSSLSocketFactory(SSLSocketFactory pSocketFactory) {
        this.sslSocketFactory = pSocketFactory;
    }

    public SSLSocketFactory getSSLSocketFactory() {
        return this.sslSocketFactory;
    }

    /* access modifiers changed from: protected */
    public URLConnection newURLConnection(URL pURL) throws IOException {
        URLConnection conn = super.newURLConnection(pURL);
        SSLSocketFactory sslSockFactory = getSSLSocketFactory();
        if (sslSockFactory != null && (conn instanceof HttpsURLConnection)) {
            ((HttpsURLConnection) conn).setSSLSocketFactory(sslSockFactory);
        }
        return conn;
    }
}
