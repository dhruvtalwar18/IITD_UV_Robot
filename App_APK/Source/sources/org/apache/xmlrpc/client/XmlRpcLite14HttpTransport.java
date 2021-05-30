package org.apache.xmlrpc.client;

import javax.net.ssl.SSLSocketFactory;

public class XmlRpcLite14HttpTransport extends XmlRpcLiteHttpTransport {
    private SSLSocketFactory sslSocketFactory;

    public XmlRpcLite14HttpTransport(XmlRpcClient pClient) {
        super(pClient);
    }

    public SSLSocketFactory getSSLSocketFactory() {
        return this.sslSocketFactory;
    }

    public void setSSLSocketFactory(SSLSocketFactory pSSLSocketFactory) {
        this.sslSocketFactory = pSSLSocketFactory;
    }

    /* JADX WARNING: type inference failed for: r1v1, types: [javax.net.SocketFactory] */
    /* access modifiers changed from: protected */
    /* JADX WARNING: Multi-variable type inference failed */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public java.net.Socket newSocket(boolean r3, java.lang.String r4, int r5) throws java.net.UnknownHostException, java.io.IOException {
        /*
            r2 = this;
            if (r3 == 0) goto L_0x0014
            javax.net.ssl.SSLSocketFactory r0 = r2.getSSLSocketFactory()
            if (r0 != 0) goto L_0x000f
            javax.net.SocketFactory r1 = javax.net.ssl.SSLSocketFactory.getDefault()
            r0 = r1
            javax.net.ssl.SSLSocketFactory r0 = (javax.net.ssl.SSLSocketFactory) r0
        L_0x000f:
            java.net.Socket r1 = r0.createSocket(r4, r5)
            return r1
        L_0x0014:
            java.net.Socket r0 = super.newSocket(r3, r4, r5)
            return r0
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.xmlrpc.client.XmlRpcLite14HttpTransport.newSocket(boolean, java.lang.String, int):java.net.Socket");
    }
}
