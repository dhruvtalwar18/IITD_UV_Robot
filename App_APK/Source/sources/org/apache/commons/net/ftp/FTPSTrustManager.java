package org.apache.commons.net.ftp;

import java.security.cert.CertificateException;
import java.security.cert.X509Certificate;
import javax.net.ssl.X509TrustManager;

public class FTPSTrustManager implements X509TrustManager {
    public void checkClientTrusted(X509Certificate[] certificates, String authType) {
    }

    public void checkServerTrusted(X509Certificate[] certificates, String authType) throws CertificateException {
        for (X509Certificate checkValidity : certificates) {
            checkValidity.checkValidity();
        }
    }

    public X509Certificate[] getAcceptedIssuers() {
        return null;
    }
}
