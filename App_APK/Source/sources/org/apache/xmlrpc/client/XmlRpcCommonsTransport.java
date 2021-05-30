package org.apache.xmlrpc.client;

import java.io.BufferedOutputStream;
import java.io.FilterOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import org.apache.commons.httpclient.Credentials;
import org.apache.commons.httpclient.Header;
import org.apache.commons.httpclient.HttpClient;
import org.apache.commons.httpclient.HttpException;
import org.apache.commons.httpclient.HttpMethod;
import org.apache.commons.httpclient.HttpVersion;
import org.apache.commons.httpclient.URI;
import org.apache.commons.httpclient.URIException;
import org.apache.commons.httpclient.UsernamePasswordCredentials;
import org.apache.commons.httpclient.auth.AuthScope;
import org.apache.commons.httpclient.methods.PostMethod;
import org.apache.commons.httpclient.methods.RequestEntity;
import org.apache.commons.httpclient.params.HttpMethodParams;
import org.apache.xmlrpc.XmlRpcException;
import org.apache.xmlrpc.XmlRpcRequest;
import org.apache.xmlrpc.client.XmlRpcStreamTransport;
import org.apache.xmlrpc.common.XmlRpcStreamRequestConfig;
import org.apache.xmlrpc.util.HttpUtil;
import org.apache.xmlrpc.util.XmlRpcIOException;
import org.xml.sax.SAXException;

public class XmlRpcCommonsTransport extends XmlRpcHttpTransport {
    private static final int MAX_REDIRECT_ATTEMPTS = 100;
    private static final String userAgent = (USER_AGENT + " (Jakarta Commons httpclient Transport)");
    protected final HttpClient client;
    /* access modifiers changed from: private */
    public XmlRpcHttpClientConfig config;
    /* access modifiers changed from: private */
    public int contentLength = -1;
    protected PostMethod method;

    public XmlRpcCommonsTransport(XmlRpcCommonsTransportFactory pFactory) {
        super(pFactory.getClient(), userAgent);
        HttpClient httpClient = pFactory.getHttpClient();
        this.client = httpClient == null ? newHttpClient() : httpClient;
    }

    /* access modifiers changed from: protected */
    public void setContentLength(int pLength) {
        this.contentLength = pLength;
    }

    /* access modifiers changed from: protected */
    public HttpClient newHttpClient() {
        return new HttpClient();
    }

    /* access modifiers changed from: protected */
    public void initHttpHeaders(XmlRpcRequest pRequest) throws XmlRpcClientException {
        this.config = (XmlRpcHttpClientConfig) pRequest.getConfig();
        this.method = newPostMethod(this.config);
        super.initHttpHeaders(pRequest);
        if (this.config.getConnectionTimeout() != 0) {
            this.client.getHttpConnectionManager().getParams().setConnectionTimeout(this.config.getConnectionTimeout());
        }
        if (this.config.getReplyTimeout() != 0) {
            this.client.getHttpConnectionManager().getParams().setSoTimeout(this.config.getReplyTimeout());
        }
        this.method.getParams().setVersion(HttpVersion.HTTP_1_1);
    }

    /* access modifiers changed from: protected */
    public PostMethod newPostMethod(XmlRpcHttpClientConfig pConfig) {
        return new PostMethod(pConfig.getServerURL().toString());
    }

    /* access modifiers changed from: protected */
    public void setRequestHeader(String pHeader, String pValue) {
        this.method.setRequestHeader(new Header(pHeader, pValue));
    }

    /* access modifiers changed from: protected */
    public boolean isResponseGzipCompressed() {
        Header h = this.method.getResponseHeader("Content-Encoding");
        if (h == null) {
            return false;
        }
        return HttpUtil.isUsingGzipEncoding(h.getValue());
    }

    /* access modifiers changed from: protected */
    public InputStream getInputStream() throws XmlRpcException {
        try {
            checkStatus(this.method);
            return this.method.getResponseBodyAsStream();
        } catch (HttpException e) {
            throw new XmlRpcClientException("Error in HTTP transport: " + e.getMessage(), e);
        } catch (IOException e2) {
            throw new XmlRpcClientException("I/O error in server communication: " + e2.getMessage(), e2);
        }
    }

    /* access modifiers changed from: protected */
    public void setCredentials(XmlRpcHttpClientConfig pConfig) throws XmlRpcClientException {
        String userName = pConfig.getBasicUserName();
        if (userName != null) {
            String enc = pConfig.getBasicEncoding();
            if (enc == null) {
                enc = "UTF-8";
            }
            this.client.getParams().setParameter(HttpMethodParams.CREDENTIAL_CHARSET, enc);
            Credentials creds = new UsernamePasswordCredentials(userName, pConfig.getBasicPassword());
            this.client.getState().setCredentials(new AuthScope((String) null, -1, (String) null, AuthScope.ANY_SCHEME), creds);
            this.client.getParams().setAuthenticationPreemptive(true);
        }
    }

    /* access modifiers changed from: protected */
    public void close() throws XmlRpcClientException {
        this.method.releaseConnection();
    }

    /* access modifiers changed from: protected */
    public boolean isResponseGzipCompressed(XmlRpcStreamRequestConfig pConfig) {
        Header h = this.method.getResponseHeader("Content-Encoding");
        if (h == null) {
            return false;
        }
        return HttpUtil.isUsingGzipEncoding(h.getValue());
    }

    /* access modifiers changed from: protected */
    public boolean isRedirectRequired() {
        int statusCode = this.method.getStatusCode();
        if (statusCode == 307) {
            return true;
        }
        switch (statusCode) {
            case 301:
            case 302:
            case 303:
                return true;
            default:
                return false;
        }
    }

    /* access modifiers changed from: protected */
    public void resetClientForRedirect() throws XmlRpcException {
        Header locationHeader = this.method.getResponseHeader("location");
        if (locationHeader != null) {
            try {
                this.method.setURI(new URI(locationHeader.getValue(), true, this.method.getURI().getProtocolCharset()));
                this.method.getHostAuthState().invalidate();
            } catch (URIException ex) {
                throw new XmlRpcException(ex.getMessage(), (Throwable) ex);
            }
        } else {
            throw new XmlRpcException("Invalid redirect: Missing location header");
        }
    }

    /* access modifiers changed from: protected */
    public void writeRequest(final XmlRpcStreamTransport.ReqWriter pWriter) throws XmlRpcException {
        this.method.setRequestEntity(new RequestEntity() {
            public boolean isRepeatable() {
                return true;
            }

            public void writeRequest(OutputStream pOut) throws IOException {
                OutputStream ostream;
                try {
                    if (XmlRpcCommonsTransport.this.isUsingByteArrayOutput(XmlRpcCommonsTransport.this.config)) {
                        ostream = new FilterOutputStream(pOut) {
                            public void close() throws IOException {
                                flush();
                            }
                        };
                    } else {
                        ostream = new BufferedOutputStream(pOut) {
                            public void close() throws IOException {
                                flush();
                            }
                        };
                    }
                    pWriter.write(ostream);
                } catch (XmlRpcException e) {
                    throw new XmlRpcIOException(e);
                } catch (SAXException e2) {
                    throw new XmlRpcIOException(e2);
                }
            }

            public long getContentLength() {
                return (long) XmlRpcCommonsTransport.this.contentLength;
            }

            public String getContentType() {
                return "text/xml";
            }
        });
        int redirectAttempts = 0;
        while (true) {
            try {
                this.client.executeMethod(this.method);
                if (isRedirectRequired()) {
                    int redirectAttempts2 = redirectAttempts + 1;
                    if (redirectAttempts <= 100) {
                        resetClientForRedirect();
                        redirectAttempts = redirectAttempts2;
                    } else {
                        throw new XmlRpcException("Too many redirects.");
                    }
                } else {
                    return;
                }
            } catch (XmlRpcIOException e) {
                Throwable t = e.getLinkedException();
                if (t instanceof XmlRpcException) {
                    throw ((XmlRpcException) t);
                }
                throw new XmlRpcException("Unexpected exception: " + t.getMessage(), t);
            } catch (IOException e2) {
                throw new XmlRpcException("I/O error while communicating with HTTP server: " + e2.getMessage(), (Throwable) e2);
            }
        }
    }

    private void checkStatus(HttpMethod pMethod) throws XmlRpcHttpTransportException {
        int status = pMethod.getStatusCode();
        if (status < 200 || status > 299) {
            throw new XmlRpcHttpTransportException(status, pMethod.getStatusText());
        }
    }
}
