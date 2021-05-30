package org.apache.xmlrpc.client;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.UnsupportedEncodingException;
import java.lang.reflect.UndeclaredThrowableException;
import java.net.URL;
import java.util.Properties;
import org.apache.xmlrpc.XmlRpcException;
import org.apache.xmlrpc.XmlRpcRequest;
import org.apache.xmlrpc.client.XmlRpcStreamTransport;
import org.apache.xmlrpc.util.HttpUtil;
import org.xml.sax.SAXException;

public abstract class XmlRpcHttpTransport extends XmlRpcStreamTransport {
    public static final String USER_AGENT;
    private String userAgent;

    /* access modifiers changed from: protected */
    public abstract void setRequestHeader(String str, String str2);

    protected class ByteArrayReqWriter implements XmlRpcStreamTransport.ReqWriter {
        private final ByteArrayOutputStream baos = new ByteArrayOutputStream();

        ByteArrayReqWriter(XmlRpcRequest pRequest) throws XmlRpcException, IOException, SAXException {
            new XmlRpcStreamTransport.ReqWriterImpl(pRequest).write(this.baos);
        }

        /* access modifiers changed from: protected */
        public int getContentLength() {
            return this.baos.size();
        }

        public void write(OutputStream pStream) throws IOException {
            try {
                this.baos.writeTo(pStream);
                pStream.close();
                OutputStream pStream2 = null;
                if (pStream2 != null) {
                    try {
                        pStream2.close();
                        return;
                    } catch (Throwable th) {
                        return;
                    }
                } else {
                    return;
                }
            } catch (Throwable th2) {
            }
            throw th;
        }
    }

    static {
        URL url = XmlRpcHttpTransport.class.getResource("XmlRpcClient.properties");
        if (url != null) {
            InputStream stream = null;
            try {
                stream = url.openStream();
                Properties props = new Properties();
                props.load(stream);
                USER_AGENT = props.getProperty("user.agent");
                if (USER_AGENT == null || USER_AGENT.trim().length() == 0) {
                    throw new IllegalStateException("The property user.agent is not set.");
                }
                stream.close();
                InputStream stream2 = null;
                if (stream2 != null) {
                    try {
                        stream2.close();
                        return;
                    } catch (Throwable th) {
                        return;
                    }
                } else {
                    return;
                }
            } catch (IOException e) {
                throw new UndeclaredThrowableException(e, "Failed to load resource " + url + ": " + e.getMessage());
            } catch (Throwable th2) {
            }
        } else {
            throw new IllegalStateException("Failed to locate resource: XmlRpcClient.properties");
        }
        throw th;
    }

    protected XmlRpcHttpTransport(XmlRpcClient pClient, String pUserAgent) {
        super(pClient);
        this.userAgent = pUserAgent;
    }

    /* access modifiers changed from: protected */
    public String getUserAgent() {
        return this.userAgent;
    }

    /* access modifiers changed from: protected */
    public void setCredentials(XmlRpcHttpClientConfig pConfig) throws XmlRpcClientException {
        try {
            String auth = HttpUtil.encodeBasicAuthentication(pConfig.getBasicUserName(), pConfig.getBasicPassword(), pConfig.getBasicEncoding());
            if (auth != null) {
                setRequestHeader("Authorization", "Basic " + auth);
            }
        } catch (UnsupportedEncodingException e) {
            throw new XmlRpcClientException("Unsupported encoding: " + pConfig.getBasicEncoding(), e);
        }
    }

    /* access modifiers changed from: protected */
    public void setContentLength(int pLength) {
        setRequestHeader("Content-Length", Integer.toString(pLength));
    }

    /* access modifiers changed from: protected */
    public void setCompressionHeaders(XmlRpcHttpClientConfig pConfig) {
        if (pConfig.isGzipCompressing()) {
            setRequestHeader("Content-Encoding", "gzip");
        }
        if (pConfig.isGzipRequesting()) {
            setRequestHeader("Accept-Encoding", "gzip");
        }
    }

    /* access modifiers changed from: protected */
    public void initHttpHeaders(XmlRpcRequest pRequest) throws XmlRpcClientException {
        XmlRpcHttpClientConfig config = (XmlRpcHttpClientConfig) pRequest.getConfig();
        setRequestHeader("Content-Type", "text/xml");
        if (config.getUserAgent() != null) {
            setRequestHeader("User-Agent", config.getUserAgent());
        } else {
            setRequestHeader("User-Agent", getUserAgent());
        }
        setCredentials(config);
        setCompressionHeaders(config);
    }

    public Object sendRequest(XmlRpcRequest pRequest) throws XmlRpcException {
        initHttpHeaders(pRequest);
        return super.sendRequest(pRequest);
    }

    /* access modifiers changed from: protected */
    public boolean isUsingByteArrayOutput(XmlRpcHttpClientConfig pConfig) {
        return !pConfig.isEnabledForExtensions() || !pConfig.isContentLengthOptional();
    }

    /* access modifiers changed from: protected */
    public XmlRpcStreamTransport.ReqWriter newReqWriter(XmlRpcRequest pRequest) throws XmlRpcException, IOException, SAXException {
        XmlRpcHttpClientConfig config = (XmlRpcHttpClientConfig) pRequest.getConfig();
        if (!isUsingByteArrayOutput(config)) {
            return super.newReqWriter(pRequest);
        }
        ByteArrayReqWriter reqWriter = new ByteArrayReqWriter(pRequest);
        setContentLength(reqWriter.getContentLength());
        if (isCompressingRequest(config)) {
            return new XmlRpcStreamTransport.GzipReqWriter(reqWriter);
        }
        return reqWriter;
    }
}
