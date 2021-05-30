package org.apache.xmlrpc.client;

import java.io.IOException;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.net.URLConnection;
import org.apache.xmlrpc.XmlRpcException;
import org.apache.xmlrpc.XmlRpcRequest;
import org.apache.xmlrpc.client.XmlRpcStreamTransport;
import org.apache.xmlrpc.common.XmlRpcStreamRequestConfig;
import org.apache.xmlrpc.util.HttpUtil;
import org.xml.sax.SAXException;

public class XmlRpcSunHttpTransport extends XmlRpcHttpTransport {
    private static final String userAgent = (USER_AGENT + " (Sun HTTP Transport)");
    private URLConnection conn;

    public XmlRpcSunHttpTransport(XmlRpcClient pClient) {
        super(pClient, userAgent);
    }

    /* access modifiers changed from: protected */
    public URLConnection newURLConnection(URL pURL) throws IOException {
        return pURL.openConnection();
    }

    /* access modifiers changed from: protected */
    public URLConnection getURLConnection() {
        return this.conn;
    }

    public Object sendRequest(XmlRpcRequest pRequest) throws XmlRpcException {
        try {
            URLConnection c = newURLConnection(((XmlRpcHttpClientConfig) pRequest.getConfig()).getServerURL());
            this.conn = c;
            c.setUseCaches(false);
            c.setDoInput(true);
            c.setDoOutput(true);
            return super.sendRequest(pRequest);
        } catch (IOException e) {
            throw new XmlRpcException("Failed to create URLConnection: " + e.getMessage(), (Throwable) e);
        }
    }

    /* access modifiers changed from: protected */
    public void setRequestHeader(String pHeader, String pValue) {
        getURLConnection().setRequestProperty(pHeader, pValue);
    }

    /* access modifiers changed from: protected */
    public void close() throws XmlRpcClientException {
        URLConnection c = getURLConnection();
        if (c instanceof HttpURLConnection) {
            ((HttpURLConnection) c).disconnect();
        }
    }

    /* access modifiers changed from: protected */
    public boolean isResponseGzipCompressed(XmlRpcStreamRequestConfig pConfig) {
        return HttpUtil.isUsingGzipEncoding(getURLConnection().getHeaderField("Content-Encoding"));
    }

    /* access modifiers changed from: protected */
    public InputStream getInputStream() throws XmlRpcException {
        try {
            URLConnection connection = getURLConnection();
            if (connection instanceof HttpURLConnection) {
                HttpURLConnection httpConnection = (HttpURLConnection) connection;
                int responseCode = httpConnection.getResponseCode();
                if (responseCode < 200 || responseCode > 299) {
                    throw new XmlRpcHttpTransportException(responseCode, httpConnection.getResponseMessage());
                }
            }
            return connection.getInputStream();
        } catch (IOException e) {
            throw new XmlRpcException("Failed to create input stream: " + e.getMessage(), (Throwable) e);
        }
    }

    /* access modifiers changed from: protected */
    public void writeRequest(XmlRpcStreamTransport.ReqWriter pWriter) throws IOException, XmlRpcException, SAXException {
        pWriter.write(getURLConnection().getOutputStream());
    }
}
