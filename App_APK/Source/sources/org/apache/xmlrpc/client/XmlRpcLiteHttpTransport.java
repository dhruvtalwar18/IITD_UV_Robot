package org.apache.xmlrpc.client;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.UnsupportedEncodingException;
import java.net.Socket;
import java.net.URL;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.apache.commons.httpclient.cookie.CookieSpec;
import org.apache.xmlrpc.XmlRpcException;
import org.apache.xmlrpc.XmlRpcRequest;
import org.apache.xmlrpc.client.XmlRpcStreamTransport;
import org.apache.xmlrpc.common.XmlRpcStreamRequestConfig;
import org.xml.sax.SAXException;

public class XmlRpcLiteHttpTransport extends XmlRpcHttpTransport {
    private static final String userAgent = (USER_AGENT + " (Lite HTTP Transport)");
    private XmlRpcHttpClientConfig config;
    private final Map headers = new HashMap();
    private String host;
    private String hostname;
    private InputStream input;
    private OutputStream output;
    private int port;
    private boolean responseGzipCompressed = false;
    /* access modifiers changed from: private */
    public Socket socket;
    private boolean ssl;
    private String uri;

    public XmlRpcLiteHttpTransport(XmlRpcClient pClient) {
        super(pClient, userAgent);
    }

    public Object sendRequest(XmlRpcRequest pRequest) throws XmlRpcException {
        String str;
        this.config = (XmlRpcHttpClientConfig) pRequest.getConfig();
        URL url = this.config.getServerURL();
        this.ssl = "https".equals(url.getProtocol());
        this.hostname = url.getHost();
        int p = url.getPort();
        this.port = p < 1 ? 80 : p;
        String u = url.getFile();
        this.uri = (u == null || "".equals(u)) ? CookieSpec.PATH_DELIM : u;
        if (this.port == 80) {
            str = this.hostname;
        } else {
            str = this.hostname + ":" + this.port;
        }
        this.host = str;
        this.headers.put("Host", this.host);
        return super.sendRequest(pRequest);
    }

    /* access modifiers changed from: protected */
    public void setRequestHeader(String pHeader, String pValue) {
        List list;
        Object value = this.headers.get(pHeader);
        if (value == null) {
            this.headers.put(pHeader, pValue);
            return;
        }
        if (value instanceof String) {
            list = new ArrayList();
            list.add(value);
            this.headers.put(pHeader, list);
        } else {
            list = (List) value;
        }
        list.add(pValue);
    }

    /* access modifiers changed from: protected */
    public void close() throws XmlRpcClientException {
        IOException e = null;
        if (this.input != null) {
            try {
                this.input.close();
            } catch (IOException ex) {
                e = ex;
            }
        }
        if (this.output != null) {
            try {
                this.output.close();
            } catch (IOException ex2) {
                if (e != null) {
                    e = ex2;
                }
            }
        }
        if (this.socket != null) {
            try {
                this.socket.close();
            } catch (IOException ex3) {
                if (e != null) {
                    e = ex3;
                }
            }
        }
        if (e != null) {
            throw new XmlRpcClientException("Failed to close connection: " + e.getMessage(), e);
        }
    }

    /* JADX WARNING: Code restructure failed: missing block: B:4:?, code lost:
        sendRequestHeaders(r7.output);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:5:0x0025, code lost:
        return r7.output;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    private java.io.OutputStream getOutputStream() throws org.apache.xmlrpc.XmlRpcException {
        /*
            r7 = this;
            r0 = 3
            r1 = 100
            r2 = 0
        L_0x0004:
            boolean r3 = r7.ssl     // Catch:{ ConnectException -> 0x0028 }
            java.lang.String r4 = r7.hostname     // Catch:{ ConnectException -> 0x0028 }
            int r5 = r7.port     // Catch:{ ConnectException -> 0x0028 }
            java.net.Socket r3 = r7.newSocket(r3, r4, r5)     // Catch:{ ConnectException -> 0x0028 }
            r7.socket = r3     // Catch:{ ConnectException -> 0x0028 }
            org.apache.xmlrpc.client.XmlRpcLiteHttpTransport$1 r3 = new org.apache.xmlrpc.client.XmlRpcLiteHttpTransport$1     // Catch:{ ConnectException -> 0x0028 }
            java.net.Socket r4 = r7.socket     // Catch:{ ConnectException -> 0x0028 }
            java.io.OutputStream r4 = r4.getOutputStream()     // Catch:{ ConnectException -> 0x0028 }
            r3.<init>(r4)     // Catch:{ ConnectException -> 0x0028 }
            r7.output = r3     // Catch:{ ConnectException -> 0x0028 }
            java.io.OutputStream r2 = r7.output     // Catch:{ IOException -> 0x0026 }
            r7.sendRequestHeaders(r2)     // Catch:{ IOException -> 0x0026 }
            java.io.OutputStream r2 = r7.output     // Catch:{ IOException -> 0x0026 }
            return r2
        L_0x0026:
            r0 = move-exception
            goto L_0x0065
        L_0x0028:
            r3 = move-exception
            r4 = 3
            if (r2 >= r4) goto L_0x0036
            r4 = 100
            java.lang.Thread.sleep(r4)     // Catch:{ InterruptedException -> 0x0032 }
            goto L_0x0033
        L_0x0032:
            r4 = move-exception
        L_0x0033:
            int r2 = r2 + 1
            goto L_0x0004
        L_0x0036:
            org.apache.xmlrpc.XmlRpcException r4 = new org.apache.xmlrpc.XmlRpcException     // Catch:{ IOException -> 0x0026 }
            java.lang.StringBuilder r5 = new java.lang.StringBuilder     // Catch:{ IOException -> 0x0026 }
            r5.<init>()     // Catch:{ IOException -> 0x0026 }
            java.lang.String r6 = "Failed to connect to "
            r5.append(r6)     // Catch:{ IOException -> 0x0026 }
            java.lang.String r6 = r7.hostname     // Catch:{ IOException -> 0x0026 }
            r5.append(r6)     // Catch:{ IOException -> 0x0026 }
            java.lang.String r6 = ":"
            r5.append(r6)     // Catch:{ IOException -> 0x0026 }
            int r6 = r7.port     // Catch:{ IOException -> 0x0026 }
            r5.append(r6)     // Catch:{ IOException -> 0x0026 }
            java.lang.String r6 = ": "
            r5.append(r6)     // Catch:{ IOException -> 0x0026 }
            java.lang.String r6 = r3.getMessage()     // Catch:{ IOException -> 0x0026 }
            r5.append(r6)     // Catch:{ IOException -> 0x0026 }
            java.lang.String r5 = r5.toString()     // Catch:{ IOException -> 0x0026 }
            r4.<init>((java.lang.String) r5, (java.lang.Throwable) r3)     // Catch:{ IOException -> 0x0026 }
            throw r4     // Catch:{ IOException -> 0x0026 }
        L_0x0065:
            org.apache.xmlrpc.XmlRpcException r1 = new org.apache.xmlrpc.XmlRpcException
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            java.lang.String r3 = "Failed to open connection to "
            r2.append(r3)
            java.lang.String r3 = r7.hostname
            r2.append(r3)
            java.lang.String r3 = ":"
            r2.append(r3)
            int r3 = r7.port
            r2.append(r3)
            java.lang.String r3 = ": "
            r2.append(r3)
            java.lang.String r3 = r0.getMessage()
            r2.append(r3)
            java.lang.String r2 = r2.toString()
            r1.<init>((java.lang.String) r2, (java.lang.Throwable) r0)
            throw r1
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.xmlrpc.client.XmlRpcLiteHttpTransport.getOutputStream():java.io.OutputStream");
    }

    /* access modifiers changed from: protected */
    public Socket newSocket(boolean pSSL, String pHostName, int pPort) throws UnknownHostException, IOException {
        if (!pSSL) {
            return new Socket(pHostName, pPort);
        }
        throw new IOException("Unable to create SSL connections, use the XmlRpcLite14HttpTransportFactory.");
    }

    private byte[] toHTTPBytes(String pValue) throws UnsupportedEncodingException {
        return pValue.getBytes("US-ASCII");
    }

    private void sendHeader(OutputStream pOut, String pKey, String pValue) throws IOException {
        pOut.write(toHTTPBytes(pKey + ": " + pValue + "\r\n"));
    }

    private void sendRequestHeaders(OutputStream pOut) throws IOException {
        pOut.write(("POST " + this.uri + " HTTP/1.0\r\n").getBytes("US-ASCII"));
        for (Map.Entry entry : this.headers.entrySet()) {
            String key = (String) entry.getKey();
            Object value = entry.getValue();
            if (value instanceof String) {
                sendHeader(pOut, key, (String) value);
            } else {
                List list = (List) value;
                for (int i = 0; i < list.size(); i++) {
                    sendHeader(pOut, key, (String) list.get(i));
                }
            }
        }
        pOut.write(toHTTPBytes("\r\n"));
    }

    /* access modifiers changed from: protected */
    public boolean isResponseGzipCompressed(XmlRpcStreamRequestConfig pConfig) {
        return this.responseGzipCompressed;
    }

    /* access modifiers changed from: protected */
    /* JADX WARNING: No exception handlers in catch block: Catch:{  } */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public java.io.InputStream getInputStream() throws org.apache.xmlrpc.XmlRpcException {
        /*
            r9 = this;
            r0 = 2048(0x800, float:2.87E-42)
            byte[] r0 = new byte[r0]
            org.apache.xmlrpc.client.XmlRpcHttpClientConfig r1 = r9.config     // Catch:{ IOException -> 0x00cd }
            int r1 = r1.getReplyTimeout()     // Catch:{ IOException -> 0x00cd }
            if (r1 == 0) goto L_0x0017
            java.net.Socket r1 = r9.socket     // Catch:{ IOException -> 0x00cd }
            org.apache.xmlrpc.client.XmlRpcHttpClientConfig r2 = r9.config     // Catch:{ IOException -> 0x00cd }
            int r2 = r2.getReplyTimeout()     // Catch:{ IOException -> 0x00cd }
            r1.setSoTimeout(r2)     // Catch:{ IOException -> 0x00cd }
        L_0x0017:
            java.io.BufferedInputStream r1 = new java.io.BufferedInputStream     // Catch:{ IOException -> 0x00cd }
            java.net.Socket r2 = r9.socket     // Catch:{ IOException -> 0x00cd }
            java.io.InputStream r2 = r2.getInputStream()     // Catch:{ IOException -> 0x00cd }
            r1.<init>(r2)     // Catch:{ IOException -> 0x00cd }
            r9.input = r1     // Catch:{ IOException -> 0x00cd }
            java.io.InputStream r1 = r9.input     // Catch:{ IOException -> 0x00cd }
            java.lang.String r1 = org.apache.xmlrpc.util.HttpUtil.readLine(r1, r0)     // Catch:{ IOException -> 0x00cd }
            java.util.StringTokenizer r2 = new java.util.StringTokenizer     // Catch:{ IOException -> 0x00cd }
            r2.<init>(r1)     // Catch:{ IOException -> 0x00cd }
            r2.nextToken()     // Catch:{ IOException -> 0x00cd }
            java.lang.String r3 = r2.nextToken()     // Catch:{ IOException -> 0x00cd }
            java.lang.String r4 = "\n\r"
            java.lang.String r4 = r2.nextToken(r4)     // Catch:{ IOException -> 0x00cd }
            int r5 = java.lang.Integer.parseInt(r3)     // Catch:{ NumberFormatException -> 0x00ac }
            r6 = 200(0xc8, float:2.8E-43)
            if (r5 < r6) goto L_0x00a6
            r6 = 299(0x12b, float:4.19E-43)
            if (r5 > r6) goto L_0x00a6
            r6 = -1
            r7 = r1
            r1 = -1
        L_0x004d:
            java.io.InputStream r8 = r9.input     // Catch:{ IOException -> 0x00cd }
            java.lang.String r8 = org.apache.xmlrpc.util.HttpUtil.readLine(r8, r0)     // Catch:{ IOException -> 0x00cd }
            r7 = r8
            if (r7 == 0) goto L_0x0099
            java.lang.String r8 = ""
            boolean r8 = r8.equals(r7)     // Catch:{ IOException -> 0x00cd }
            if (r8 == 0) goto L_0x005f
            goto L_0x0099
        L_0x005f:
            java.lang.String r8 = r7.toLowerCase()     // Catch:{ IOException -> 0x00cd }
            r7 = r8
            java.lang.String r8 = "content-length:"
            boolean r8 = r7.startsWith(r8)     // Catch:{ IOException -> 0x00cd }
            if (r8 == 0) goto L_0x0080
            java.lang.String r8 = "content-length:"
            int r8 = r8.length()     // Catch:{ IOException -> 0x00cd }
            java.lang.String r8 = r7.substring(r8)     // Catch:{ IOException -> 0x00cd }
            java.lang.String r8 = r8.trim()     // Catch:{ IOException -> 0x00cd }
            int r8 = java.lang.Integer.parseInt(r8)     // Catch:{ IOException -> 0x00cd }
            r1 = r8
            goto L_0x004d
        L_0x0080:
            java.lang.String r8 = "content-encoding:"
            boolean r8 = r7.startsWith(r8)     // Catch:{ IOException -> 0x00cd }
            if (r8 == 0) goto L_0x004d
            java.lang.String r8 = "content-encoding:"
            int r8 = r8.length()     // Catch:{ IOException -> 0x00cd }
            java.lang.String r8 = r7.substring(r8)     // Catch:{ IOException -> 0x00cd }
            boolean r8 = org.apache.xmlrpc.util.HttpUtil.isUsingGzipEncoding((java.lang.String) r8)     // Catch:{ IOException -> 0x00cd }
            r9.responseGzipCompressed = r8     // Catch:{ IOException -> 0x00cd }
            goto L_0x004d
        L_0x0099:
            if (r1 != r6) goto L_0x009e
            java.io.InputStream r6 = r9.input     // Catch:{ IOException -> 0x00cd }
            goto L_0x00a5
        L_0x009e:
            org.apache.xmlrpc.util.LimitedInputStream r6 = new org.apache.xmlrpc.util.LimitedInputStream     // Catch:{ IOException -> 0x00cd }
            java.io.InputStream r8 = r9.input     // Catch:{ IOException -> 0x00cd }
            r6.<init>(r8, r1)     // Catch:{ IOException -> 0x00cd }
        L_0x00a5:
            return r6
        L_0x00a6:
            org.apache.xmlrpc.client.XmlRpcHttpTransportException r6 = new org.apache.xmlrpc.client.XmlRpcHttpTransportException     // Catch:{ IOException -> 0x00cd }
            r6.<init>(r5, r4)     // Catch:{ IOException -> 0x00cd }
            throw r6     // Catch:{ IOException -> 0x00cd }
        L_0x00ac:
            r5 = move-exception
            org.apache.xmlrpc.client.XmlRpcClientException r6 = new org.apache.xmlrpc.client.XmlRpcClientException     // Catch:{ IOException -> 0x00cd }
            java.lang.StringBuilder r7 = new java.lang.StringBuilder     // Catch:{ IOException -> 0x00cd }
            r7.<init>()     // Catch:{ IOException -> 0x00cd }
            java.lang.String r8 = "Server returned invalid status code: "
            r7.append(r8)     // Catch:{ IOException -> 0x00cd }
            r7.append(r3)     // Catch:{ IOException -> 0x00cd }
            java.lang.String r8 = " "
            r7.append(r8)     // Catch:{ IOException -> 0x00cd }
            r7.append(r4)     // Catch:{ IOException -> 0x00cd }
            java.lang.String r7 = r7.toString()     // Catch:{ IOException -> 0x00cd }
            r8 = 0
            r6.<init>(r7, r8)     // Catch:{ IOException -> 0x00cd }
            throw r6     // Catch:{ IOException -> 0x00cd }
        L_0x00cd:
            r1 = move-exception
            org.apache.xmlrpc.client.XmlRpcClientException r2 = new org.apache.xmlrpc.client.XmlRpcClientException
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r4 = "Failed to read server response: "
            r3.append(r4)
            java.lang.String r4 = r1.getMessage()
            r3.append(r4)
            java.lang.String r3 = r3.toString()
            r2.<init>(r3, r1)
            throw r2
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.xmlrpc.client.XmlRpcLiteHttpTransport.getInputStream():java.io.InputStream");
    }

    /* access modifiers changed from: protected */
    public boolean isUsingByteArrayOutput(XmlRpcHttpClientConfig pConfig) {
        boolean result = super.isUsingByteArrayOutput(pConfig);
        if (result) {
            return result;
        }
        throw new IllegalStateException("The Content-Length header is required with HTTP/1.0, and HTTP/1.1 is unsupported by the Lite HTTP Transport.");
    }

    /* access modifiers changed from: protected */
    public void writeRequest(XmlRpcStreamTransport.ReqWriter pWriter) throws XmlRpcException, IOException, SAXException {
        pWriter.write(getOutputStream());
    }
}
