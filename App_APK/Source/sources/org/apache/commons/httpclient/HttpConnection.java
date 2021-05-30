package org.apache.commons.httpclient;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InterruptedIOException;
import java.io.OutputStream;
import java.net.InetAddress;
import java.net.Socket;
import java.net.SocketException;
import org.apache.commons.httpclient.params.HttpConnectionParams;
import org.apache.commons.httpclient.protocol.Protocol;
import org.apache.commons.httpclient.protocol.ProtocolSocketFactory;
import org.apache.commons.httpclient.protocol.SecureProtocolSocketFactory;
import org.apache.commons.httpclient.util.EncodingUtil;
import org.apache.commons.httpclient.util.ExceptionUtil;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

public class HttpConnection {
    private static final byte[] CRLF = {13, 10};
    private static final Log LOG;
    static /* synthetic */ Class class$org$apache$commons$httpclient$HttpConnection;
    private String hostName;
    private HttpConnectionManager httpConnectionManager;
    private InputStream inputStream;
    protected boolean isOpen;
    private InputStream lastResponseInputStream;
    private InetAddress localAddress;
    private boolean locked;
    private OutputStream outputStream;
    private HttpConnectionParams params;
    private int portNumber;
    private Protocol protocolInUse;
    private String proxyHostName;
    private int proxyPortNumber;
    private Socket socket;
    private boolean tunnelEstablished;
    private boolean usingSecureSocket;

    public HttpConnection(String host, int port) {
        this((String) null, -1, host, (String) null, port, Protocol.getProtocol("http"));
    }

    public HttpConnection(String host, int port, Protocol protocol) {
        this((String) null, -1, host, (String) null, port, protocol);
    }

    public HttpConnection(String host, String virtualHost, int port, Protocol protocol) {
        this((String) null, -1, host, virtualHost, port, protocol);
    }

    public HttpConnection(String proxyHost, int proxyPort, String host, int port) {
        this(proxyHost, proxyPort, host, (String) null, port, Protocol.getProtocol("http"));
    }

    public HttpConnection(HostConfiguration hostConfiguration) {
        this(hostConfiguration.getProxyHost(), hostConfiguration.getProxyPort(), hostConfiguration.getHost(), hostConfiguration.getPort(), hostConfiguration.getProtocol());
        this.localAddress = hostConfiguration.getLocalAddress();
    }

    public HttpConnection(String proxyHost, int proxyPort, String host, String virtualHost, int port, Protocol protocol) {
        this(proxyHost, proxyPort, host, port, protocol);
    }

    public HttpConnection(String proxyHost, int proxyPort, String host, int port, Protocol protocol) {
        this.hostName = null;
        this.portNumber = -1;
        this.proxyHostName = null;
        this.proxyPortNumber = -1;
        this.socket = null;
        this.inputStream = null;
        this.outputStream = null;
        this.lastResponseInputStream = null;
        this.isOpen = false;
        this.params = new HttpConnectionParams();
        this.locked = false;
        this.usingSecureSocket = false;
        this.tunnelEstablished = false;
        if (host == null) {
            throw new IllegalArgumentException("host parameter is null");
        } else if (protocol != null) {
            this.proxyHostName = proxyHost;
            this.proxyPortNumber = proxyPort;
            this.hostName = host;
            this.portNumber = protocol.resolvePort(port);
            this.protocolInUse = protocol;
        } else {
            throw new IllegalArgumentException("protocol is null");
        }
    }

    /* access modifiers changed from: protected */
    public Socket getSocket() {
        return this.socket;
    }

    public String getHost() {
        return this.hostName;
    }

    public void setHost(String host) throws IllegalStateException {
        if (host != null) {
            assertNotOpen();
            this.hostName = host;
            return;
        }
        throw new IllegalArgumentException("host parameter is null");
    }

    public String getVirtualHost() {
        return this.hostName;
    }

    public void setVirtualHost(String host) throws IllegalStateException {
        assertNotOpen();
    }

    public int getPort() {
        if (this.portNumber < 0) {
            return isSecure() ? 443 : 80;
        }
        return this.portNumber;
    }

    public void setPort(int port) throws IllegalStateException {
        assertNotOpen();
        this.portNumber = port;
    }

    public String getProxyHost() {
        return this.proxyHostName;
    }

    public void setProxyHost(String host) throws IllegalStateException {
        assertNotOpen();
        this.proxyHostName = host;
    }

    public int getProxyPort() {
        return this.proxyPortNumber;
    }

    public void setProxyPort(int port) throws IllegalStateException {
        assertNotOpen();
        this.proxyPortNumber = port;
    }

    public boolean isSecure() {
        return this.protocolInUse.isSecure();
    }

    public Protocol getProtocol() {
        return this.protocolInUse;
    }

    public void setProtocol(Protocol protocol) {
        assertNotOpen();
        if (protocol != null) {
            this.protocolInUse = protocol;
            return;
        }
        throw new IllegalArgumentException("protocol is null");
    }

    public InetAddress getLocalAddress() {
        return this.localAddress;
    }

    public void setLocalAddress(InetAddress localAddress2) {
        assertNotOpen();
        this.localAddress = localAddress2;
    }

    public boolean isOpen() {
        return this.isOpen;
    }

    public boolean closeIfStale() throws IOException {
        if (!this.isOpen || !isStale()) {
            return false;
        }
        LOG.debug("Connection is stale, closing...");
        close();
        return true;
    }

    public boolean isStaleCheckingEnabled() {
        return this.params.isStaleCheckingEnabled();
    }

    public void setStaleCheckingEnabled(boolean staleCheckEnabled) {
        this.params.setStaleCheckingEnabled(staleCheckEnabled);
    }

    /* access modifiers changed from: protected */
    public boolean isStale() throws IOException {
        if (!this.isOpen) {
            return true;
        }
        boolean isStale = false;
        try {
            if (this.inputStream.available() > 0) {
                return false;
            }
            this.socket.setSoTimeout(1);
            this.inputStream.mark(1);
            if (this.inputStream.read() == -1) {
                isStale = true;
            } else {
                this.inputStream.reset();
            }
            this.socket.setSoTimeout(this.params.getSoTimeout());
            return isStale;
        } catch (InterruptedIOException e) {
            if (ExceptionUtil.isSocketTimeoutException(e)) {
                return isStale;
            }
            throw e;
        } catch (IOException e2) {
            LOG.debug("An error occurred while reading from the socket, is appears to be stale", e2);
            return true;
        } catch (Throwable th) {
            this.socket.setSoTimeout(this.params.getSoTimeout());
            throw th;
        }
    }

    public boolean isProxied() {
        return this.proxyHostName != null && this.proxyPortNumber > 0;
    }

    public void setLastResponseInputStream(InputStream inStream) {
        this.lastResponseInputStream = inStream;
    }

    public InputStream getLastResponseInputStream() {
        return this.lastResponseInputStream;
    }

    public HttpConnectionParams getParams() {
        return this.params;
    }

    public void setParams(HttpConnectionParams params2) {
        if (params2 != null) {
            this.params = params2;
            return;
        }
        throw new IllegalArgumentException("Parameters may not be null");
    }

    public void setSoTimeout(int timeout) throws SocketException, IllegalStateException {
        this.params.setSoTimeout(timeout);
        if (this.socket != null) {
            this.socket.setSoTimeout(timeout);
        }
    }

    public void setSocketTimeout(int timeout) throws SocketException, IllegalStateException {
        assertOpen();
        if (this.socket != null) {
            this.socket.setSoTimeout(timeout);
        }
    }

    public int getSoTimeout() throws SocketException {
        return this.params.getSoTimeout();
    }

    public void setConnectionTimeout(int timeout) {
        this.params.setConnectionTimeout(timeout);
    }

    public void open() throws IOException {
        ProtocolSocketFactory socketFactory;
        LOG.trace("enter HttpConnection.open()");
        String host = this.proxyHostName == null ? this.hostName : this.proxyHostName;
        int port = this.proxyHostName == null ? this.portNumber : this.proxyPortNumber;
        assertNotOpen();
        if (LOG.isDebugEnabled()) {
            Log log = LOG;
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Open connection to ");
            stringBuffer.append(host);
            stringBuffer.append(":");
            stringBuffer.append(port);
            log.debug(stringBuffer.toString());
        }
        try {
            boolean z = false;
            if (this.socket == null) {
                this.usingSecureSocket = isSecure() && !isProxied();
                if (!isSecure() || !isProxied()) {
                    socketFactory = this.protocolInUse.getSocketFactory();
                } else {
                    socketFactory = Protocol.getProtocol("http").getSocketFactory();
                }
                this.socket = socketFactory.createSocket(host, port, this.localAddress, 0, this.params);
            }
            this.socket.setTcpNoDelay(this.params.getTcpNoDelay());
            this.socket.setSoTimeout(this.params.getSoTimeout());
            int linger = this.params.getLinger();
            if (linger >= 0) {
                Socket socket2 = this.socket;
                if (linger > 0) {
                    z = true;
                }
                socket2.setSoLinger(z, linger);
            }
            int sndBufSize = this.params.getSendBufferSize();
            if (sndBufSize >= 0) {
                this.socket.setSendBufferSize(sndBufSize);
            }
            int rcvBufSize = this.params.getReceiveBufferSize();
            if (rcvBufSize >= 0) {
                this.socket.setReceiveBufferSize(rcvBufSize);
            }
            int outbuffersize = this.socket.getSendBufferSize();
            if (outbuffersize > 2048 || outbuffersize <= 0) {
                outbuffersize = 2048;
            }
            int inbuffersize = this.socket.getReceiveBufferSize();
            if (inbuffersize > 2048 || inbuffersize <= 0) {
                inbuffersize = 2048;
            }
            this.inputStream = new BufferedInputStream(this.socket.getInputStream(), inbuffersize);
            this.outputStream = new BufferedOutputStream(this.socket.getOutputStream(), outbuffersize);
            this.isOpen = true;
        } catch (IOException e) {
            closeSocketAndStreams();
            throw e;
        }
    }

    public void tunnelCreated() throws IllegalStateException, IOException {
        LOG.trace("enter HttpConnection.tunnelCreated()");
        if (!isSecure() || !isProxied()) {
            throw new IllegalStateException("Connection must be secure and proxied to use this feature");
        } else if (!this.usingSecureSocket) {
            if (LOG.isDebugEnabled()) {
                Log log = LOG;
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append("Secure tunnel to ");
                stringBuffer.append(this.hostName);
                stringBuffer.append(":");
                stringBuffer.append(this.portNumber);
                log.debug(stringBuffer.toString());
            }
            this.socket = ((SecureProtocolSocketFactory) this.protocolInUse.getSocketFactory()).createSocket(this.socket, this.hostName, this.portNumber, true);
            int sndBufSize = this.params.getSendBufferSize();
            if (sndBufSize >= 0) {
                this.socket.setSendBufferSize(sndBufSize);
            }
            int rcvBufSize = this.params.getReceiveBufferSize();
            if (rcvBufSize >= 0) {
                this.socket.setReceiveBufferSize(rcvBufSize);
            }
            int outbuffersize = this.socket.getSendBufferSize();
            if (outbuffersize > 2048) {
                outbuffersize = 2048;
            }
            int inbuffersize = this.socket.getReceiveBufferSize();
            if (inbuffersize > 2048) {
                inbuffersize = 2048;
            }
            this.inputStream = new BufferedInputStream(this.socket.getInputStream(), inbuffersize);
            this.outputStream = new BufferedOutputStream(this.socket.getOutputStream(), outbuffersize);
            this.usingSecureSocket = true;
            this.tunnelEstablished = true;
        } else {
            throw new IllegalStateException("Already using a secure socket");
        }
    }

    public boolean isTransparent() {
        return !isProxied() || this.tunnelEstablished;
    }

    public void flushRequestOutputStream() throws IOException {
        LOG.trace("enter HttpConnection.flushRequestOutputStream()");
        assertOpen();
        this.outputStream.flush();
    }

    public OutputStream getRequestOutputStream() throws IOException, IllegalStateException {
        LOG.trace("enter HttpConnection.getRequestOutputStream()");
        assertOpen();
        OutputStream out = this.outputStream;
        if (Wire.CONTENT_WIRE.enabled()) {
            return new WireLogOutputStream(out, Wire.CONTENT_WIRE);
        }
        return out;
    }

    public InputStream getResponseInputStream() throws IOException, IllegalStateException {
        LOG.trace("enter HttpConnection.getResponseInputStream()");
        assertOpen();
        return this.inputStream;
    }

    public boolean isResponseAvailable() throws IOException {
        LOG.trace("enter HttpConnection.isResponseAvailable()");
        if (!this.isOpen || this.inputStream.available() <= 0) {
            return false;
        }
        return true;
    }

    /* JADX WARNING: Unknown top exception splitter block from list: {B:18:0x0077=Splitter:B:18:0x0077, B:8:0x003f=Splitter:B:8:0x003f} */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public boolean isResponseAvailable(int r6) throws java.io.IOException {
        /*
            r5 = this;
            org.apache.commons.logging.Log r0 = LOG
            java.lang.String r1 = "enter HttpConnection.isResponseAvailable(int)"
            r0.trace(r1)
            r5.assertOpen()
            r0 = 0
            java.io.InputStream r1 = r5.inputStream
            int r1 = r1.available()
            if (r1 <= 0) goto L_0x0016
            r0 = 1
            goto L_0x008d
        L_0x0016:
            java.net.Socket r1 = r5.socket     // Catch:{ InterruptedIOException -> 0x004d }
            r1.setSoTimeout(r6)     // Catch:{ InterruptedIOException -> 0x004d }
            java.io.InputStream r1 = r5.inputStream     // Catch:{ InterruptedIOException -> 0x004d }
            r2 = 1
            r1.mark(r2)     // Catch:{ InterruptedIOException -> 0x004d }
            java.io.InputStream r1 = r5.inputStream     // Catch:{ InterruptedIOException -> 0x004d }
            int r1 = r1.read()     // Catch:{ InterruptedIOException -> 0x004d }
            r2 = -1
            if (r1 == r2) goto L_0x0038
            java.io.InputStream r2 = r5.inputStream     // Catch:{ InterruptedIOException -> 0x004d }
            r2.reset()     // Catch:{ InterruptedIOException -> 0x004d }
            org.apache.commons.logging.Log r2 = LOG     // Catch:{ InterruptedIOException -> 0x004d }
            java.lang.String r3 = "Input data available"
            r2.debug(r3)     // Catch:{ InterruptedIOException -> 0x004d }
            r0 = 1
            goto L_0x003f
        L_0x0038:
            org.apache.commons.logging.Log r2 = LOG     // Catch:{ InterruptedIOException -> 0x004d }
            java.lang.String r3 = "Input data not available"
            r2.debug(r3)     // Catch:{ InterruptedIOException -> 0x004d }
        L_0x003f:
            java.net.Socket r1 = r5.socket     // Catch:{ IOException -> 0x0083 }
            org.apache.commons.httpclient.params.HttpConnectionParams r2 = r5.params     // Catch:{ IOException -> 0x0083 }
            int r2 = r2.getSoTimeout()     // Catch:{ IOException -> 0x0083 }
            r1.setSoTimeout(r2)     // Catch:{ IOException -> 0x0083 }
            goto L_0x0082
        L_0x004b:
            r1 = move-exception
            goto L_0x008f
        L_0x004d:
            r1 = move-exception
            boolean r2 = org.apache.commons.httpclient.util.ExceptionUtil.isSocketTimeoutException(r1)     // Catch:{ all -> 0x004b }
            if (r2 == 0) goto L_0x008e
            org.apache.commons.logging.Log r2 = LOG     // Catch:{ all -> 0x004b }
            boolean r2 = r2.isDebugEnabled()     // Catch:{ all -> 0x004b }
            if (r2 == 0) goto L_0x0077
            org.apache.commons.logging.Log r2 = LOG     // Catch:{ all -> 0x004b }
            java.lang.StringBuffer r3 = new java.lang.StringBuffer     // Catch:{ all -> 0x004b }
            r3.<init>()     // Catch:{ all -> 0x004b }
            java.lang.String r4 = "Input data not available after "
            r3.append(r4)     // Catch:{ all -> 0x004b }
            r3.append(r6)     // Catch:{ all -> 0x004b }
            java.lang.String r4 = " ms"
            r3.append(r4)     // Catch:{ all -> 0x004b }
            java.lang.String r3 = r3.toString()     // Catch:{ all -> 0x004b }
            r2.debug(r3)     // Catch:{ all -> 0x004b }
        L_0x0077:
            java.net.Socket r1 = r5.socket     // Catch:{ IOException -> 0x0083 }
            org.apache.commons.httpclient.params.HttpConnectionParams r2 = r5.params     // Catch:{ IOException -> 0x0083 }
            int r2 = r2.getSoTimeout()     // Catch:{ IOException -> 0x0083 }
            r1.setSoTimeout(r2)     // Catch:{ IOException -> 0x0083 }
        L_0x0082:
            goto L_0x008d
        L_0x0083:
            r1 = move-exception
            org.apache.commons.logging.Log r2 = LOG
            java.lang.String r3 = "An error ocurred while resetting soTimeout, we will assume that no response is available."
            r2.debug(r3, r1)
            r0 = 0
        L_0x008d:
            return r0
        L_0x008e:
            throw r1     // Catch:{ all -> 0x004b }
        L_0x008f:
            java.net.Socket r2 = r5.socket     // Catch:{ IOException -> 0x009c }
            org.apache.commons.httpclient.params.HttpConnectionParams r3 = r5.params     // Catch:{ IOException -> 0x009c }
            int r3 = r3.getSoTimeout()     // Catch:{ IOException -> 0x009c }
            r2.setSoTimeout(r3)     // Catch:{ IOException -> 0x009c }
            goto L_0x00a5
        L_0x009c:
            r2 = move-exception
            org.apache.commons.logging.Log r3 = LOG
            java.lang.String r4 = "An error ocurred while resetting soTimeout, we will assume that no response is available."
            r3.debug(r4, r2)
            r0 = 0
        L_0x00a5:
            throw r1
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.httpclient.HttpConnection.isResponseAvailable(int):boolean");
    }

    public void write(byte[] data) throws IOException, IllegalStateException {
        LOG.trace("enter HttpConnection.write(byte[])");
        write(data, 0, data.length);
    }

    public void write(byte[] data, int offset, int length) throws IOException, IllegalStateException {
        LOG.trace("enter HttpConnection.write(byte[], int, int)");
        if (offset < 0) {
            throw new IllegalArgumentException("Array offset may not be negative");
        } else if (length < 0) {
            throw new IllegalArgumentException("Array length may not be negative");
        } else if (offset + length <= data.length) {
            assertOpen();
            this.outputStream.write(data, offset, length);
        } else {
            throw new IllegalArgumentException("Given offset and length exceed the array length");
        }
    }

    public void writeLine(byte[] data) throws IOException, IllegalStateException {
        LOG.trace("enter HttpConnection.writeLine(byte[])");
        write(data);
        writeLine();
    }

    public void writeLine() throws IOException, IllegalStateException {
        LOG.trace("enter HttpConnection.writeLine()");
        write(CRLF);
    }

    public void print(String data) throws IOException, IllegalStateException {
        LOG.trace("enter HttpConnection.print(String)");
        write(EncodingUtil.getBytes(data, "ISO-8859-1"));
    }

    public void print(String data, String charset) throws IOException, IllegalStateException {
        LOG.trace("enter HttpConnection.print(String)");
        write(EncodingUtil.getBytes(data, charset));
    }

    public void printLine(String data) throws IOException, IllegalStateException {
        LOG.trace("enter HttpConnection.printLine(String)");
        writeLine(EncodingUtil.getBytes(data, "ISO-8859-1"));
    }

    public void printLine(String data, String charset) throws IOException, IllegalStateException {
        LOG.trace("enter HttpConnection.printLine(String)");
        writeLine(EncodingUtil.getBytes(data, charset));
    }

    public void printLine() throws IOException, IllegalStateException {
        LOG.trace("enter HttpConnection.printLine()");
        writeLine();
    }

    public String readLine() throws IOException, IllegalStateException {
        LOG.trace("enter HttpConnection.readLine()");
        assertOpen();
        return HttpParser.readLine(this.inputStream);
    }

    public String readLine(String charset) throws IOException, IllegalStateException {
        LOG.trace("enter HttpConnection.readLine()");
        assertOpen();
        return HttpParser.readLine(this.inputStream, charset);
    }

    public void shutdownOutput() {
        LOG.trace("enter HttpConnection.shutdownOutput()");
        try {
            Class<?> cls = this.socket.getClass();
            cls.getMethod("shutdownOutput", new Class[0]).invoke(this.socket, new Object[0]);
        } catch (Exception ex) {
            LOG.debug("Unexpected Exception caught", ex);
        }
    }

    public void close() {
        LOG.trace("enter HttpConnection.close()");
        closeSocketAndStreams();
    }

    public HttpConnectionManager getHttpConnectionManager() {
        return this.httpConnectionManager;
    }

    public void setHttpConnectionManager(HttpConnectionManager httpConnectionManager2) {
        this.httpConnectionManager = httpConnectionManager2;
    }

    public void releaseConnection() {
        LOG.trace("enter HttpConnection.releaseConnection()");
        if (this.locked) {
            LOG.debug("Connection is locked.  Call to releaseConnection() ignored.");
        } else if (this.httpConnectionManager != null) {
            LOG.debug("Releasing connection back to connection manager.");
            this.httpConnectionManager.releaseConnection(this);
        } else {
            LOG.warn("HttpConnectionManager is null.  Connection cannot be released.");
        }
    }

    /* access modifiers changed from: protected */
    public boolean isLocked() {
        return this.locked;
    }

    /* access modifiers changed from: protected */
    public void setLocked(boolean locked2) {
        this.locked = locked2;
    }

    /* access modifiers changed from: protected */
    public void closeSocketAndStreams() {
        LOG.trace("enter HttpConnection.closeSockedAndStreams()");
        this.isOpen = false;
        this.lastResponseInputStream = null;
        if (this.outputStream != null) {
            OutputStream temp = this.outputStream;
            this.outputStream = null;
            try {
                temp.close();
            } catch (Exception ex) {
                LOG.debug("Exception caught when closing output", ex);
            }
        }
        if (this.inputStream != null) {
            InputStream temp2 = this.inputStream;
            this.inputStream = null;
            try {
                temp2.close();
            } catch (Exception ex2) {
                LOG.debug("Exception caught when closing input", ex2);
            }
        }
        if (this.socket != null) {
            Socket temp3 = this.socket;
            this.socket = null;
            try {
                temp3.close();
            } catch (Exception ex3) {
                LOG.debug("Exception caught when closing socket", ex3);
            }
        }
        this.tunnelEstablished = false;
        this.usingSecureSocket = false;
    }

    /* access modifiers changed from: protected */
    public void assertNotOpen() throws IllegalStateException {
        if (this.isOpen) {
            throw new IllegalStateException("Connection is open");
        }
    }

    /* access modifiers changed from: protected */
    public void assertOpen() throws IllegalStateException {
        if (!this.isOpen) {
            throw new IllegalStateException("Connection is not open");
        }
    }

    public int getSendBufferSize() throws SocketException {
        if (this.socket == null) {
            return -1;
        }
        return this.socket.getSendBufferSize();
    }

    public void setSendBufferSize(int sendBufferSize) throws SocketException {
        this.params.setSendBufferSize(sendBufferSize);
    }

    static {
        Class cls;
        if (class$org$apache$commons$httpclient$HttpConnection == null) {
            cls = class$("org.apache.commons.httpclient.HttpConnection");
            class$org$apache$commons$httpclient$HttpConnection = cls;
        } else {
            cls = class$org$apache$commons$httpclient$HttpConnection;
        }
        LOG = LogFactory.getLog(cls);
    }

    static /* synthetic */ Class class$(String x0) {
        try {
            return Class.forName(x0);
        } catch (ClassNotFoundException x1) {
            throw new NoClassDefFoundError(x1.getMessage());
        }
    }
}
