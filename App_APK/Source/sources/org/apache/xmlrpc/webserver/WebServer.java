package org.apache.xmlrpc.webserver;

import java.io.IOException;
import java.net.BindException;
import java.net.InetAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.ArrayList;
import java.util.List;
import java.util.StringTokenizer;
import org.apache.xmlrpc.server.XmlRpcStreamServer;
import org.apache.xmlrpc.util.ThreadPool;

public class WebServer implements Runnable {
    static final String HTTP_11 = "HTTP/1.1";
    protected final List accept;
    private InetAddress address;
    protected final List deny;
    private Thread listener;
    private boolean paranoid;
    private ThreadPool pool;
    private int port;
    protected final XmlRpcStreamServer server;
    protected ServerSocket serverSocket;

    private class AddressMatcher {
        private final int[] pattern;

        AddressMatcher(String pAddress) {
            try {
                this.pattern = new int[4];
                StringTokenizer st = new StringTokenizer(pAddress, ".");
                if (st.countTokens() == 4) {
                    for (int i = 0; i < 4; i++) {
                        String next = st.nextToken();
                        if ("*".equals(next)) {
                            this.pattern[i] = 256;
                        } else {
                            this.pattern[i] = (byte) Integer.parseInt(next);
                        }
                    }
                    return;
                }
                throw new IllegalArgumentException();
            } catch (Exception e) {
                throw new IllegalArgumentException("\"" + pAddress + "\" does not represent a valid IP address");
            }
        }

        /* access modifiers changed from: package-private */
        public boolean matches(byte[] pAddress) {
            for (int i = 0; i < 4; i++) {
                if (this.pattern[i] <= 255 && this.pattern[i] != pAddress[i]) {
                    return false;
                }
            }
            return true;
        }
    }

    /* access modifiers changed from: protected */
    public XmlRpcStreamServer newXmlRpcStreamServer() {
        return new ConnectionServer();
    }

    public WebServer(int pPort) {
        this(pPort, (InetAddress) null);
    }

    public WebServer(int pPort, InetAddress pAddr) {
        this.accept = new ArrayList();
        this.deny = new ArrayList();
        this.server = newXmlRpcStreamServer();
        this.address = pAddr;
        this.port = pPort;
    }

    /* access modifiers changed from: protected */
    public ServerSocket createServerSocket(int pPort, int backlog, InetAddress addr) throws IOException {
        return new ServerSocket(pPort, backlog, addr);
    }

    private synchronized void setupServerSocket(int backlog) throws IOException {
        int i = 1;
        while (true) {
            try {
                this.serverSocket = createServerSocket(this.port, backlog, this.address);
                if (this.serverSocket.getSoTimeout() <= 0) {
                    this.serverSocket.setSoTimeout(4096);
                }
            } catch (BindException e) {
                if (i != 10) {
                    long waitUntil = System.currentTimeMillis() + 1000;
                    while (true) {
                        long l = waitUntil - System.currentTimeMillis();
                        if (l <= 0) {
                            break;
                        }
                        try {
                            Thread.sleep(l);
                        } catch (InterruptedException e2) {
                        }
                    }
                    i++;
                } else {
                    throw e;
                }
            }
        }
    }

    public void start() throws IOException {
        setupServerSocket(50);
        if (this.listener == null) {
            this.listener = new Thread(this, "XML-RPC Weblistener");
            this.listener.start();
        }
    }

    public void setParanoid(boolean pParanoid) {
        this.paranoid = pParanoid;
    }

    /* access modifiers changed from: protected */
    public boolean isParanoid() {
        return this.paranoid;
    }

    public void acceptClient(String pAddress) {
        this.accept.add(new AddressMatcher(pAddress));
    }

    public void denyClient(String pAddress) {
        this.deny.add(new AddressMatcher(pAddress));
    }

    /* access modifiers changed from: protected */
    public boolean allowConnection(Socket s) {
        if (!this.paranoid) {
            return true;
        }
        int l = this.deny.size();
        byte[] addr = s.getInetAddress().getAddress();
        for (int i = 0; i < l; i++) {
            if (((AddressMatcher) this.deny.get(i)).matches(addr)) {
                return false;
            }
        }
        int l2 = this.accept.size();
        for (int i2 = 0; i2 < l2; i2++) {
            if (((AddressMatcher) this.accept.get(i2)).matches(addr)) {
                return true;
            }
        }
        return false;
    }

    /* access modifiers changed from: protected */
    public ThreadPool.Task newTask(WebServer pServer, XmlRpcStreamServer pXmlRpcServer, Socket pSocket) throws IOException {
        return new Connection(pServer, pXmlRpcServer, pSocket);
    }

    /* JADX INFO: finally extract failed */
    /* JADX WARNING: Code restructure failed: missing block: B:23:0x005b, code lost:
        r1 = move-exception;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:24:0x005c, code lost:
        if (r0 != null) goto L_0x005e;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:26:?, code lost:
        r0.close();
     */
    /* JADX WARNING: Code restructure failed: missing block: B:29:?, code lost:
        throw r1;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:30:0x0064, code lost:
        r0 = move-exception;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:32:?, code lost:
        log(r0);
     */
    /* JADX WARNING: Failed to process nested try/catch */
    /* JADX WARNING: Removed duplicated region for block: B:33:0x0069 A[ExcHandler: InterruptedIOException (e java.io.InterruptedIOException), Splitter:B:4:0x000a] */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public void run() {
        /*
            r4 = this;
            org.apache.xmlrpc.util.ThreadPool r0 = r4.newThreadPool()
            r4.pool = r0
        L_0x0006:
            java.lang.Thread r0 = r4.listener     // Catch:{ all -> 0x0080 }
            if (r0 == 0) goto L_0x006b
            java.net.ServerSocket r0 = r4.serverSocket     // Catch:{ InterruptedIOException -> 0x0069, Throwable -> 0x0064 }
            java.net.Socket r0 = r0.accept()     // Catch:{ InterruptedIOException -> 0x0069, Throwable -> 0x0064 }
            r1 = 1
            r0.setTcpNoDelay(r1)     // Catch:{ SocketException -> 0x0015 }
            goto L_0x0019
        L_0x0015:
            r1 = move-exception
            r4.log((java.lang.Throwable) r1)     // Catch:{ InterruptedIOException -> 0x0069, Throwable -> 0x0064 }
        L_0x0019:
            boolean r1 = r4.allowConnection(r0)     // Catch:{ all -> 0x005b }
            if (r1 == 0) goto L_0x0053
            r1 = 30000(0x7530, float:4.2039E-41)
            r0.setSoTimeout(r1)     // Catch:{ all -> 0x005b }
            org.apache.xmlrpc.server.XmlRpcStreamServer r1 = r4.server     // Catch:{ all -> 0x005b }
            org.apache.xmlrpc.util.ThreadPool$Task r1 = r4.newTask(r4, r1, r0)     // Catch:{ all -> 0x005b }
            org.apache.xmlrpc.util.ThreadPool r2 = r4.pool     // Catch:{ all -> 0x005b }
            boolean r2 = r2.startTask(r1)     // Catch:{ all -> 0x005b }
            if (r2 == 0) goto L_0x0034
            r0 = 0
            goto L_0x0053
        L_0x0034:
            java.lang.StringBuilder r2 = new java.lang.StringBuilder     // Catch:{ all -> 0x005b }
            r2.<init>()     // Catch:{ all -> 0x005b }
            java.lang.String r3 = "Maximum load of "
            r2.append(r3)     // Catch:{ all -> 0x005b }
            org.apache.xmlrpc.util.ThreadPool r3 = r4.pool     // Catch:{ all -> 0x005b }
            int r3 = r3.getMaxThreads()     // Catch:{ all -> 0x005b }
            r2.append(r3)     // Catch:{ all -> 0x005b }
            java.lang.String r3 = " exceeded, rejecting client"
            r2.append(r3)     // Catch:{ all -> 0x005b }
            java.lang.String r2 = r2.toString()     // Catch:{ all -> 0x005b }
            r4.log((java.lang.String) r2)     // Catch:{ all -> 0x005b }
        L_0x0053:
            if (r0 == 0) goto L_0x006a
            r0.close()     // Catch:{ Throwable -> 0x0059, InterruptedIOException -> 0x0069 }
            goto L_0x006a
        L_0x0059:
            r1 = move-exception
            goto L_0x006a
        L_0x005b:
            r1 = move-exception
            if (r0 == 0) goto L_0x0063
            r0.close()     // Catch:{ Throwable -> 0x0062, InterruptedIOException -> 0x0069 }
            goto L_0x0063
        L_0x0062:
            r2 = move-exception
        L_0x0063:
            throw r1     // Catch:{ InterruptedIOException -> 0x0069, Throwable -> 0x0064 }
        L_0x0064:
            r0 = move-exception
            r4.log((java.lang.Throwable) r0)     // Catch:{ all -> 0x0080 }
            goto L_0x006a
        L_0x0069:
            r0 = move-exception
        L_0x006a:
            goto L_0x0006
        L_0x006b:
            java.net.ServerSocket r0 = r4.serverSocket
            if (r0 == 0) goto L_0x0079
            java.net.ServerSocket r0 = r4.serverSocket     // Catch:{ IOException -> 0x0075 }
            r0.close()     // Catch:{ IOException -> 0x0075 }
            goto L_0x0079
        L_0x0075:
            r0 = move-exception
            r4.log((java.lang.Throwable) r0)
        L_0x0079:
            org.apache.xmlrpc.util.ThreadPool r0 = r4.pool
            r0.shutdown()
            return
        L_0x0080:
            r0 = move-exception
            java.net.ServerSocket r1 = r4.serverSocket
            if (r1 == 0) goto L_0x008f
            java.net.ServerSocket r1 = r4.serverSocket     // Catch:{ IOException -> 0x008b }
            r1.close()     // Catch:{ IOException -> 0x008b }
            goto L_0x008f
        L_0x008b:
            r1 = move-exception
            r4.log((java.lang.Throwable) r1)
        L_0x008f:
            org.apache.xmlrpc.util.ThreadPool r1 = r4.pool
            r1.shutdown()
            throw r0
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.xmlrpc.webserver.WebServer.run():void");
    }

    /* access modifiers changed from: protected */
    public ThreadPool newThreadPool() {
        return new ThreadPool(this.server.getMaxThreads(), "XML-RPC");
    }

    public synchronized void shutdown() {
        if (this.listener != null) {
            Thread l = this.listener;
            this.listener = null;
            l.interrupt();
            if (this.pool != null) {
                this.pool.shutdown();
            }
        }
    }

    public int getPort() {
        return this.serverSocket.getLocalPort();
    }

    public void log(Throwable pError) {
        this.server.getErrorLogger().log(pError.getMessage() == null ? pError.getClass().getName() : pError.getMessage(), pError);
    }

    public void log(String pMessage) {
        this.server.getErrorLogger().log(pMessage);
    }

    public XmlRpcStreamServer getXmlRpcServer() {
        return this.server;
    }
}
