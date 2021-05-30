package org.apache.commons.httpclient;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.lang.ref.Reference;
import java.lang.ref.ReferenceQueue;
import java.lang.ref.WeakReference;
import java.net.InetAddress;
import java.net.SocketException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.Map;
import java.util.WeakHashMap;
import org.apache.commons.httpclient.params.HttpConnectionManagerParams;
import org.apache.commons.httpclient.params.HttpConnectionParams;
import org.apache.commons.httpclient.protocol.Protocol;
import org.apache.commons.httpclient.util.IdleConnectionHandler;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

public class MultiThreadedHttpConnectionManager implements HttpConnectionManager {
    private static WeakHashMap ALL_CONNECTION_MANAGERS = new WeakHashMap();
    public static final int DEFAULT_MAX_HOST_CONNECTIONS = 2;
    public static final int DEFAULT_MAX_TOTAL_CONNECTIONS = 20;
    /* access modifiers changed from: private */
    public static final Log LOG;
    /* access modifiers changed from: private */
    public static final ReferenceQueue REFERENCE_QUEUE = new ReferenceQueue();
    private static ReferenceQueueThread REFERENCE_QUEUE_THREAD;
    /* access modifiers changed from: private */
    public static final Map REFERENCE_TO_CONNECTION_SOURCE = new HashMap();
    static /* synthetic */ Class class$org$apache$commons$httpclient$MultiThreadedHttpConnectionManager;
    private ConnectionPool connectionPool = new ConnectionPool();
    /* access modifiers changed from: private */
    public HttpConnectionManagerParams params = new HttpConnectionManagerParams();
    /* access modifiers changed from: private */
    public volatile boolean shutdown = false;

    static {
        Class cls;
        if (class$org$apache$commons$httpclient$MultiThreadedHttpConnectionManager == null) {
            cls = class$("org.apache.commons.httpclient.MultiThreadedHttpConnectionManager");
            class$org$apache$commons$httpclient$MultiThreadedHttpConnectionManager = cls;
        } else {
            cls = class$org$apache$commons$httpclient$MultiThreadedHttpConnectionManager;
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

    public static void shutdownAll() {
        synchronized (REFERENCE_TO_CONNECTION_SOURCE) {
            synchronized (ALL_CONNECTION_MANAGERS) {
                MultiThreadedHttpConnectionManager[] connManagers = (MultiThreadedHttpConnectionManager[]) ALL_CONNECTION_MANAGERS.keySet().toArray(new MultiThreadedHttpConnectionManager[ALL_CONNECTION_MANAGERS.size()]);
                for (int i = 0; i < connManagers.length; i++) {
                    if (connManagers[i] != null) {
                        connManagers[i].shutdown();
                    }
                }
            }
            if (REFERENCE_QUEUE_THREAD != null) {
                REFERENCE_QUEUE_THREAD.shutdown();
                REFERENCE_QUEUE_THREAD = null;
            }
            REFERENCE_TO_CONNECTION_SOURCE.clear();
        }
    }

    /* access modifiers changed from: private */
    public static void storeReferenceToConnection(HttpConnectionWithReference connection, HostConfiguration hostConfiguration, ConnectionPool connectionPool2) {
        ConnectionSource source = new ConnectionSource();
        source.connectionPool = connectionPool2;
        source.hostConfiguration = hostConfiguration;
        synchronized (REFERENCE_TO_CONNECTION_SOURCE) {
            if (REFERENCE_QUEUE_THREAD == null) {
                REFERENCE_QUEUE_THREAD = new ReferenceQueueThread();
                REFERENCE_QUEUE_THREAD.start();
            }
            REFERENCE_TO_CONNECTION_SOURCE.put(connection.reference, source);
        }
    }

    /* access modifiers changed from: private */
    public static void shutdownCheckedOutConnections(ConnectionPool connectionPool2) {
        ArrayList connectionsToClose = new ArrayList();
        synchronized (REFERENCE_TO_CONNECTION_SOURCE) {
            Iterator referenceIter = REFERENCE_TO_CONNECTION_SOURCE.keySet().iterator();
            while (referenceIter.hasNext()) {
                Reference ref = (Reference) referenceIter.next();
                if (((ConnectionSource) REFERENCE_TO_CONNECTION_SOURCE.get(ref)).connectionPool == connectionPool2) {
                    referenceIter.remove();
                    HttpConnection connection = (HttpConnection) ref.get();
                    if (connection != null) {
                        connectionsToClose.add(connection);
                    }
                }
            }
        }
        Iterator i = connectionsToClose.iterator();
        while (i.hasNext()) {
            HttpConnection connection2 = (HttpConnection) i.next();
            connection2.close();
            connection2.setHttpConnectionManager((HttpConnectionManager) null);
            connection2.releaseConnection();
        }
    }

    /* access modifiers changed from: private */
    public static void removeReferenceToConnection(HttpConnectionWithReference connection) {
        synchronized (REFERENCE_TO_CONNECTION_SOURCE) {
            REFERENCE_TO_CONNECTION_SOURCE.remove(connection.reference);
        }
    }

    public MultiThreadedHttpConnectionManager() {
        synchronized (ALL_CONNECTION_MANAGERS) {
            ALL_CONNECTION_MANAGERS.put(this, (Object) null);
        }
    }

    public synchronized void shutdown() {
        synchronized (this.connectionPool) {
            if (!this.shutdown) {
                this.shutdown = true;
                this.connectionPool.shutdown();
            }
        }
    }

    public boolean isConnectionStaleCheckingEnabled() {
        return this.params.isStaleCheckingEnabled();
    }

    public void setConnectionStaleCheckingEnabled(boolean connectionStaleCheckingEnabled) {
        this.params.setStaleCheckingEnabled(connectionStaleCheckingEnabled);
    }

    public void setMaxConnectionsPerHost(int maxHostConnections) {
        this.params.setDefaultMaxConnectionsPerHost(maxHostConnections);
    }

    public int getMaxConnectionsPerHost() {
        return this.params.getDefaultMaxConnectionsPerHost();
    }

    public void setMaxTotalConnections(int maxTotalConnections) {
        this.params.setMaxTotalConnections(maxTotalConnections);
    }

    public int getMaxTotalConnections() {
        return this.params.getMaxTotalConnections();
    }

    public HttpConnection getConnection(HostConfiguration hostConfiguration) {
        while (true) {
            try {
                return getConnectionWithTimeout(hostConfiguration, 0);
            } catch (ConnectionPoolTimeoutException e) {
                LOG.debug("Unexpected exception while waiting for connection", e);
            }
        }
    }

    public HttpConnection getConnectionWithTimeout(HostConfiguration hostConfiguration, long timeout) throws ConnectionPoolTimeoutException {
        LOG.trace("enter HttpConnectionManager.getConnectionWithTimeout(HostConfiguration, long)");
        if (hostConfiguration != null) {
            if (LOG.isDebugEnabled()) {
                Log log = LOG;
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append("HttpConnectionManager.getConnection:  config = ");
                stringBuffer.append(hostConfiguration);
                stringBuffer.append(", timeout = ");
                stringBuffer.append(timeout);
                log.debug(stringBuffer.toString());
            }
            return new HttpConnectionAdapter(doGetConnection(hostConfiguration, timeout));
        }
        throw new IllegalArgumentException("hostConfiguration is null");
    }

    public HttpConnection getConnection(HostConfiguration hostConfiguration, long timeout) throws HttpException {
        LOG.trace("enter HttpConnectionManager.getConnection(HostConfiguration, long)");
        try {
            return getConnectionWithTimeout(hostConfiguration, timeout);
        } catch (ConnectionPoolTimeoutException e) {
            throw new HttpException(e.getMessage());
        }
    }

    /* JADX WARNING: Removed duplicated region for block: B:103:0x016f A[Catch:{ all -> 0x019b, all -> 0x01a2 }] */
    /* JADX WARNING: Removed duplicated region for block: B:105:0x017f A[Catch:{ all -> 0x019b, all -> 0x01a2 }] */
    /* JADX WARNING: Removed duplicated region for block: B:87:0x0136 A[SYNTHETIC, Splitter:B:87:0x0136] */
    /* JADX WARNING: Removed duplicated region for block: B:96:0x015b A[SYNTHETIC, Splitter:B:96:0x015b] */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    private org.apache.commons.httpclient.HttpConnection doGetConnection(org.apache.commons.httpclient.HostConfiguration r25, long r26) throws org.apache.commons.httpclient.ConnectionPoolTimeoutException {
        /*
            r24 = this;
            r1 = r24
            r2 = r25
            r3 = 0
            org.apache.commons.httpclient.params.HttpConnectionManagerParams r0 = r1.params
            int r4 = r0.getMaxConnectionsPerHost(r2)
            org.apache.commons.httpclient.params.HttpConnectionManagerParams r0 = r1.params
            int r5 = r0.getMaxTotalConnections()
            org.apache.commons.httpclient.MultiThreadedHttpConnectionManager$ConnectionPool r6 = r1.connectionPool
            monitor-enter(r6)
            org.apache.commons.httpclient.HostConfiguration r0 = new org.apache.commons.httpclient.HostConfiguration     // Catch:{ all -> 0x019b }
            r0.<init>(r2)     // Catch:{ all -> 0x019b }
            r2 = r0
            org.apache.commons.httpclient.MultiThreadedHttpConnectionManager$ConnectionPool r0 = r1.connectionPool     // Catch:{ all -> 0x019b }
            r7 = 1
            org.apache.commons.httpclient.MultiThreadedHttpConnectionManager$HostConnectionPool r0 = r0.getHostPool(r2, r7)     // Catch:{ all -> 0x019b }
            r8 = r0
            r0 = 0
            r10 = 0
            int r12 = (r26 > r10 ? 1 : (r26 == r10 ? 0 : -1))
            if (r12 <= 0) goto L_0x002a
            goto L_0x002b
        L_0x002a:
            r7 = 0
        L_0x002b:
            r12 = r26
            r14 = 0
            r9 = r0
            r22 = r10
            r10 = r12
            r12 = r22
        L_0x0035:
            if (r3 != 0) goto L_0x0195
            boolean r0 = r1.shutdown     // Catch:{ all -> 0x019b }
            if (r0 != 0) goto L_0x0189
            java.util.LinkedList r0 = r8.freeConnections     // Catch:{ all -> 0x019b }
            int r0 = r0.size()     // Catch:{ all -> 0x019b }
            if (r0 <= 0) goto L_0x0052
            org.apache.commons.httpclient.MultiThreadedHttpConnectionManager$ConnectionPool r0 = r1.connectionPool     // Catch:{ all -> 0x004b }
            org.apache.commons.httpclient.HttpConnection r0 = r0.getFreeConnection(r2)     // Catch:{ all -> 0x004b }
            r3 = r0
            goto L_0x0035
        L_0x004b:
            r0 = move-exception
            r18 = r4
            r19 = r5
            goto L_0x01a0
        L_0x0052:
            int r0 = r8.numConnections     // Catch:{ all -> 0x019b }
            if (r0 >= r4) goto L_0x0066
            org.apache.commons.httpclient.MultiThreadedHttpConnectionManager$ConnectionPool r0 = r1.connectionPool     // Catch:{ all -> 0x004b }
            int r0 = r0.numConnections     // Catch:{ all -> 0x004b }
            if (r0 >= r5) goto L_0x0066
            org.apache.commons.httpclient.MultiThreadedHttpConnectionManager$ConnectionPool r0 = r1.connectionPool     // Catch:{ all -> 0x004b }
            org.apache.commons.httpclient.HttpConnection r0 = r0.createConnection(r2)     // Catch:{ all -> 0x004b }
            r3 = r0
            goto L_0x0035
        L_0x0066:
            int r0 = r8.numConnections     // Catch:{ all -> 0x019b }
            if (r0 >= r4) goto L_0x0083
            org.apache.commons.httpclient.MultiThreadedHttpConnectionManager$ConnectionPool r0 = r1.connectionPool     // Catch:{ all -> 0x004b }
            java.util.LinkedList r0 = r0.freeConnections     // Catch:{ all -> 0x004b }
            int r0 = r0.size()     // Catch:{ all -> 0x004b }
            if (r0 <= 0) goto L_0x0083
            org.apache.commons.httpclient.MultiThreadedHttpConnectionManager$ConnectionPool r0 = r1.connectionPool     // Catch:{ all -> 0x004b }
            r0.deleteLeastUsedConnection()     // Catch:{ all -> 0x004b }
            org.apache.commons.httpclient.MultiThreadedHttpConnectionManager$ConnectionPool r0 = r1.connectionPool     // Catch:{ all -> 0x004b }
            org.apache.commons.httpclient.HttpConnection r0 = r0.createConnection(r2)     // Catch:{ all -> 0x004b }
            r3 = r0
            goto L_0x0035
        L_0x0083:
            if (r7 == 0) goto L_0x00b2
            r16 = 0
            int r0 = (r10 > r16 ? 1 : (r10 == r16 ? 0 : -1))
            if (r0 <= 0) goto L_0x008e
            r18 = r4
            goto L_0x00b6
        L_0x008e:
            org.apache.commons.httpclient.ConnectionPoolTimeoutException r0 = new org.apache.commons.httpclient.ConnectionPoolTimeoutException     // Catch:{ InterruptedException -> 0x00aa, all -> 0x00a3 }
            r18 = r4
            java.lang.String r4 = "Timeout waiting for connection"
            r0.<init>(r4)     // Catch:{ InterruptedException -> 0x009d, all -> 0x0098 }
            throw r0     // Catch:{ InterruptedException -> 0x009d, all -> 0x0098 }
        L_0x0098:
            r0 = move-exception
            r19 = r5
            goto L_0x016b
        L_0x009d:
            r0 = move-exception
            r19 = r5
        L_0x00a0:
            r4 = 0
            goto L_0x0132
        L_0x00a3:
            r0 = move-exception
            r18 = r4
            r19 = r5
            goto L_0x016b
        L_0x00aa:
            r0 = move-exception
            r18 = r4
            r19 = r5
            r4 = 0
            goto L_0x0132
        L_0x00b2:
            r18 = r4
            r16 = 0
        L_0x00b6:
            org.apache.commons.logging.Log r0 = LOG     // Catch:{ InterruptedException -> 0x012e, all -> 0x012a }
            boolean r0 = r0.isDebugEnabled()     // Catch:{ InterruptedException -> 0x012e, all -> 0x012a }
            if (r0 == 0) goto L_0x00d7
            org.apache.commons.logging.Log r0 = LOG     // Catch:{ InterruptedException -> 0x012e, all -> 0x012a }
            java.lang.StringBuffer r4 = new java.lang.StringBuffer     // Catch:{ InterruptedException -> 0x012e, all -> 0x012a }
            r4.<init>()     // Catch:{ InterruptedException -> 0x012e, all -> 0x012a }
            r19 = r5
            java.lang.String r5 = "Unable to get a connection, waiting..., hostConfig="
            r4.append(r5)     // Catch:{ InterruptedException -> 0x00ec }
            r4.append(r2)     // Catch:{ InterruptedException -> 0x00ec }
            java.lang.String r4 = r4.toString()     // Catch:{ InterruptedException -> 0x00ec }
            r0.debug(r4)     // Catch:{ InterruptedException -> 0x00ec }
            goto L_0x00d9
        L_0x00d7:
            r19 = r5
        L_0x00d9:
            if (r9 != 0) goto L_0x00ee
            org.apache.commons.httpclient.MultiThreadedHttpConnectionManager$WaitingThread r0 = new org.apache.commons.httpclient.MultiThreadedHttpConnectionManager$WaitingThread     // Catch:{ InterruptedException -> 0x00ec }
            r4 = 0
            r0.<init>()     // Catch:{ InterruptedException -> 0x00ec }
            r9 = r0
            r9.hostConnectionPool = r8     // Catch:{ InterruptedException -> 0x00ec }
            java.lang.Thread r0 = java.lang.Thread.currentThread()     // Catch:{ InterruptedException -> 0x00ec }
            r9.thread = r0     // Catch:{ InterruptedException -> 0x00ec }
            r4 = 0
            goto L_0x00f1
        L_0x00ec:
            r0 = move-exception
            goto L_0x00a0
        L_0x00ee:
            r4 = 0
            r9.interruptedByConnectionPool = r4     // Catch:{ InterruptedException -> 0x0128 }
        L_0x00f1:
            if (r7 == 0) goto L_0x00f9
            long r20 = java.lang.System.currentTimeMillis()     // Catch:{ InterruptedException -> 0x0128 }
            r14 = r20
        L_0x00f9:
            java.util.LinkedList r0 = r8.waitingThreads     // Catch:{ InterruptedException -> 0x0128 }
            r0.addLast(r9)     // Catch:{ InterruptedException -> 0x0128 }
            org.apache.commons.httpclient.MultiThreadedHttpConnectionManager$ConnectionPool r0 = r1.connectionPool     // Catch:{ InterruptedException -> 0x0128 }
            java.util.LinkedList r0 = r0.waitingThreads     // Catch:{ InterruptedException -> 0x0128 }
            r0.addLast(r9)     // Catch:{ InterruptedException -> 0x0128 }
            org.apache.commons.httpclient.MultiThreadedHttpConnectionManager$ConnectionPool r0 = r1.connectionPool     // Catch:{ InterruptedException -> 0x0128 }
            r0.wait(r10)     // Catch:{ InterruptedException -> 0x0128 }
            boolean r0 = r9.interruptedByConnectionPool     // Catch:{ all -> 0x01a2 }
            if (r0 != 0) goto L_0x011e
            java.util.LinkedList r0 = r8.waitingThreads     // Catch:{ all -> 0x01a2 }
            r0.remove(r9)     // Catch:{ all -> 0x01a2 }
            org.apache.commons.httpclient.MultiThreadedHttpConnectionManager$ConnectionPool r0 = r1.connectionPool     // Catch:{ all -> 0x01a2 }
            java.util.LinkedList r0 = r0.waitingThreads     // Catch:{ all -> 0x01a2 }
            r0.remove(r9)     // Catch:{ all -> 0x01a2 }
        L_0x011e:
            if (r7 == 0) goto L_0x0155
            long r20 = java.lang.System.currentTimeMillis()     // Catch:{ all -> 0x01a2 }
            r12 = r20
            r0 = 0
            goto L_0x0151
        L_0x0128:
            r0 = move-exception
            goto L_0x0132
        L_0x012a:
            r0 = move-exception
            r19 = r5
            goto L_0x016b
        L_0x012e:
            r0 = move-exception
            r19 = r5
            r4 = 0
        L_0x0132:
            boolean r5 = r9.interruptedByConnectionPool     // Catch:{ all -> 0x016a }
            if (r5 == 0) goto L_0x015b
            boolean r0 = r9.interruptedByConnectionPool     // Catch:{ all -> 0x01a2 }
            if (r0 != 0) goto L_0x0148
            java.util.LinkedList r0 = r8.waitingThreads     // Catch:{ all -> 0x01a2 }
            r0.remove(r9)     // Catch:{ all -> 0x01a2 }
            org.apache.commons.httpclient.MultiThreadedHttpConnectionManager$ConnectionPool r0 = r1.connectionPool     // Catch:{ all -> 0x01a2 }
            java.util.LinkedList r0 = r0.waitingThreads     // Catch:{ all -> 0x01a2 }
            r0.remove(r9)     // Catch:{ all -> 0x01a2 }
        L_0x0148:
            if (r7 == 0) goto L_0x0155
            long r20 = java.lang.System.currentTimeMillis()     // Catch:{ all -> 0x01a2 }
            r12 = r20
            r0 = 0
        L_0x0151:
            long r20 = r12 - r14
            long r10 = r10 - r20
        L_0x0155:
            r4 = r18
            r5 = r19
            goto L_0x0035
        L_0x015b:
            org.apache.commons.logging.Log r4 = LOG     // Catch:{ all -> 0x016a }
            java.lang.String r5 = "Interrupted while waiting for connection"
            r4.debug(r5, r0)     // Catch:{ all -> 0x016a }
            java.lang.IllegalThreadStateException r4 = new java.lang.IllegalThreadStateException     // Catch:{ all -> 0x016a }
            java.lang.String r5 = "Interrupted while waiting in MultiThreadedHttpConnectionManager"
            r4.<init>(r5)     // Catch:{ all -> 0x016a }
            throw r4     // Catch:{ all -> 0x016a }
        L_0x016a:
            r0 = move-exception
        L_0x016b:
            boolean r4 = r9.interruptedByConnectionPool     // Catch:{ all -> 0x01a2 }
            if (r4 != 0) goto L_0x017d
            java.util.LinkedList r4 = r8.waitingThreads     // Catch:{ all -> 0x01a2 }
            r4.remove(r9)     // Catch:{ all -> 0x01a2 }
            org.apache.commons.httpclient.MultiThreadedHttpConnectionManager$ConnectionPool r4 = r1.connectionPool     // Catch:{ all -> 0x01a2 }
            java.util.LinkedList r4 = r4.waitingThreads     // Catch:{ all -> 0x01a2 }
            r4.remove(r9)     // Catch:{ all -> 0x01a2 }
        L_0x017d:
            if (r7 == 0) goto L_0x0188
            long r4 = java.lang.System.currentTimeMillis()     // Catch:{ all -> 0x01a2 }
            r12 = r4
            r4 = 0
            long r4 = r12 - r14
            long r10 = r10 - r4
        L_0x0188:
            throw r0     // Catch:{ all -> 0x01a2 }
        L_0x0189:
            r18 = r4
            r19 = r5
            java.lang.IllegalStateException r0 = new java.lang.IllegalStateException     // Catch:{ all -> 0x01a2 }
            java.lang.String r4 = "Connection factory has been shutdown."
            r0.<init>(r4)     // Catch:{ all -> 0x01a2 }
            throw r0     // Catch:{ all -> 0x01a2 }
        L_0x0195:
            r18 = r4
            r19 = r5
            monitor-exit(r6)     // Catch:{ all -> 0x01a2 }
            return r3
        L_0x019b:
            r0 = move-exception
            r18 = r4
            r19 = r5
        L_0x01a0:
            monitor-exit(r6)     // Catch:{ all -> 0x01a2 }
            throw r0
        L_0x01a2:
            r0 = move-exception
            goto L_0x01a0
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.httpclient.MultiThreadedHttpConnectionManager.doGetConnection(org.apache.commons.httpclient.HostConfiguration, long):org.apache.commons.httpclient.HttpConnection");
    }

    public int getConnectionsInPool(HostConfiguration hostConfiguration) {
        int i;
        synchronized (this.connectionPool) {
            i = 0;
            HostConnectionPool hostPool = this.connectionPool.getHostPool(hostConfiguration, false);
            if (hostPool != null) {
                i = hostPool.numConnections;
            }
        }
        return i;
    }

    public int getConnectionsInPool() {
        int access$200;
        synchronized (this.connectionPool) {
            access$200 = this.connectionPool.numConnections;
        }
        return access$200;
    }

    public int getConnectionsInUse(HostConfiguration hostConfiguration) {
        return getConnectionsInPool(hostConfiguration);
    }

    public int getConnectionsInUse() {
        return getConnectionsInPool();
    }

    public void deleteClosedConnections() {
        this.connectionPool.deleteClosedConnections();
    }

    public void closeIdleConnections(long idleTimeout) {
        this.connectionPool.closeIdleConnections(idleTimeout);
        deleteClosedConnections();
    }

    public void releaseConnection(HttpConnection conn) {
        LOG.trace("enter HttpConnectionManager.releaseConnection(HttpConnection)");
        if (conn instanceof HttpConnectionAdapter) {
            conn = ((HttpConnectionAdapter) conn).getWrappedConnection();
        }
        SimpleHttpConnectionManager.finishLastResponse(conn);
        this.connectionPool.freeConnection(conn);
    }

    /* access modifiers changed from: private */
    public HostConfiguration configurationForConnection(HttpConnection conn) {
        HostConfiguration connectionConfiguration = new HostConfiguration();
        connectionConfiguration.setHost(conn.getHost(), conn.getPort(), conn.getProtocol());
        if (conn.getLocalAddress() != null) {
            connectionConfiguration.setLocalAddress(conn.getLocalAddress());
        }
        if (conn.getProxyHost() != null) {
            connectionConfiguration.setProxy(conn.getProxyHost(), conn.getProxyPort());
        }
        return connectionConfiguration;
    }

    public HttpConnectionManagerParams getParams() {
        return this.params;
    }

    public void setParams(HttpConnectionManagerParams params2) {
        if (params2 != null) {
            this.params = params2;
            return;
        }
        throw new IllegalArgumentException("Parameters may not be null");
    }

    private class ConnectionPool {
        /* access modifiers changed from: private */
        public LinkedList freeConnections;
        private IdleConnectionHandler idleConnectionHandler;
        private final Map mapHosts;
        /* access modifiers changed from: private */
        public int numConnections;
        /* access modifiers changed from: private */
        public LinkedList waitingThreads;

        private ConnectionPool() {
            this.freeConnections = new LinkedList();
            this.waitingThreads = new LinkedList();
            this.mapHosts = new HashMap();
            this.idleConnectionHandler = new IdleConnectionHandler();
            this.numConnections = 0;
        }

        public synchronized void shutdown() {
            Iterator iter = this.freeConnections.iterator();
            while (iter.hasNext()) {
                iter.remove();
                ((HttpConnection) iter.next()).close();
            }
            MultiThreadedHttpConnectionManager.shutdownCheckedOutConnections(this);
            Iterator iter2 = this.waitingThreads.iterator();
            while (iter2.hasNext()) {
                WaitingThread waiter = (WaitingThread) iter2.next();
                iter2.remove();
                waiter.interruptedByConnectionPool = true;
                waiter.thread.interrupt();
            }
            this.mapHosts.clear();
            this.idleConnectionHandler.removeAll();
        }

        public synchronized HttpConnection createConnection(HostConfiguration hostConfiguration) {
            HttpConnectionWithReference connection;
            HostConnectionPool hostPool = getHostPool(hostConfiguration, true);
            if (MultiThreadedHttpConnectionManager.LOG.isDebugEnabled()) {
                Log access$700 = MultiThreadedHttpConnectionManager.LOG;
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append("Allocating new connection, hostConfig=");
                stringBuffer.append(hostConfiguration);
                access$700.debug(stringBuffer.toString());
            }
            connection = new HttpConnectionWithReference(hostConfiguration);
            connection.getParams().setDefaults(MultiThreadedHttpConnectionManager.this.params);
            connection.setHttpConnectionManager(MultiThreadedHttpConnectionManager.this);
            this.numConnections++;
            hostPool.numConnections++;
            MultiThreadedHttpConnectionManager.storeReferenceToConnection(connection, hostConfiguration, this);
            return connection;
        }

        public synchronized void handleLostConnection(HostConfiguration config) {
            HostConnectionPool hostPool = getHostPool(config, true);
            hostPool.numConnections--;
            if (hostPool.numConnections == 0 && hostPool.waitingThreads.isEmpty()) {
                this.mapHosts.remove(config);
            }
            this.numConnections--;
            notifyWaitingThread(config);
        }

        public synchronized HostConnectionPool getHostPool(HostConfiguration hostConfiguration, boolean create) {
            HostConnectionPool listConnections;
            MultiThreadedHttpConnectionManager.LOG.trace("enter HttpConnectionManager.ConnectionPool.getHostPool(HostConfiguration)");
            listConnections = (HostConnectionPool) this.mapHosts.get(hostConfiguration);
            if (listConnections == null && create) {
                listConnections = new HostConnectionPool();
                listConnections.hostConfiguration = hostConfiguration;
                this.mapHosts.put(hostConfiguration, listConnections);
            }
            return listConnections;
        }

        public synchronized HttpConnection getFreeConnection(HostConfiguration hostConfiguration) {
            HttpConnectionWithReference connection;
            connection = null;
            HostConnectionPool hostPool = getHostPool(hostConfiguration, false);
            if (hostPool != null && hostPool.freeConnections.size() > 0) {
                connection = (HttpConnectionWithReference) hostPool.freeConnections.removeLast();
                this.freeConnections.remove(connection);
                MultiThreadedHttpConnectionManager.storeReferenceToConnection(connection, hostConfiguration, this);
                if (MultiThreadedHttpConnectionManager.LOG.isDebugEnabled()) {
                    Log access$700 = MultiThreadedHttpConnectionManager.LOG;
                    StringBuffer stringBuffer = new StringBuffer();
                    stringBuffer.append("Getting free connection, hostConfig=");
                    stringBuffer.append(hostConfiguration);
                    access$700.debug(stringBuffer.toString());
                }
                this.idleConnectionHandler.remove(connection);
            } else if (MultiThreadedHttpConnectionManager.LOG.isDebugEnabled()) {
                Log access$7002 = MultiThreadedHttpConnectionManager.LOG;
                StringBuffer stringBuffer2 = new StringBuffer();
                stringBuffer2.append("There were no free connections to get, hostConfig=");
                stringBuffer2.append(hostConfiguration);
                access$7002.debug(stringBuffer2.toString());
            }
            return connection;
        }

        public synchronized void deleteClosedConnections() {
            Iterator iter = this.freeConnections.iterator();
            while (iter.hasNext()) {
                HttpConnection conn = (HttpConnection) iter.next();
                if (!conn.isOpen()) {
                    iter.remove();
                    deleteConnection(conn);
                }
            }
        }

        public synchronized void closeIdleConnections(long idleTimeout) {
            this.idleConnectionHandler.closeIdleConnections(idleTimeout);
        }

        private synchronized void deleteConnection(HttpConnection connection) {
            HostConfiguration connectionConfiguration = MultiThreadedHttpConnectionManager.this.configurationForConnection(connection);
            if (MultiThreadedHttpConnectionManager.LOG.isDebugEnabled()) {
                Log access$700 = MultiThreadedHttpConnectionManager.LOG;
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append("Reclaiming connection, hostConfig=");
                stringBuffer.append(connectionConfiguration);
                access$700.debug(stringBuffer.toString());
            }
            connection.close();
            HostConnectionPool hostPool = getHostPool(connectionConfiguration, true);
            hostPool.freeConnections.remove(connection);
            hostPool.numConnections--;
            this.numConnections--;
            if (hostPool.numConnections == 0 && hostPool.waitingThreads.isEmpty()) {
                this.mapHosts.remove(connectionConfiguration);
            }
            this.idleConnectionHandler.remove(connection);
        }

        public synchronized void deleteLeastUsedConnection() {
            HttpConnection connection = (HttpConnection) this.freeConnections.removeFirst();
            if (connection != null) {
                deleteConnection(connection);
            } else if (MultiThreadedHttpConnectionManager.LOG.isDebugEnabled()) {
                MultiThreadedHttpConnectionManager.LOG.debug("Attempted to reclaim an unused connection but there were none.");
            }
        }

        public synchronized void notifyWaitingThread(HostConfiguration configuration) {
            notifyWaitingThread(getHostPool(configuration, true));
        }

        public synchronized void notifyWaitingThread(HostConnectionPool hostPool) {
            WaitingThread waitingThread = null;
            if (hostPool.waitingThreads.size() > 0) {
                if (MultiThreadedHttpConnectionManager.LOG.isDebugEnabled()) {
                    Log access$700 = MultiThreadedHttpConnectionManager.LOG;
                    StringBuffer stringBuffer = new StringBuffer();
                    stringBuffer.append("Notifying thread waiting on host pool, hostConfig=");
                    stringBuffer.append(hostPool.hostConfiguration);
                    access$700.debug(stringBuffer.toString());
                }
                waitingThread = (WaitingThread) hostPool.waitingThreads.removeFirst();
                this.waitingThreads.remove(waitingThread);
            } else if (this.waitingThreads.size() > 0) {
                if (MultiThreadedHttpConnectionManager.LOG.isDebugEnabled()) {
                    MultiThreadedHttpConnectionManager.LOG.debug("No-one waiting on host pool, notifying next waiting thread.");
                }
                waitingThread = (WaitingThread) this.waitingThreads.removeFirst();
                waitingThread.hostConnectionPool.waitingThreads.remove(waitingThread);
            } else if (MultiThreadedHttpConnectionManager.LOG.isDebugEnabled()) {
                MultiThreadedHttpConnectionManager.LOG.debug("Notifying no-one, there are no waiting threads");
            }
            if (waitingThread != null) {
                waitingThread.interruptedByConnectionPool = true;
                waitingThread.thread.interrupt();
            }
        }

        public void freeConnection(HttpConnection conn) {
            HostConfiguration connectionConfiguration = MultiThreadedHttpConnectionManager.this.configurationForConnection(conn);
            if (MultiThreadedHttpConnectionManager.LOG.isDebugEnabled()) {
                Log access$700 = MultiThreadedHttpConnectionManager.LOG;
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append("Freeing connection, hostConfig=");
                stringBuffer.append(connectionConfiguration);
                access$700.debug(stringBuffer.toString());
            }
            synchronized (this) {
                if (MultiThreadedHttpConnectionManager.this.shutdown) {
                    conn.close();
                    return;
                }
                HostConnectionPool hostPool = getHostPool(connectionConfiguration, true);
                hostPool.freeConnections.add(conn);
                if (hostPool.numConnections == 0) {
                    Log access$7002 = MultiThreadedHttpConnectionManager.LOG;
                    StringBuffer stringBuffer2 = new StringBuffer();
                    stringBuffer2.append("Host connection pool not found, hostConfig=");
                    stringBuffer2.append(connectionConfiguration);
                    access$7002.error(stringBuffer2.toString());
                    hostPool.numConnections = 1;
                }
                this.freeConnections.add(conn);
                MultiThreadedHttpConnectionManager.removeReferenceToConnection((HttpConnectionWithReference) conn);
                if (this.numConnections == 0) {
                    Log access$7003 = MultiThreadedHttpConnectionManager.LOG;
                    StringBuffer stringBuffer3 = new StringBuffer();
                    stringBuffer3.append("Host connection pool not found, hostConfig=");
                    stringBuffer3.append(connectionConfiguration);
                    access$7003.error(stringBuffer3.toString());
                    this.numConnections = 1;
                }
                this.idleConnectionHandler.add(conn);
                notifyWaitingThread(hostPool);
            }
        }
    }

    private static class ConnectionSource {
        public ConnectionPool connectionPool;
        public HostConfiguration hostConfiguration;

        private ConnectionSource() {
        }
    }

    private static class HostConnectionPool {
        public LinkedList freeConnections;
        public HostConfiguration hostConfiguration;
        public int numConnections;
        public LinkedList waitingThreads;

        private HostConnectionPool() {
            this.freeConnections = new LinkedList();
            this.waitingThreads = new LinkedList();
            this.numConnections = 0;
        }
    }

    private static class WaitingThread {
        public HostConnectionPool hostConnectionPool;
        public boolean interruptedByConnectionPool;
        public Thread thread;

        private WaitingThread() {
            this.interruptedByConnectionPool = false;
        }
    }

    private static class ReferenceQueueThread extends Thread {
        private volatile boolean shutdown = false;

        public ReferenceQueueThread() {
            setDaemon(true);
            setName("MultiThreadedHttpConnectionManager cleanup");
        }

        public void shutdown() {
            this.shutdown = true;
            interrupt();
        }

        private void handleReference(Reference ref) {
            ConnectionSource source;
            synchronized (MultiThreadedHttpConnectionManager.REFERENCE_TO_CONNECTION_SOURCE) {
                source = (ConnectionSource) MultiThreadedHttpConnectionManager.REFERENCE_TO_CONNECTION_SOURCE.remove(ref);
            }
            if (source != null) {
                if (MultiThreadedHttpConnectionManager.LOG.isDebugEnabled()) {
                    Log access$700 = MultiThreadedHttpConnectionManager.LOG;
                    StringBuffer stringBuffer = new StringBuffer();
                    stringBuffer.append("Connection reclaimed by garbage collector, hostConfig=");
                    stringBuffer.append(source.hostConfiguration);
                    access$700.debug(stringBuffer.toString());
                }
                source.connectionPool.handleLostConnection(source.hostConfiguration);
            }
        }

        public void run() {
            while (!this.shutdown) {
                try {
                    Reference ref = MultiThreadedHttpConnectionManager.REFERENCE_QUEUE.remove();
                    if (ref != null) {
                        handleReference(ref);
                    }
                } catch (InterruptedException e) {
                    MultiThreadedHttpConnectionManager.LOG.debug("ReferenceQueueThread interrupted", e);
                }
            }
        }
    }

    private static class HttpConnectionWithReference extends HttpConnection {
        public WeakReference reference = new WeakReference(this, MultiThreadedHttpConnectionManager.REFERENCE_QUEUE);

        public HttpConnectionWithReference(HostConfiguration hostConfiguration) {
            super(hostConfiguration);
        }
    }

    private static class HttpConnectionAdapter extends HttpConnection {
        private HttpConnection wrappedConnection;

        public HttpConnectionAdapter(HttpConnection connection) {
            super(connection.getHost(), connection.getPort(), connection.getProtocol());
            this.wrappedConnection = connection;
        }

        /* access modifiers changed from: protected */
        public boolean hasConnection() {
            return this.wrappedConnection != null;
        }

        /* access modifiers changed from: package-private */
        public HttpConnection getWrappedConnection() {
            return this.wrappedConnection;
        }

        public void close() {
            if (hasConnection()) {
                this.wrappedConnection.close();
            }
        }

        public InetAddress getLocalAddress() {
            if (hasConnection()) {
                return this.wrappedConnection.getLocalAddress();
            }
            return null;
        }

        public boolean isStaleCheckingEnabled() {
            if (hasConnection()) {
                return this.wrappedConnection.isStaleCheckingEnabled();
            }
            return false;
        }

        public void setLocalAddress(InetAddress localAddress) {
            if (hasConnection()) {
                this.wrappedConnection.setLocalAddress(localAddress);
                return;
            }
            throw new IllegalStateException("Connection has been released");
        }

        public void setStaleCheckingEnabled(boolean staleCheckEnabled) {
            if (hasConnection()) {
                this.wrappedConnection.setStaleCheckingEnabled(staleCheckEnabled);
                return;
            }
            throw new IllegalStateException("Connection has been released");
        }

        public String getHost() {
            if (hasConnection()) {
                return this.wrappedConnection.getHost();
            }
            return null;
        }

        public HttpConnectionManager getHttpConnectionManager() {
            if (hasConnection()) {
                return this.wrappedConnection.getHttpConnectionManager();
            }
            return null;
        }

        public InputStream getLastResponseInputStream() {
            if (hasConnection()) {
                return this.wrappedConnection.getLastResponseInputStream();
            }
            return null;
        }

        public int getPort() {
            if (hasConnection()) {
                return this.wrappedConnection.getPort();
            }
            return -1;
        }

        public Protocol getProtocol() {
            if (hasConnection()) {
                return this.wrappedConnection.getProtocol();
            }
            return null;
        }

        public String getProxyHost() {
            if (hasConnection()) {
                return this.wrappedConnection.getProxyHost();
            }
            return null;
        }

        public int getProxyPort() {
            if (hasConnection()) {
                return this.wrappedConnection.getProxyPort();
            }
            return -1;
        }

        public OutputStream getRequestOutputStream() throws IOException, IllegalStateException {
            if (hasConnection()) {
                return this.wrappedConnection.getRequestOutputStream();
            }
            return null;
        }

        public InputStream getResponseInputStream() throws IOException, IllegalStateException {
            if (hasConnection()) {
                return this.wrappedConnection.getResponseInputStream();
            }
            return null;
        }

        public boolean isOpen() {
            if (hasConnection()) {
                return this.wrappedConnection.isOpen();
            }
            return false;
        }

        public boolean closeIfStale() throws IOException {
            if (hasConnection()) {
                return this.wrappedConnection.closeIfStale();
            }
            return false;
        }

        public boolean isProxied() {
            if (hasConnection()) {
                return this.wrappedConnection.isProxied();
            }
            return false;
        }

        public boolean isResponseAvailable() throws IOException {
            if (hasConnection()) {
                return this.wrappedConnection.isResponseAvailable();
            }
            return false;
        }

        public boolean isResponseAvailable(int timeout) throws IOException {
            if (hasConnection()) {
                return this.wrappedConnection.isResponseAvailable(timeout);
            }
            return false;
        }

        public boolean isSecure() {
            if (hasConnection()) {
                return this.wrappedConnection.isSecure();
            }
            return false;
        }

        public boolean isTransparent() {
            if (hasConnection()) {
                return this.wrappedConnection.isTransparent();
            }
            return false;
        }

        public void open() throws IOException {
            if (hasConnection()) {
                this.wrappedConnection.open();
                return;
            }
            throw new IllegalStateException("Connection has been released");
        }

        public void print(String data) throws IOException, IllegalStateException {
            if (hasConnection()) {
                this.wrappedConnection.print(data);
                return;
            }
            throw new IllegalStateException("Connection has been released");
        }

        public void printLine() throws IOException, IllegalStateException {
            if (hasConnection()) {
                this.wrappedConnection.printLine();
                return;
            }
            throw new IllegalStateException("Connection has been released");
        }

        public void printLine(String data) throws IOException, IllegalStateException {
            if (hasConnection()) {
                this.wrappedConnection.printLine(data);
                return;
            }
            throw new IllegalStateException("Connection has been released");
        }

        public String readLine() throws IOException, IllegalStateException {
            if (hasConnection()) {
                return this.wrappedConnection.readLine();
            }
            throw new IllegalStateException("Connection has been released");
        }

        public String readLine(String charset) throws IOException, IllegalStateException {
            if (hasConnection()) {
                return this.wrappedConnection.readLine(charset);
            }
            throw new IllegalStateException("Connection has been released");
        }

        public void releaseConnection() {
            if (!isLocked() && hasConnection()) {
                HttpConnection wrappedConnection2 = this.wrappedConnection;
                this.wrappedConnection = null;
                wrappedConnection2.releaseConnection();
            }
        }

        public void setConnectionTimeout(int timeout) {
            if (hasConnection()) {
                this.wrappedConnection.setConnectionTimeout(timeout);
            }
        }

        public void setHost(String host) throws IllegalStateException {
            if (hasConnection()) {
                this.wrappedConnection.setHost(host);
            }
        }

        public void setHttpConnectionManager(HttpConnectionManager httpConnectionManager) {
            if (hasConnection()) {
                this.wrappedConnection.setHttpConnectionManager(httpConnectionManager);
            }
        }

        public void setLastResponseInputStream(InputStream inStream) {
            if (hasConnection()) {
                this.wrappedConnection.setLastResponseInputStream(inStream);
            }
        }

        public void setPort(int port) throws IllegalStateException {
            if (hasConnection()) {
                this.wrappedConnection.setPort(port);
            }
        }

        public void setProtocol(Protocol protocol) {
            if (hasConnection()) {
                this.wrappedConnection.setProtocol(protocol);
            }
        }

        public void setProxyHost(String host) throws IllegalStateException {
            if (hasConnection()) {
                this.wrappedConnection.setProxyHost(host);
            }
        }

        public void setProxyPort(int port) throws IllegalStateException {
            if (hasConnection()) {
                this.wrappedConnection.setProxyPort(port);
            }
        }

        public void setSoTimeout(int timeout) throws SocketException, IllegalStateException {
            if (hasConnection()) {
                this.wrappedConnection.setSoTimeout(timeout);
            }
        }

        public void shutdownOutput() {
            if (hasConnection()) {
                this.wrappedConnection.shutdownOutput();
            }
        }

        public void tunnelCreated() throws IllegalStateException, IOException {
            if (hasConnection()) {
                this.wrappedConnection.tunnelCreated();
            }
        }

        public void write(byte[] data, int offset, int length) throws IOException, IllegalStateException {
            if (hasConnection()) {
                this.wrappedConnection.write(data, offset, length);
                return;
            }
            throw new IllegalStateException("Connection has been released");
        }

        public void write(byte[] data) throws IOException, IllegalStateException {
            if (hasConnection()) {
                this.wrappedConnection.write(data);
                return;
            }
            throw new IllegalStateException("Connection has been released");
        }

        public void writeLine() throws IOException, IllegalStateException {
            if (hasConnection()) {
                this.wrappedConnection.writeLine();
                return;
            }
            throw new IllegalStateException("Connection has been released");
        }

        public void writeLine(byte[] data) throws IOException, IllegalStateException {
            if (hasConnection()) {
                this.wrappedConnection.writeLine(data);
                return;
            }
            throw new IllegalStateException("Connection has been released");
        }

        public void flushRequestOutputStream() throws IOException {
            if (hasConnection()) {
                this.wrappedConnection.flushRequestOutputStream();
                return;
            }
            throw new IllegalStateException("Connection has been released");
        }

        public int getSoTimeout() throws SocketException {
            if (hasConnection()) {
                return this.wrappedConnection.getSoTimeout();
            }
            throw new IllegalStateException("Connection has been released");
        }

        public String getVirtualHost() {
            if (hasConnection()) {
                return this.wrappedConnection.getVirtualHost();
            }
            throw new IllegalStateException("Connection has been released");
        }

        public void setVirtualHost(String host) throws IllegalStateException {
            if (hasConnection()) {
                this.wrappedConnection.setVirtualHost(host);
                return;
            }
            throw new IllegalStateException("Connection has been released");
        }

        public int getSendBufferSize() throws SocketException {
            if (hasConnection()) {
                return this.wrappedConnection.getSendBufferSize();
            }
            throw new IllegalStateException("Connection has been released");
        }

        public void setSendBufferSize(int sendBufferSize) throws SocketException {
            if (hasConnection()) {
                this.wrappedConnection.setSendBufferSize(sendBufferSize);
                return;
            }
            throw new IllegalStateException("Connection has been released");
        }

        public HttpConnectionParams getParams() {
            if (hasConnection()) {
                return this.wrappedConnection.getParams();
            }
            throw new IllegalStateException("Connection has been released");
        }

        public void setParams(HttpConnectionParams params) {
            if (hasConnection()) {
                this.wrappedConnection.setParams(params);
                return;
            }
            throw new IllegalStateException("Connection has been released");
        }

        public void print(String data, String charset) throws IOException, IllegalStateException {
            if (hasConnection()) {
                this.wrappedConnection.print(data, charset);
                return;
            }
            throw new IllegalStateException("Connection has been released");
        }

        public void printLine(String data, String charset) throws IOException, IllegalStateException {
            if (hasConnection()) {
                this.wrappedConnection.printLine(data, charset);
                return;
            }
            throw new IllegalStateException("Connection has been released");
        }

        public void setSocketTimeout(int timeout) throws SocketException, IllegalStateException {
            if (hasConnection()) {
                this.wrappedConnection.setSocketTimeout(timeout);
                return;
            }
            throw new IllegalStateException("Connection has been released");
        }
    }
}
