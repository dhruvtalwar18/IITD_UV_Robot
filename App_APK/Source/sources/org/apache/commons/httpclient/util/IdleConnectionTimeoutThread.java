package org.apache.commons.httpclient.util;

import java.util.ArrayList;
import java.util.List;
import org.apache.commons.httpclient.HttpConnectionManager;

public class IdleConnectionTimeoutThread extends Thread {
    private List connectionManagers = new ArrayList();
    private long connectionTimeout = 3000;
    private boolean shutdown = false;
    private long timeoutInterval = 1000;

    public IdleConnectionTimeoutThread() {
        setDaemon(true);
    }

    public synchronized void addConnectionManager(HttpConnectionManager connectionManager) {
        if (!this.shutdown) {
            this.connectionManagers.add(connectionManager);
        } else {
            throw new IllegalStateException("IdleConnectionTimeoutThread has been shutdown");
        }
    }

    public synchronized void removeConnectionManager(HttpConnectionManager connectionManager) {
        if (!this.shutdown) {
            this.connectionManagers.remove(connectionManager);
        } else {
            throw new IllegalStateException("IdleConnectionTimeoutThread has been shutdown");
        }
    }

    /* access modifiers changed from: protected */
    public void handleCloseIdleConnections(HttpConnectionManager connectionManager) {
        connectionManager.closeIdleConnections(this.connectionTimeout);
    }

    public synchronized void run() {
        while (!this.shutdown) {
            for (HttpConnectionManager connectionManager : this.connectionManagers) {
                handleCloseIdleConnections(connectionManager);
            }
            try {
                wait(this.timeoutInterval);
            } catch (InterruptedException e) {
            }
        }
        this.connectionManagers.clear();
    }

    public synchronized void shutdown() {
        this.shutdown = true;
        notifyAll();
    }

    public synchronized void setConnectionTimeout(long connectionTimeout2) {
        if (!this.shutdown) {
            this.connectionTimeout = connectionTimeout2;
        } else {
            throw new IllegalStateException("IdleConnectionTimeoutThread has been shutdown");
        }
    }

    public synchronized void setTimeoutInterval(long timeoutInterval2) {
        if (!this.shutdown) {
            this.timeoutInterval = timeoutInterval2;
        } else {
            throw new IllegalStateException("IdleConnectionTimeoutThread has been shutdown");
        }
    }
}
