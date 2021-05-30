package org.apache.xmlrpc.common;

import java.util.ArrayList;
import java.util.List;

public abstract class XmlRpcWorkerFactory {
    private final XmlRpcController controller;
    private int numThreads;
    private final List pool = new ArrayList();
    private final XmlRpcWorker singleton = newWorker();

    /* access modifiers changed from: protected */
    public abstract XmlRpcWorker newWorker();

    public XmlRpcWorkerFactory(XmlRpcController pController) {
        this.controller = pController;
    }

    public XmlRpcController getController() {
        return this.controller;
    }

    public synchronized XmlRpcWorker getWorker() throws XmlRpcLoadException {
        int max = this.controller.getMaxThreads();
        if (max > 0) {
            if (this.numThreads == max) {
                throw new XmlRpcLoadException("Maximum number of concurrent requests exceeded: " + max);
            }
        }
        if (max == 0) {
            return this.singleton;
        }
        this.numThreads++;
        if (this.pool.size() == 0) {
            return newWorker();
        }
        return (XmlRpcWorker) this.pool.remove(this.pool.size() - 1);
    }

    public synchronized void releaseWorker(XmlRpcWorker pWorker) {
        this.numThreads--;
        int max = this.controller.getMaxThreads();
        if (pWorker != this.singleton) {
            if (this.pool.size() < max) {
                this.pool.add(pWorker);
            }
        }
    }

    public synchronized int getCurrentRequests() {
        return this.numThreads;
    }
}
