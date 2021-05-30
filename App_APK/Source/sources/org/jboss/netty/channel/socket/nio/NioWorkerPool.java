package org.jboss.netty.channel.socket.nio;

import java.util.concurrent.Executor;

public class NioWorkerPool extends AbstractNioWorkerPool<NioWorker> {
    public NioWorkerPool(Executor executor, int workerCount, boolean allowShutdownOnIdle) {
        super(executor, workerCount, allowShutdownOnIdle);
    }

    /* access modifiers changed from: protected */
    public NioWorker createWorker(Executor executor, boolean allowShutdownOnIdle) {
        return new NioWorker(executor, allowShutdownOnIdle);
    }
}
