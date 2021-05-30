package org.jboss.netty.channel.socket.nio;

import java.util.concurrent.Executor;

public class NioDatagramWorkerPool extends AbstractNioWorkerPool<NioDatagramWorker> {
    public NioDatagramWorkerPool(Executor executor, int workerCount, boolean allowShutdownOnIdle) {
        super(executor, workerCount, allowShutdownOnIdle);
    }

    /* access modifiers changed from: protected */
    public NioDatagramWorker createWorker(Executor executor, boolean allowShutdownOnIdle) {
        return new NioDatagramWorker(executor, allowShutdownOnIdle);
    }
}
