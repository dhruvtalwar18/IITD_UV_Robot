package org.jboss.netty.channel.socket.nio;

import java.util.concurrent.Executor;
import java.util.concurrent.atomic.AtomicInteger;
import org.jboss.netty.channel.socket.nio.AbstractNioWorker;
import org.jboss.netty.util.ExternalResourceReleasable;
import org.jboss.netty.util.internal.ExecutorUtil;

public abstract class AbstractNioWorkerPool<E extends AbstractNioWorker> implements WorkerPool<E>, ExternalResourceReleasable {
    private final Executor workerExecutor;
    private final AtomicInteger workerIndex = new AtomicInteger();
    private final AbstractNioWorker[] workers;

    /* access modifiers changed from: protected */
    public abstract E createWorker(Executor executor, boolean z);

    AbstractNioWorkerPool(Executor workerExecutor2, int workerCount, boolean allowShutDownOnIdle) {
        if (workerExecutor2 == null) {
            throw new NullPointerException("workerExecutor");
        } else if (workerCount > 0) {
            this.workers = new AbstractNioWorker[workerCount];
            for (int i = 0; i < this.workers.length; i++) {
                this.workers[i] = createWorker(workerExecutor2, allowShutDownOnIdle);
            }
            this.workerExecutor = workerExecutor2;
        } else {
            throw new IllegalArgumentException("workerCount (" + workerCount + ") " + "must be a positive integer.");
        }
    }

    public E nextWorker() {
        return this.workers[Math.abs(this.workerIndex.getAndIncrement() % this.workers.length)];
    }

    public void releaseExternalResources() {
        ExecutorUtil.terminate(this.workerExecutor);
    }
}
