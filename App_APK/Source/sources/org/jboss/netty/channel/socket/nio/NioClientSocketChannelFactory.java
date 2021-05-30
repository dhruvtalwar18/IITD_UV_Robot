package org.jboss.netty.channel.socket.nio;

import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import org.jboss.netty.channel.ChannelPipeline;
import org.jboss.netty.channel.socket.ClientSocketChannelFactory;
import org.jboss.netty.channel.socket.SocketChannel;
import org.jboss.netty.util.ExternalResourceReleasable;
import org.jboss.netty.util.internal.ExecutorUtil;

public class NioClientSocketChannelFactory implements ClientSocketChannelFactory {
    private static final int DEFAULT_BOSS_COUNT = 1;
    private final Executor bossExecutor;
    private final NioClientSocketPipelineSink sink;
    private final WorkerPool<NioWorker> workerPool;

    public NioClientSocketChannelFactory() {
        this(Executors.newCachedThreadPool(), Executors.newCachedThreadPool());
    }

    public NioClientSocketChannelFactory(Executor bossExecutor2, Executor workerExecutor) {
        this(bossExecutor2, workerExecutor, 1, SelectorUtil.DEFAULT_IO_THREADS);
    }

    public NioClientSocketChannelFactory(Executor bossExecutor2, Executor workerExecutor, int workerCount) {
        this(bossExecutor2, workerExecutor, 1, workerCount);
    }

    public NioClientSocketChannelFactory(Executor bossExecutor2, Executor workerExecutor, int bossCount, int workerCount) {
        this(bossExecutor2, bossCount, (WorkerPool<NioWorker>) new NioWorkerPool(workerExecutor, workerCount, true));
    }

    public NioClientSocketChannelFactory(Executor bossExecutor2, int bossCount, WorkerPool<NioWorker> workerPool2) {
        if (bossExecutor2 == null) {
            throw new NullPointerException("bossExecutor");
        } else if (workerPool2 == null) {
            throw new NullPointerException("workerPool");
        } else if (bossCount > 0) {
            this.bossExecutor = bossExecutor2;
            this.workerPool = workerPool2;
            this.sink = new NioClientSocketPipelineSink(bossExecutor2, bossCount, workerPool2);
        } else {
            throw new IllegalArgumentException("bossCount (" + bossCount + ") " + "must be a positive integer.");
        }
    }

    public SocketChannel newChannel(ChannelPipeline pipeline) {
        return new NioClientSocketChannel(this, pipeline, this.sink, this.sink.nextWorker());
    }

    public void releaseExternalResources() {
        ExecutorUtil.terminate(this.bossExecutor);
        if (this.workerPool instanceof ExternalResourceReleasable) {
            ((ExternalResourceReleasable) this.workerPool).releaseExternalResources();
        }
    }
}
