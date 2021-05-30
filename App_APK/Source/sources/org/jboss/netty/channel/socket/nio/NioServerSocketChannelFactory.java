package org.jboss.netty.channel.socket.nio;

import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import org.jboss.netty.channel.ChannelPipeline;
import org.jboss.netty.channel.ChannelSink;
import org.jboss.netty.channel.socket.ServerSocketChannel;
import org.jboss.netty.channel.socket.ServerSocketChannelFactory;
import org.jboss.netty.util.ExternalResourceReleasable;
import org.jboss.netty.util.internal.ExecutorUtil;

public class NioServerSocketChannelFactory implements ServerSocketChannelFactory {
    final Executor bossExecutor;
    private final ChannelSink sink;
    private final WorkerPool<NioWorker> workerPool;

    public NioServerSocketChannelFactory() {
        this((Executor) Executors.newCachedThreadPool(), (Executor) Executors.newCachedThreadPool());
    }

    public NioServerSocketChannelFactory(Executor bossExecutor2, Executor workerExecutor) {
        this(bossExecutor2, workerExecutor, SelectorUtil.DEFAULT_IO_THREADS);
    }

    public NioServerSocketChannelFactory(Executor bossExecutor2, Executor workerExecutor, int workerCount) {
        this(bossExecutor2, (WorkerPool<NioWorker>) new NioWorkerPool(workerExecutor, workerCount, true));
    }

    public NioServerSocketChannelFactory(Executor bossExecutor2, WorkerPool<NioWorker> workerPool2) {
        if (bossExecutor2 == null) {
            throw new NullPointerException("bossExecutor");
        } else if (workerPool2 != null) {
            this.bossExecutor = bossExecutor2;
            this.workerPool = workerPool2;
            this.sink = new NioServerSocketPipelineSink(workerPool2);
        } else {
            throw new NullPointerException("workerPool");
        }
    }

    public ServerSocketChannel newChannel(ChannelPipeline pipeline) {
        return new NioServerSocketChannel(this, pipeline, this.sink);
    }

    public void releaseExternalResources() {
        ExecutorUtil.terminate(this.bossExecutor);
        if (this.workerPool instanceof ExternalResourceReleasable) {
            ((ExternalResourceReleasable) this.workerPool).releaseExternalResources();
        }
    }
}
