package org.jboss.netty.channel.socket.oio;

import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import org.jboss.netty.channel.ChannelPipeline;
import org.jboss.netty.channel.ChannelSink;
import org.jboss.netty.channel.socket.ServerSocketChannel;
import org.jboss.netty.channel.socket.ServerSocketChannelFactory;
import org.jboss.netty.util.internal.ExecutorUtil;

public class OioServerSocketChannelFactory implements ServerSocketChannelFactory {
    final Executor bossExecutor;
    private final ChannelSink sink;
    private final Executor workerExecutor;

    public OioServerSocketChannelFactory() {
        this(Executors.newCachedThreadPool(), Executors.newCachedThreadPool());
    }

    public OioServerSocketChannelFactory(Executor bossExecutor2, Executor workerExecutor2) {
        if (bossExecutor2 == null) {
            throw new NullPointerException("bossExecutor");
        } else if (workerExecutor2 != null) {
            this.bossExecutor = bossExecutor2;
            this.workerExecutor = workerExecutor2;
            this.sink = new OioServerSocketPipelineSink(workerExecutor2);
        } else {
            throw new NullPointerException("workerExecutor");
        }
    }

    public ServerSocketChannel newChannel(ChannelPipeline pipeline) {
        return new OioServerSocketChannel(this, pipeline, this.sink);
    }

    public void releaseExternalResources() {
        ExecutorUtil.terminate(this.bossExecutor, this.workerExecutor);
    }
}
