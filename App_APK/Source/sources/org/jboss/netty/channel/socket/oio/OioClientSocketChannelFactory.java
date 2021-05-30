package org.jboss.netty.channel.socket.oio;

import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import org.jboss.netty.channel.ChannelPipeline;
import org.jboss.netty.channel.socket.ClientSocketChannelFactory;
import org.jboss.netty.channel.socket.SocketChannel;
import org.jboss.netty.util.internal.ExecutorUtil;

public class OioClientSocketChannelFactory implements ClientSocketChannelFactory {
    final OioClientSocketPipelineSink sink;
    private final Executor workerExecutor;

    public OioClientSocketChannelFactory() {
        this(Executors.newCachedThreadPool());
    }

    public OioClientSocketChannelFactory(Executor workerExecutor2) {
        if (workerExecutor2 != null) {
            this.workerExecutor = workerExecutor2;
            this.sink = new OioClientSocketPipelineSink(workerExecutor2);
            return;
        }
        throw new NullPointerException("workerExecutor");
    }

    public SocketChannel newChannel(ChannelPipeline pipeline) {
        return new OioClientSocketChannel(this, pipeline, this.sink);
    }

    public void releaseExternalResources() {
        ExecutorUtil.terminate(this.workerExecutor);
    }
}
