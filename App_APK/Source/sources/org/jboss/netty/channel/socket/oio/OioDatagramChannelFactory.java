package org.jboss.netty.channel.socket.oio;

import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import org.jboss.netty.channel.ChannelPipeline;
import org.jboss.netty.channel.socket.DatagramChannel;
import org.jboss.netty.channel.socket.DatagramChannelFactory;
import org.jboss.netty.util.internal.ExecutorUtil;

public class OioDatagramChannelFactory implements DatagramChannelFactory {
    final OioDatagramPipelineSink sink;
    private final Executor workerExecutor;

    public OioDatagramChannelFactory() {
        this(Executors.newCachedThreadPool());
    }

    public OioDatagramChannelFactory(Executor workerExecutor2) {
        if (workerExecutor2 != null) {
            this.workerExecutor = workerExecutor2;
            this.sink = new OioDatagramPipelineSink(workerExecutor2);
            return;
        }
        throw new NullPointerException("workerExecutor");
    }

    public DatagramChannel newChannel(ChannelPipeline pipeline) {
        return new OioDatagramChannel(this, pipeline, this.sink);
    }

    public void releaseExternalResources() {
        ExecutorUtil.terminate(this.workerExecutor);
    }
}
