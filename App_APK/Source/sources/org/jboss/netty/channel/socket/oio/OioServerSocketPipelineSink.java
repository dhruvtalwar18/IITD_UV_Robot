package org.jboss.netty.channel.socket.oio;

import java.io.IOException;
import java.net.Socket;
import java.net.SocketAddress;
import java.net.SocketTimeoutException;
import java.util.concurrent.Executor;
import org.jboss.netty.channel.Channel;
import org.jboss.netty.channel.ChannelEvent;
import org.jboss.netty.channel.ChannelFuture;
import org.jboss.netty.channel.ChannelPipeline;
import org.jboss.netty.channel.ChannelState;
import org.jboss.netty.channel.ChannelStateEvent;
import org.jboss.netty.channel.Channels;
import org.jboss.netty.channel.MessageEvent;
import org.jboss.netty.logging.InternalLogger;
import org.jboss.netty.logging.InternalLoggerFactory;
import org.jboss.netty.util.ThreadRenamingRunnable;
import org.jboss.netty.util.internal.DeadLockProofWorker;

class OioServerSocketPipelineSink extends AbstractOioChannelSink {
    static final InternalLogger logger = InternalLoggerFactory.getInstance((Class<?>) OioServerSocketPipelineSink.class);
    final Executor workerExecutor;

    OioServerSocketPipelineSink(Executor workerExecutor2) {
        this.workerExecutor = workerExecutor2;
    }

    public void eventSunk(ChannelPipeline pipeline, ChannelEvent e) throws Exception {
        Channel channel = e.getChannel();
        if (channel instanceof OioServerSocketChannel) {
            handleServerSocket(e);
        } else if (channel instanceof OioAcceptedSocketChannel) {
            handleAcceptedSocket(e);
        }
    }

    private void handleServerSocket(ChannelEvent e) {
        if (e instanceof ChannelStateEvent) {
            ChannelStateEvent event = (ChannelStateEvent) e;
            OioServerSocketChannel channel = (OioServerSocketChannel) event.getChannel();
            ChannelFuture future = event.getFuture();
            ChannelState state = event.getState();
            Object value = event.getValue();
            switch (state) {
                case OPEN:
                    if (Boolean.FALSE.equals(value)) {
                        close(channel, future);
                        return;
                    }
                    return;
                case BOUND:
                    if (value != null) {
                        bind(channel, future, (SocketAddress) value);
                        return;
                    } else {
                        close(channel, future);
                        return;
                    }
                default:
                    return;
            }
        }
    }

    private static void handleAcceptedSocket(ChannelEvent e) {
        if (e instanceof ChannelStateEvent) {
            ChannelStateEvent event = (ChannelStateEvent) e;
            OioAcceptedSocketChannel channel = (OioAcceptedSocketChannel) event.getChannel();
            ChannelFuture future = event.getFuture();
            ChannelState state = event.getState();
            Object value = event.getValue();
            switch (state) {
                case OPEN:
                    if (Boolean.FALSE.equals(value)) {
                        AbstractOioWorker.close(channel, future);
                        return;
                    }
                    return;
                case BOUND:
                case CONNECTED:
                    if (value == null) {
                        AbstractOioWorker.close(channel, future);
                        return;
                    }
                    return;
                case INTEREST_OPS:
                    AbstractOioWorker.setInterestOps(channel, future, ((Integer) value).intValue());
                    return;
                default:
                    return;
            }
        } else if (e instanceof MessageEvent) {
            MessageEvent event2 = (MessageEvent) e;
            OioWorker.write((OioSocketChannel) event2.getChannel(), event2.getFuture(), event2.getMessage());
        }
    }

    private void bind(OioServerSocketChannel channel, ChannelFuture future, SocketAddress localAddress) {
        boolean bound = false;
        try {
            channel.socket.bind(localAddress, channel.getConfig().getBacklog());
            bound = true;
            future.setSuccess();
            Channels.fireChannelBound((Channel) channel, channel.getLocalAddress());
            Executor bossExecutor = ((OioServerSocketChannelFactory) channel.getFactory()).bossExecutor;
            Boss boss = new Boss(channel);
            DeadLockProofWorker.start(bossExecutor, new ThreadRenamingRunnable(boss, "Old I/O server boss (" + channel + ')'));
            if (1 == 0 && 1 != 0) {
                close(channel, future);
            }
        } catch (Throwable th) {
            if (0 == 0 && bound) {
                close(channel, future);
            }
            throw th;
        }
    }

    private static void close(OioServerSocketChannel channel, ChannelFuture future) {
        boolean bound = channel.isBound();
        try {
            channel.socket.close();
            channel.shutdownLock.lock();
            if (channel.setClosed()) {
                future.setSuccess();
                if (bound) {
                    Channels.fireChannelUnbound((Channel) channel);
                }
                Channels.fireChannelClosed((Channel) channel);
            } else {
                future.setSuccess();
            }
            channel.shutdownLock.unlock();
        } catch (Throwable t) {
            future.setFailure(t);
            Channels.fireExceptionCaught((Channel) channel, t);
        }
    }

    private final class Boss implements Runnable {
        private final OioServerSocketChannel channel;

        Boss(OioServerSocketChannel channel2) {
            this.channel = channel2;
        }

        public void run() {
            this.channel.shutdownLock.lock();
            while (true) {
                try {
                    if (!this.channel.isBound()) {
                        break;
                    }
                    Socket acceptedSocket = this.channel.socket.accept();
                    try {
                        OioAcceptedSocketChannel oioAcceptedSocketChannel = new OioAcceptedSocketChannel(this.channel, this.channel.getFactory(), this.channel.getConfig().getPipelineFactory().getPipeline(), OioServerSocketPipelineSink.this, acceptedSocket);
                        Executor executor = OioServerSocketPipelineSink.this.workerExecutor;
                        OioWorker oioWorker = new OioWorker(oioAcceptedSocketChannel);
                        DeadLockProofWorker.start(executor, new ThreadRenamingRunnable(oioWorker, "Old I/O server worker (parentId: " + this.channel.getId() + ", " + this.channel + ')'));
                    } catch (Exception e) {
                        if (OioServerSocketPipelineSink.logger.isWarnEnabled()) {
                            OioServerSocketPipelineSink.logger.warn("Failed to initialize an accepted socket.", e);
                        }
                        try {
                            acceptedSocket.close();
                        } catch (IOException e2) {
                            if (OioServerSocketPipelineSink.logger.isWarnEnabled()) {
                                OioServerSocketPipelineSink.logger.warn("Failed to close a partially accepted socket.", e2);
                            }
                        }
                    }
                } catch (SocketTimeoutException e3) {
                } catch (Throwable th) {
                    this.channel.shutdownLock.unlock();
                    throw th;
                }
            }
            this.channel.shutdownLock.unlock();
        }
    }
}
