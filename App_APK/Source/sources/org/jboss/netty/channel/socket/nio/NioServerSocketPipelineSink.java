package org.jboss.netty.channel.socket.nio;

import java.io.IOException;
import java.net.SocketAddress;
import java.net.SocketTimeoutException;
import java.nio.channels.CancelledKeyException;
import java.nio.channels.ClosedChannelException;
import java.nio.channels.ClosedSelectorException;
import java.nio.channels.Selector;
import java.nio.channels.SocketChannel;
import java.util.concurrent.Executor;
import java.util.concurrent.atomic.AtomicInteger;
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

class NioServerSocketPipelineSink extends AbstractNioChannelSink {
    static final /* synthetic */ boolean $assertionsDisabled = false;
    static final InternalLogger logger = InternalLoggerFactory.getInstance((Class<?>) NioServerSocketPipelineSink.class);
    private static final AtomicInteger nextId = new AtomicInteger();
    final int id = nextId.incrementAndGet();
    private final WorkerPool<NioWorker> workerPool;

    NioServerSocketPipelineSink(WorkerPool<NioWorker> workerPool2) {
        this.workerPool = workerPool2;
    }

    public void eventSunk(ChannelPipeline pipeline, ChannelEvent e) throws Exception {
        Channel channel = e.getChannel();
        if (channel instanceof NioServerSocketChannel) {
            handleServerSocket(e);
        } else if (channel instanceof NioSocketChannel) {
            handleAcceptedSocket(e);
        }
    }

    private void handleServerSocket(ChannelEvent e) {
        if (e instanceof ChannelStateEvent) {
            ChannelStateEvent event = (ChannelStateEvent) e;
            NioServerSocketChannel channel = (NioServerSocketChannel) event.getChannel();
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
            NioSocketChannel channel = (NioSocketChannel) event.getChannel();
            ChannelFuture future = event.getFuture();
            ChannelState state = event.getState();
            Object value = event.getValue();
            switch (state) {
                case OPEN:
                    if (Boolean.FALSE.equals(value)) {
                        channel.worker.close(channel, future);
                        return;
                    }
                    return;
                case BOUND:
                case CONNECTED:
                    if (value == null) {
                        channel.worker.close(channel, future);
                        return;
                    }
                    return;
                case INTEREST_OPS:
                    channel.worker.setInterestOps(channel, future, ((Integer) value).intValue());
                    return;
                default:
                    return;
            }
        } else if (e instanceof MessageEvent) {
            MessageEvent event2 = (MessageEvent) e;
            NioSocketChannel channel2 = (NioSocketChannel) event2.getChannel();
            boolean offer = channel2.writeBufferQueue.offer(event2);
            channel2.worker.writeFromUserCode(channel2);
        }
    }

    private void bind(NioServerSocketChannel channel, ChannelFuture future, SocketAddress localAddress) {
        boolean bound = false;
        try {
            channel.socket.socket().bind(localAddress, channel.getConfig().getBacklog());
            bound = true;
            future.setSuccess();
            Channels.fireChannelBound((Channel) channel, (SocketAddress) channel.getLocalAddress());
            Executor bossExecutor = ((NioServerSocketChannelFactory) channel.getFactory()).bossExecutor;
            Boss boss = new Boss(channel);
            DeadLockProofWorker.start(bossExecutor, new ThreadRenamingRunnable(boss, "New I/O server boss #" + this.id + " (" + channel + ')'));
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

    private static void close(NioServerSocketChannel channel, ChannelFuture future) {
        boolean bound = channel.isBound();
        try {
            if (channel.socket.isOpen()) {
                channel.socket.close();
                Selector selector = channel.selector;
                if (selector != null) {
                    selector.wakeup();
                }
            }
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

    /* access modifiers changed from: package-private */
    public NioWorker nextWorker() {
        return this.workerPool.nextWorker();
    }

    private final class Boss implements Runnable {
        private final NioServerSocketChannel channel;
        private final Selector selector = Selector.open();

        Boss(NioServerSocketChannel channel2) throws IOException {
            this.channel = channel2;
            try {
                channel2.socket.register(this.selector, 16);
                if (1 == 0) {
                    closeSelector();
                }
                channel2.selector = this.selector;
            } catch (Throwable th) {
                if (0 == 0) {
                    closeSelector();
                }
                throw th;
            }
        }

        public void run() {
            Thread currentThread = Thread.currentThread();
            this.channel.shutdownLock.lock();
            while (true) {
                try {
                    if (this.selector.select(1000) > 0) {
                        this.selector.selectedKeys().clear();
                    }
                    while (true) {
                        SocketChannel acceptedSocket = this.channel.socket.accept();
                        if (acceptedSocket == null) {
                            break;
                        }
                        registerAcceptedChannel(acceptedSocket, currentThread);
                    }
                } catch (SocketTimeoutException | CancelledKeyException | ClosedSelectorException e) {
                } catch (ClosedChannelException e2) {
                    this.channel.shutdownLock.unlock();
                    closeSelector();
                    return;
                } catch (Throwable th) {
                    this.channel.shutdownLock.unlock();
                    closeSelector();
                    throw th;
                }
            }
        }

        private void registerAcceptedChannel(SocketChannel acceptedSocket, Thread currentThread) {
            try {
                ChannelPipeline pipeline = this.channel.getConfig().getPipelineFactory().getPipeline();
                NioWorker worker = NioServerSocketPipelineSink.this.nextWorker();
                worker.register(new NioAcceptedSocketChannel(this.channel.getFactory(), pipeline, this.channel, NioServerSocketPipelineSink.this, acceptedSocket, worker, currentThread), (ChannelFuture) null);
            } catch (Exception e) {
                if (NioServerSocketPipelineSink.logger.isWarnEnabled()) {
                    NioServerSocketPipelineSink.logger.warn("Failed to initialize an accepted socket.", e);
                }
                try {
                    acceptedSocket.close();
                } catch (IOException e2) {
                    if (NioServerSocketPipelineSink.logger.isWarnEnabled()) {
                        NioServerSocketPipelineSink.logger.warn("Failed to close a partially accepted socket.", e2);
                    }
                }
            }
        }

        private void closeSelector() {
            this.channel.selector = null;
            try {
                this.selector.close();
            } catch (Exception e) {
                if (NioServerSocketPipelineSink.logger.isWarnEnabled()) {
                    NioServerSocketPipelineSink.logger.warn("Failed to close a selector.", e);
                }
            }
        }
    }
}
