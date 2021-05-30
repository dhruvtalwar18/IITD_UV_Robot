package org.jboss.netty.channel.socket.nio;

import java.io.IOException;
import java.net.ConnectException;
import java.net.SocketAddress;
import java.nio.channels.ClosedChannelException;
import java.nio.channels.SelectionKey;
import java.nio.channels.Selector;
import java.nio.channels.SocketChannel;
import java.util.Iterator;
import java.util.Queue;
import java.util.Set;
import java.util.concurrent.Executor;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import org.jboss.netty.channel.Channel;
import org.jboss.netty.channel.ChannelEvent;
import org.jboss.netty.channel.ChannelFuture;
import org.jboss.netty.channel.ChannelFutureListener;
import org.jboss.netty.channel.ChannelPipeline;
import org.jboss.netty.channel.ChannelState;
import org.jboss.netty.channel.ChannelStateEvent;
import org.jboss.netty.channel.Channels;
import org.jboss.netty.channel.MessageEvent;
import org.jboss.netty.logging.InternalLogger;
import org.jboss.netty.logging.InternalLoggerFactory;
import org.jboss.netty.util.internal.QueueFactory;

class NioClientSocketPipelineSink extends AbstractNioChannelSink {
    static final /* synthetic */ boolean $assertionsDisabled = false;
    static final InternalLogger logger = InternalLoggerFactory.getInstance((Class<?>) NioClientSocketPipelineSink.class);
    private static final AtomicInteger nextId = new AtomicInteger();
    final Executor bossExecutor;
    private final AtomicInteger bossIndex = new AtomicInteger();
    private final Boss[] bosses;
    final int id = nextId.incrementAndGet();
    private final WorkerPool<NioWorker> workerPool;

    NioClientSocketPipelineSink(Executor bossExecutor2, int bossCount, WorkerPool<NioWorker> workerPool2) {
        this.bossExecutor = bossExecutor2;
        this.bosses = new Boss[bossCount];
        for (int i = 0; i < this.bosses.length; i++) {
            this.bosses[i] = new Boss(i);
        }
        this.workerPool = workerPool2;
    }

    public void eventSunk(ChannelPipeline pipeline, ChannelEvent e) throws Exception {
        if (e instanceof ChannelStateEvent) {
            ChannelStateEvent event = (ChannelStateEvent) e;
            NioClientSocketChannel channel = (NioClientSocketChannel) event.getChannel();
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
                    if (value != null) {
                        bind(channel, future, (SocketAddress) value);
                        return;
                    } else {
                        channel.worker.close(channel, future);
                        return;
                    }
                case CONNECTED:
                    if (value != null) {
                        connect(channel, future, (SocketAddress) value);
                        return;
                    } else {
                        channel.worker.close(channel, future);
                        return;
                    }
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

    private static void bind(NioClientSocketChannel channel, ChannelFuture future, SocketAddress localAddress) {
        try {
            ((SocketChannel) channel.channel).socket().bind(localAddress);
            channel.boundManually = true;
            channel.setBound();
            future.setSuccess();
            Channels.fireChannelBound((Channel) channel, (SocketAddress) channel.getLocalAddress());
        } catch (Throwable t) {
            future.setFailure(t);
            Channels.fireExceptionCaught((Channel) channel, t);
        }
    }

    private void connect(NioClientSocketChannel channel, final ChannelFuture cf, SocketAddress remoteAddress) {
        try {
            if (((SocketChannel) channel.channel).connect(remoteAddress)) {
                channel.worker.register(channel, cf);
                return;
            }
            channel.getCloseFuture().addListener(new ChannelFutureListener() {
                public void operationComplete(ChannelFuture f) throws Exception {
                    if (!cf.isDone()) {
                        cf.setFailure(new ClosedChannelException());
                    }
                }
            });
            cf.addListener(ChannelFutureListener.CLOSE_ON_FAILURE);
            channel.connectFuture = cf;
            nextBoss().register(channel);
        } catch (Throwable t) {
            cf.setFailure(t);
            Channels.fireExceptionCaught((Channel) channel, t);
            channel.worker.close(channel, Channels.succeededFuture(channel));
        }
    }

    /* access modifiers changed from: package-private */
    public Boss nextBoss() {
        return this.bosses[Math.abs(this.bossIndex.getAndIncrement() % this.bosses.length)];
    }

    /* access modifiers changed from: package-private */
    public NioWorker nextWorker() {
        return this.workerPool.nextWorker();
    }

    private final class Boss implements Runnable {
        static final /* synthetic */ boolean $assertionsDisabled = false;
        private final Queue<Runnable> registerTaskQueue = QueueFactory.createQueue(Runnable.class);
        volatile Selector selector;
        private final Object startStopLock = new Object();
        private boolean started;
        private final int subId;
        private final AtomicBoolean wakenUp = new AtomicBoolean();

        static {
            Class<NioClientSocketPipelineSink> cls = NioClientSocketPipelineSink.class;
        }

        Boss(int subId2) {
            this.subId = subId2;
        }

        /* access modifiers changed from: package-private */
        /* JADX WARNING: Code restructure failed: missing block: B:35:0x0077, code lost:
            r5 = null;
            r6.selector = null;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:53:0x00a3, code lost:
            if (r10.wakenUp.compareAndSet(false, true) == false) goto L_?;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:54:0x00a5, code lost:
            r2.wakeup();
         */
        /* JADX WARNING: Code restructure failed: missing block: B:62:?, code lost:
            return;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:63:?, code lost:
            return;
         */
        /* Code decompiled incorrectly, please refer to instructions dump. */
        public void register(org.jboss.netty.channel.socket.nio.NioClientSocketChannel r11) {
            /*
                r10 = this;
                org.jboss.netty.channel.socket.nio.NioClientSocketPipelineSink$RegisterTask r0 = new org.jboss.netty.channel.socket.nio.NioClientSocketPipelineSink$RegisterTask
                r0.<init>(r10, r11)
                java.lang.Object r1 = r10.startStopLock
                monitor-enter(r1)
                r2 = 0
                boolean r3 = r10.started     // Catch:{ all -> 0x00a9 }
                r4 = 0
                if (r3 != 0) goto L_0x008e
                java.nio.channels.Selector r3 = java.nio.channels.Selector.open()     // Catch:{ Throwable -> 0x0083 }
                r5 = r3
                r10.selector = r3     // Catch:{ Throwable -> 0x0081 }
                r3 = r4
                org.jboss.netty.channel.socket.nio.NioClientSocketPipelineSink r6 = org.jboss.netty.channel.socket.nio.NioClientSocketPipelineSink.this     // Catch:{ all -> 0x005f }
                java.util.concurrent.Executor r6 = r6.bossExecutor     // Catch:{ all -> 0x005f }
                org.jboss.netty.util.ThreadRenamingRunnable r7 = new org.jboss.netty.util.ThreadRenamingRunnable     // Catch:{ all -> 0x005f }
                java.lang.StringBuilder r8 = new java.lang.StringBuilder     // Catch:{ all -> 0x005f }
                r8.<init>()     // Catch:{ all -> 0x005f }
                java.lang.String r9 = "New I/O client boss #"
                r8.append(r9)     // Catch:{ all -> 0x005f }
                org.jboss.netty.channel.socket.nio.NioClientSocketPipelineSink r9 = org.jboss.netty.channel.socket.nio.NioClientSocketPipelineSink.this     // Catch:{ all -> 0x005f }
                int r9 = r9.id     // Catch:{ all -> 0x005f }
                r8.append(r9)     // Catch:{ all -> 0x005f }
                r9 = 45
                r8.append(r9)     // Catch:{ all -> 0x005f }
                int r9 = r10.subId     // Catch:{ all -> 0x005f }
                r8.append(r9)     // Catch:{ all -> 0x005f }
                java.lang.String r8 = r8.toString()     // Catch:{ all -> 0x005f }
                r7.<init>(r10, r8)     // Catch:{ all -> 0x005f }
                org.jboss.netty.util.internal.DeadLockProofWorker.start(r6, r7)     // Catch:{ all -> 0x005f }
                r3 = 1
                if (r3 != 0) goto L_0x005d
                r5.close()     // Catch:{ Throwable -> 0x004a }
                goto L_0x005a
            L_0x004a:
                r6 = move-exception
                org.jboss.netty.logging.InternalLogger r7 = org.jboss.netty.channel.socket.nio.NioClientSocketPipelineSink.logger     // Catch:{ all -> 0x007e }
                boolean r7 = r7.isWarnEnabled()     // Catch:{ all -> 0x007e }
                if (r7 == 0) goto L_0x005a
                org.jboss.netty.logging.InternalLogger r7 = org.jboss.netty.channel.socket.nio.NioClientSocketPipelineSink.logger     // Catch:{ all -> 0x007e }
                java.lang.String r8 = "Failed to close a selector."
                r7.warn(r8, r6)     // Catch:{ all -> 0x007e }
            L_0x005a:
                r5 = r2
                r10.selector = r2     // Catch:{ all -> 0x007e }
            L_0x005d:
                goto L_0x0090
            L_0x005f:
                r4 = move-exception
                r6 = r10
                if (r3 != 0) goto L_0x007a
                r5.close()     // Catch:{ Throwable -> 0x0067 }
                goto L_0x0077
            L_0x0067:
                r7 = move-exception
                org.jboss.netty.logging.InternalLogger r8 = org.jboss.netty.channel.socket.nio.NioClientSocketPipelineSink.logger     // Catch:{ all -> 0x00af }
                boolean r8 = r8.isWarnEnabled()     // Catch:{ all -> 0x00af }
                if (r8 == 0) goto L_0x0077
                org.jboss.netty.logging.InternalLogger r8 = org.jboss.netty.channel.socket.nio.NioClientSocketPipelineSink.logger     // Catch:{ all -> 0x00af }
                java.lang.String r9 = "Failed to close a selector."
                r8.warn(r9, r7)     // Catch:{ all -> 0x00af }
            L_0x0077:
                r5 = r2
                r6.selector = r2     // Catch:{ all -> 0x00af }
            L_0x007a:
                r2 = r5
                throw r4     // Catch:{ all -> 0x007c }
            L_0x007c:
                r3 = move-exception
                goto L_0x00ab
            L_0x007e:
                r2 = move-exception
                r6 = r10
                goto L_0x00ad
            L_0x0081:
                r2 = move-exception
                goto L_0x0086
            L_0x0083:
                r3 = move-exception
                r5 = r2
                r2 = r3
            L_0x0086:
                org.jboss.netty.channel.ChannelException r3 = new org.jboss.netty.channel.ChannelException     // Catch:{ all -> 0x007e }
                java.lang.String r4 = "Failed to create a selector."
                r3.<init>(r4, r2)     // Catch:{ all -> 0x007e }
                throw r3     // Catch:{ all -> 0x007e }
            L_0x008e:
                java.nio.channels.Selector r5 = r10.selector     // Catch:{ all -> 0x00a9 }
            L_0x0090:
                r2 = r5
                r3 = 1
                r10.started = r3     // Catch:{ all -> 0x00a9 }
                java.util.Queue<java.lang.Runnable> r5 = r10.registerTaskQueue     // Catch:{ all -> 0x00a9 }
                boolean r5 = r5.offer(r0)     // Catch:{ all -> 0x00a9 }
                monitor-exit(r1)     // Catch:{ all -> 0x00a9 }
                java.util.concurrent.atomic.AtomicBoolean r1 = r10.wakenUp
                boolean r1 = r1.compareAndSet(r4, r3)
                if (r1 == 0) goto L_0x00a8
                r2.wakeup()
            L_0x00a8:
                return
            L_0x00a9:
                r3 = move-exception
                r6 = r10
            L_0x00ab:
                r5 = r2
                r2 = r3
            L_0x00ad:
                monitor-exit(r1)     // Catch:{ all -> 0x00af }
                throw r2
            L_0x00af:
                r2 = move-exception
                goto L_0x00ad
            */
            throw new UnsupportedOperationException("Method not decompiled: org.jboss.netty.channel.socket.nio.NioClientSocketPipelineSink.Boss.register(org.jboss.netty.channel.socket.nio.NioClientSocketChannel):void");
        }

        public void run() {
            Selector selector2 = this.selector;
            long lastConnectTimeoutCheckTimeNanos = System.nanoTime();
            boolean shutdown = false;
            while (true) {
                this.wakenUp.set(false);
                try {
                    int selectedKeyCount = selector2.select(10);
                    if (this.wakenUp.get()) {
                        selector2.wakeup();
                    }
                    processRegisterTaskQueue();
                    if (selectedKeyCount > 0) {
                        processSelectedKeys(selector2.selectedKeys());
                    }
                    long currentTimeNanos = System.nanoTime();
                    if (currentTimeNanos - lastConnectTimeoutCheckTimeNanos >= 10000000) {
                        lastConnectTimeoutCheckTimeNanos = currentTimeNanos;
                        processConnectTimeout(selector2.keys(), currentTimeNanos);
                    }
                    if (selector2.keys().isEmpty()) {
                        if (!shutdown) {
                            if (!(NioClientSocketPipelineSink.this.bossExecutor instanceof ExecutorService) || !((ExecutorService) NioClientSocketPipelineSink.this.bossExecutor).isShutdown()) {
                                shutdown = true;
                            }
                        }
                        synchronized (this.startStopLock) {
                            if (!this.registerTaskQueue.isEmpty() || !selector2.keys().isEmpty()) {
                                shutdown = false;
                            } else {
                                this.started = false;
                                try {
                                    selector2.close();
                                } catch (IOException e) {
                                    try {
                                        if (NioClientSocketPipelineSink.logger.isWarnEnabled()) {
                                            NioClientSocketPipelineSink.logger.warn("Failed to close a selector.", e);
                                        }
                                    } catch (Throwable th) {
                                        this.selector = null;
                                        throw th;
                                    }
                                }
                                this.selector = null;
                                return;
                            }
                        }
                    } else {
                        shutdown = false;
                    }
                } catch (Throwable t) {
                    if (NioClientSocketPipelineSink.logger.isWarnEnabled()) {
                        NioClientSocketPipelineSink.logger.warn("Unexpected exception in the selector loop.", t);
                    }
                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException e2) {
                    }
                }
            }
        }

        private void processRegisterTaskQueue() {
            while (true) {
                Runnable task = this.registerTaskQueue.poll();
                if (task != null) {
                    task.run();
                } else {
                    return;
                }
            }
        }

        private void processSelectedKeys(Set<SelectionKey> selectedKeys) {
            Iterator<SelectionKey> i = selectedKeys.iterator();
            while (i.hasNext()) {
                SelectionKey k = i.next();
                i.remove();
                if (!k.isValid()) {
                    close(k);
                } else if (k.isConnectable()) {
                    connect(k);
                }
            }
        }

        private void processConnectTimeout(Set<SelectionKey> keys, long currentTimeNanos) {
            ConnectException cause = null;
            for (SelectionKey k : keys) {
                if (k.isValid()) {
                    NioClientSocketChannel ch = (NioClientSocketChannel) k.attachment();
                    if (ch.connectDeadlineNanos > 0 && currentTimeNanos >= ch.connectDeadlineNanos) {
                        if (cause == null) {
                            cause = new ConnectException("connection timed out");
                        }
                        ch.connectFuture.setFailure(cause);
                        Channels.fireExceptionCaught((Channel) ch, (Throwable) cause);
                        ch.worker.close(ch, Channels.succeededFuture(ch));
                    }
                }
            }
        }

        private void connect(SelectionKey k) {
            NioClientSocketChannel ch = (NioClientSocketChannel) k.attachment();
            try {
                if (((SocketChannel) ch.channel).finishConnect()) {
                    k.cancel();
                    ch.worker.register(ch, ch.connectFuture);
                }
            } catch (Throwable t) {
                ch.connectFuture.setFailure(t);
                Channels.fireExceptionCaught((Channel) ch, t);
                k.cancel();
                ch.worker.close(ch, Channels.succeededFuture(ch));
            }
        }

        private void close(SelectionKey k) {
            NioClientSocketChannel ch = (NioClientSocketChannel) k.attachment();
            ch.worker.close(ch, Channels.succeededFuture(ch));
        }
    }

    private static final class RegisterTask implements Runnable {
        private final Boss boss;
        private final NioClientSocketChannel channel;

        RegisterTask(Boss boss2, NioClientSocketChannel channel2) {
            this.boss = boss2;
            this.channel = channel2;
        }

        public void run() {
            try {
                ((SocketChannel) this.channel.channel).register(this.boss.selector, 8, this.channel);
            } catch (ClosedChannelException e) {
                this.channel.worker.close(this.channel, Channels.succeededFuture(this.channel));
            }
            int connectTimeout = this.channel.getConfig().getConnectTimeoutMillis();
            if (connectTimeout > 0) {
                this.channel.connectDeadlineNanos = System.nanoTime() + (((long) connectTimeout) * 1000000);
            }
        }
    }
}
