package org.jboss.netty.channel.socket.nio;

import java.io.IOException;
import java.nio.channels.CancelledKeyException;
import java.nio.channels.ClosedChannelException;
import java.nio.channels.NotYetConnectedException;
import java.nio.channels.SelectionKey;
import java.nio.channels.Selector;
import java.util.Iterator;
import java.util.Queue;
import java.util.Set;
import java.util.concurrent.Executor;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import org.jboss.netty.channel.Channel;
import org.jboss.netty.channel.ChannelFuture;
import org.jboss.netty.channel.Channels;
import org.jboss.netty.channel.MessageEvent;
import org.jboss.netty.channel.socket.Worker;
import org.jboss.netty.logging.InternalLogger;
import org.jboss.netty.logging.InternalLoggerFactory;
import org.jboss.netty.util.ThreadRenamingRunnable;
import org.jboss.netty.util.internal.DeadLockProofWorker;
import org.jboss.netty.util.internal.QueueFactory;

abstract class AbstractNioWorker implements Worker {
    static final /* synthetic */ boolean $assertionsDisabled = false;
    static final int CLEANUP_INTERVAL = 256;
    private static final int CONSTRAINT_LEVEL = NioProviderMetadata.CONSTRAINT_LEVEL;
    private static final InternalLogger logger = InternalLoggerFactory.getInstance((Class<?>) AbstractNioWorker.class);
    private static final AtomicInteger nextId = new AtomicInteger();
    private final boolean allowShutdownOnIdle;
    private volatile int cancelledKeys;
    private final Queue<Runnable> eventQueue;
    private final Executor executor;
    final int id;
    private final Queue<Runnable> registerTaskQueue;
    volatile Selector selector;
    private final ReadWriteLock selectorGuard;
    protected final SocketSendBufferPool sendBufferPool;
    private final Object startStopLock;
    private boolean started;
    protected volatile Thread thread;
    protected final AtomicBoolean wakenUp;
    protected final Queue<Runnable> writeTaskQueue;

    /* access modifiers changed from: protected */
    public abstract Runnable createRegisterTask(AbstractNioChannel<?> abstractNioChannel, ChannelFuture channelFuture);

    /* access modifiers changed from: protected */
    public abstract boolean read(SelectionKey selectionKey);

    /* access modifiers changed from: protected */
    public abstract boolean scheduleWriteIfNecessary(AbstractNioChannel<?> abstractNioChannel);

    AbstractNioWorker(Executor executor2) {
        this(executor2, true);
    }

    public AbstractNioWorker(Executor executor2, boolean allowShutdownOnIdle2) {
        this.id = nextId.incrementAndGet();
        this.wakenUp = new AtomicBoolean();
        this.selectorGuard = new ReentrantReadWriteLock();
        this.startStopLock = new Object();
        this.registerTaskQueue = QueueFactory.createQueue(Runnable.class);
        this.writeTaskQueue = QueueFactory.createQueue(Runnable.class);
        this.eventQueue = QueueFactory.createQueue(Runnable.class);
        this.sendBufferPool = new SocketSendBufferPool();
        this.executor = executor2;
        this.allowShutdownOnIdle = allowShutdownOnIdle2;
    }

    /* access modifiers changed from: package-private */
    public void register(AbstractNioChannel<?> channel, ChannelFuture future) {
        Runnable registerTask = createRegisterTask(channel, future);
        Selector selector2 = start();
        boolean offer = this.registerTaskQueue.offer(registerTask);
        if (this.wakenUp.compareAndSet(false, true)) {
            selector2.wakeup();
        }
    }

    private Selector start() {
        synchronized (this.startStopLock) {
            if (!this.started) {
                try {
                    this.selector = Selector.open();
                    Executor executor2 = this.executor;
                    DeadLockProofWorker.start(executor2, new ThreadRenamingRunnable(this, "New I/O  worker #" + this.id));
                    if (1 == 0) {
                        this.selector.close();
                        this.selector = null;
                    }
                } catch (Throwable t) {
                    logger.warn("Failed to close a selector.", t);
                }
            }
            this.started = true;
        }
        return this.selector;
        this.selector = null;
        throw th;
    }

    public void run() {
        this.thread = Thread.currentThread();
        boolean shutdown = false;
        Selector selector2 = this.selector;
        while (true) {
            this.wakenUp.set(false);
            if (CONSTRAINT_LEVEL != 0) {
                this.selectorGuard.writeLock().lock();
                this.selectorGuard.writeLock().unlock();
            }
            try {
                SelectorUtil.select(selector2);
                if (this.wakenUp.get()) {
                    selector2.wakeup();
                }
                this.cancelledKeys = 0;
                processRegisterTaskQueue();
                processEventQueue();
                processWriteTaskQueue();
                processSelectedKeys(selector2.selectedKeys());
                if (selector2.keys().isEmpty()) {
                    if (!shutdown) {
                        if (!(this.executor instanceof ExecutorService) || !((ExecutorService) this.executor).isShutdown()) {
                            if (this.allowShutdownOnIdle) {
                                shutdown = true;
                            }
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
                                    logger.warn("Failed to close a selector.", e);
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
                logger.warn("Unexpected exception in the selector loop.", t);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e2) {
                }
            }
        }
    }

    public void executeInIoThread(Runnable task) {
        executeInIoThread(task, false);
    }

    public void executeInIoThread(Runnable task, boolean alwaysAsync) {
        Selector selector2;
        if (alwaysAsync || Thread.currentThread() != this.thread) {
            start();
            if (this.eventQueue.offer(task) && (selector2 = this.selector) != null) {
                selector2.wakeup();
                return;
            }
            return;
        }
        task.run();
    }

    private void processRegisterTaskQueue() throws IOException {
        while (true) {
            Runnable task = this.registerTaskQueue.poll();
            if (task != null) {
                task.run();
                cleanUpCancelledKeys();
            } else {
                return;
            }
        }
    }

    private void processWriteTaskQueue() throws IOException {
        while (true) {
            Runnable task = this.writeTaskQueue.poll();
            if (task != null) {
                task.run();
                cleanUpCancelledKeys();
            } else {
                return;
            }
        }
    }

    private void processEventQueue() throws IOException {
        while (true) {
            Runnable task = this.eventQueue.poll();
            if (task != null) {
                task.run();
                cleanUpCancelledKeys();
            } else {
                return;
            }
        }
    }

    private void processSelectedKeys(Set<SelectionKey> selectedKeys) throws IOException {
        Iterator<SelectionKey> i = selectedKeys.iterator();
        while (i.hasNext()) {
            SelectionKey k = i.next();
            i.remove();
            try {
                int readyOps = k.readyOps();
                if (((readyOps & 1) == 0 && readyOps != 0) || read(k)) {
                    if ((readyOps & 4) != 0) {
                        writeFromSelectorLoop(k);
                    }
                    if (cleanUpCancelledKeys()) {
                        return;
                    }
                }
            } catch (CancelledKeyException e) {
                close(k);
            }
        }
    }

    private boolean cleanUpCancelledKeys() throws IOException {
        if (this.cancelledKeys < 256) {
            return false;
        }
        this.cancelledKeys = 0;
        this.selector.selectNow();
        return true;
    }

    private void close(SelectionKey k) {
        AbstractNioChannel<?> ch = (AbstractNioChannel) k.attachment();
        close(ch, Channels.succeededFuture(ch));
    }

    /* access modifiers changed from: package-private */
    public void writeFromUserCode(AbstractNioChannel<?> channel) {
        if (!channel.isConnected()) {
            cleanUpWriteBuffer(channel);
        } else if (!scheduleWriteIfNecessary(channel) && !channel.writeSuspended && !channel.inWriteNowLoop) {
            write0(channel);
        }
    }

    /* access modifiers changed from: package-private */
    public void writeFromTaskLoop(AbstractNioChannel<?> ch) {
        if (!ch.writeSuspended) {
            write0(ch);
        }
    }

    /* access modifiers changed from: package-private */
    public void writeFromSelectorLoop(SelectionKey k) {
        AbstractNioChannel<?> ch = (AbstractNioChannel) k.attachment();
        ch.writeSuspended = false;
        write0(ch);
    }

    /* access modifiers changed from: protected */
    /* JADX WARNING: Code restructure failed: missing block: B:104:?, code lost:
        return;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:105:?, code lost:
        return;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:40:0x009d, code lost:
        r0 = th;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:41:0x009e, code lost:
        r7 = r25;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:43:0x00a2, code lost:
        r27 = r3;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:59:0x00d0, code lost:
        if (r6 == false) goto L_0x00d6;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:60:0x00d2, code lost:
        org.jboss.netty.channel.Channels.fireWriteComplete((org.jboss.netty.channel.Channel) r2, r7);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:61:0x00d6, code lost:
        org.jboss.netty.channel.Channels.fireWriteCompleteLater(r2, r7);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:63:0x00dc, code lost:
        r0 = th;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:64:0x00dd, code lost:
        r7 = r25;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:65:0x00e0, code lost:
        r0 = th;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:70:?, code lost:
        r14.release();
     */
    /* JADX WARNING: Code restructure failed: missing block: B:78:0x00f9, code lost:
        org.jboss.netty.channel.Channels.fireExceptionCaught((org.jboss.netty.channel.Channel) r2, r0);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:79:0x00fd, code lost:
        org.jboss.netty.channel.Channels.fireExceptionCaughtLater((org.jboss.netty.channel.Channel) r2, r0);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:82:0x0106, code lost:
        r3 = false;
        r29 = r0;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:84:?, code lost:
        close(r2, org.jboss.netty.channel.Channels.succeededFuture(r31));
     */
    /* JADX WARNING: Code restructure failed: missing block: B:88:0x0116, code lost:
        r27 = r3;
        r28 = r24;
     */
    /* JADX WARNING: Failed to process nested try/catch */
    /* JADX WARNING: Removed duplicated region for block: B:42:0x00a1 A[ExcHandler: AsynchronousCloseException (e java.nio.channels.AsynchronousCloseException), PHI: r4 
      PHI: (r4v9 'addOpWrite' boolean) = (r4v8 'addOpWrite' boolean), (r4v1 'addOpWrite' boolean), (r4v1 'addOpWrite' boolean) binds: [B:45:0x00a7, B:37:0x008b, B:38:?] A[DONT_GENERATE, DONT_INLINE], Splitter:B:37:0x008b] */
    /* JADX WARNING: Removed duplicated region for block: B:63:0x00dc A[ExcHandler: all (th java.lang.Throwable), Splitter:B:34:0x0085] */
    /* JADX WARNING: Removed duplicated region for block: B:69:0x00e6 A[SYNTHETIC, Splitter:B:69:0x00e6] */
    /* JADX WARNING: Removed duplicated region for block: B:78:0x00f9 A[Catch:{ all -> 0x0111, all -> 0x0126 }] */
    /* JADX WARNING: Removed duplicated region for block: B:79:0x00fd A[Catch:{ all -> 0x0111, all -> 0x0126 }] */
    /* JADX WARNING: Removed duplicated region for block: B:82:0x0106  */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public void write0(org.jboss.netty.channel.socket.nio.AbstractNioChannel<?> r31) {
        /*
            r30 = this;
            r1 = r30
            r2 = r31
            r3 = 1
            r4 = 0
            r5 = 0
            boolean r6 = isIoThread(r31)
            r7 = 0
            org.jboss.netty.channel.socket.nio.SocketSendBufferPool r9 = r1.sendBufferPool
            C r0 = r2.channel
            r10 = r0
            java.nio.channels.WritableByteChannel r10 = (java.nio.channels.WritableByteChannel) r10
            java.util.Queue<org.jboss.netty.channel.MessageEvent> r11 = r2.writeBufferQueue
            org.jboss.netty.channel.socket.nio.NioChannelConfig r0 = r31.getConfig()
            int r12 = r0.getWriteSpinCount()
            java.lang.Object r13 = r2.writeLock
            monitor-enter(r13)
            r14 = 1
            r2.inWriteNowLoop = r14     // Catch:{ all -> 0x0126 }
        L_0x0024:
            org.jboss.netty.channel.MessageEvent r0 = r2.currentWriteEvent     // Catch:{ all -> 0x0122 }
            r15 = 0
            if (r0 != 0) goto L_0x004a
            java.lang.Object r16 = r11.poll()     // Catch:{ all -> 0x0126 }
            r14 = r16
            org.jboss.netty.channel.MessageEvent r14 = (org.jboss.netty.channel.MessageEvent) r14     // Catch:{ all -> 0x0126 }
            r0 = r14
            r2.currentWriteEvent = r14     // Catch:{ all -> 0x0126 }
            if (r14 != 0) goto L_0x003b
            r5 = 1
            r2.writeSuspended = r15     // Catch:{ all -> 0x0126 }
            goto L_0x00bf
        L_0x003b:
            java.lang.Object r14 = r0.getMessage()     // Catch:{ all -> 0x0126 }
            org.jboss.netty.channel.socket.nio.SocketSendBufferPool$SendBuffer r14 = r9.acquire((java.lang.Object) r14)     // Catch:{ all -> 0x0126 }
            r16 = r14
            r2.currentWriteBuffer = r14     // Catch:{ all -> 0x0126 }
            r14 = r16
            goto L_0x004c
        L_0x004a:
            org.jboss.netty.channel.socket.nio.SocketSendBufferPool$SendBuffer r14 = r2.currentWriteBuffer     // Catch:{ all -> 0x0122 }
        L_0x004c:
            r16 = r0
            org.jboss.netty.channel.ChannelFuture r0 = r16.getFuture()     // Catch:{ all -> 0x0122 }
            r24 = r0
            r17 = 0
            r0 = r12
        L_0x0057:
            r19 = 0
            r15 = 0
            if (r0 <= 0) goto L_0x0081
            long r21 = r14.transferTo(r10)     // Catch:{ AsynchronousCloseException -> 0x0078, Throwable -> 0x0074 }
            r17 = r21
            int r21 = (r17 > r19 ? 1 : (r17 == r19 ? 0 : -1))
            if (r21 == 0) goto L_0x0069
            long r7 = r7 + r17
            goto L_0x0081
        L_0x0069:
            boolean r21 = r14.finished()     // Catch:{ AsynchronousCloseException -> 0x0078, Throwable -> 0x0074 }
            if (r21 == 0) goto L_0x0070
            goto L_0x0081
        L_0x0070:
            int r0 = r0 + -1
            r15 = 0
            goto L_0x0057
        L_0x0074:
            r0 = move-exception
        L_0x0075:
            r15 = 1
            goto L_0x00e4
        L_0x0078:
            r0 = move-exception
            r27 = r3
            r25 = r7
        L_0x007d:
            r28 = r24
            goto L_0x011a
        L_0x0081:
            r25 = r7
            r7 = r17
            boolean r0 = r14.finished()     // Catch:{ AsynchronousCloseException -> 0x0115, Throwable -> 0x00e0, all -> 0x00dc }
            if (r0 == 0) goto L_0x00a5
            r14.release()     // Catch:{ AsynchronousCloseException -> 0x00a1, Throwable -> 0x009d, all -> 0x00dc }
            r2.currentWriteEvent = r15     // Catch:{ AsynchronousCloseException -> 0x00a1, Throwable -> 0x009d, all -> 0x00dc }
            r2.currentWriteBuffer = r15     // Catch:{ AsynchronousCloseException -> 0x00a1, Throwable -> 0x009d, all -> 0x00dc }
            r16 = 0
            r14 = 0
            r24.setSuccess()     // Catch:{ AsynchronousCloseException -> 0x00a1, Throwable -> 0x009d, all -> 0x00dc }
            r7 = r25
            goto L_0x011f
        L_0x009d:
            r0 = move-exception
            r7 = r25
            goto L_0x0075
        L_0x00a1:
            r0 = move-exception
            r27 = r3
            goto L_0x007d
        L_0x00a5:
            r4 = 1
            r15 = 1
            r2.writeSuspended = r15     // Catch:{ AsynchronousCloseException -> 0x00a1, Throwable -> 0x00da, all -> 0x00dc }
            int r0 = (r7 > r19 ? 1 : (r7 == r19 ? 0 : -1))
            if (r0 <= 0) goto L_0x00bc
            long r20 = r14.writtenBytes()     // Catch:{ AsynchronousCloseException -> 0x00a1, Throwable -> 0x00da, all -> 0x00dc }
            long r22 = r14.totalBytes()     // Catch:{ AsynchronousCloseException -> 0x00a1, Throwable -> 0x00da, all -> 0x00dc }
            r17 = r24
            r18 = r7
            r17.setProgress(r18, r20, r22)     // Catch:{ AsynchronousCloseException -> 0x00a1, Throwable -> 0x00da, all -> 0x00dc }
        L_0x00bc:
            r7 = r25
        L_0x00bf:
            r0 = 0
            r2.inWriteNowLoop = r0     // Catch:{ all -> 0x0126 }
            if (r3 == 0) goto L_0x00cf
            if (r4 == 0) goto L_0x00ca
            r30.setOpWrite(r31)     // Catch:{ all -> 0x0126 }
            goto L_0x00cf
        L_0x00ca:
            if (r5 == 0) goto L_0x00cf
            r30.clearOpWrite(r31)     // Catch:{ all -> 0x0126 }
        L_0x00cf:
            monitor-exit(r13)     // Catch:{ all -> 0x0126 }
            if (r6 == 0) goto L_0x00d6
            org.jboss.netty.channel.Channels.fireWriteComplete((org.jboss.netty.channel.Channel) r2, (long) r7)
            goto L_0x00d9
        L_0x00d6:
            org.jboss.netty.channel.Channels.fireWriteCompleteLater(r2, r7)
        L_0x00d9:
            return
        L_0x00da:
            r0 = move-exception
            goto L_0x00e2
        L_0x00dc:
            r0 = move-exception
            r7 = r25
            goto L_0x0127
        L_0x00e0:
            r0 = move-exception
            r15 = 1
        L_0x00e2:
            r7 = r25
        L_0x00e4:
            if (r14 == 0) goto L_0x00e9
            r14.release()     // Catch:{ all -> 0x0126 }
        L_0x00e9:
            r15 = 0
            r2.currentWriteEvent = r15     // Catch:{ all -> 0x0122 }
            r2.currentWriteBuffer = r15     // Catch:{ all -> 0x0122 }
            r14 = 0
            r15 = 0
            r27 = r3
            r3 = r24
            r3.setFailure(r0)     // Catch:{ all -> 0x0111 }
            if (r6 == 0) goto L_0x00fd
            org.jboss.netty.channel.Channels.fireExceptionCaught((org.jboss.netty.channel.Channel) r2, (java.lang.Throwable) r0)     // Catch:{ all -> 0x0111 }
            goto L_0x0100
        L_0x00fd:
            org.jboss.netty.channel.Channels.fireExceptionCaughtLater((org.jboss.netty.channel.Channel) r2, (java.lang.Throwable) r0)     // Catch:{ all -> 0x0111 }
        L_0x0100:
            r28 = r3
            boolean r3 = r0 instanceof java.io.IOException     // Catch:{ all -> 0x0111 }
            if (r3 == 0) goto L_0x011d
            r3 = 0
            r29 = r0
            org.jboss.netty.channel.ChannelFuture r0 = org.jboss.netty.channel.Channels.succeededFuture(r31)     // Catch:{ all -> 0x0126 }
            r1.close(r2, r0)     // Catch:{ all -> 0x0126 }
            goto L_0x011f
        L_0x0111:
            r0 = move-exception
            r3 = r27
            goto L_0x0127
        L_0x0115:
            r0 = move-exception
            r27 = r3
            r28 = r24
        L_0x011a:
            r7 = r25
        L_0x011d:
            r3 = r27
        L_0x011f:
            r14 = 1
            goto L_0x0024
        L_0x0122:
            r0 = move-exception
            r27 = r3
            goto L_0x0127
        L_0x0126:
            r0 = move-exception
        L_0x0127:
            monitor-exit(r13)     // Catch:{ all -> 0x0126 }
            throw r0
        */
        throw new UnsupportedOperationException("Method not decompiled: org.jboss.netty.channel.socket.nio.AbstractNioWorker.write0(org.jboss.netty.channel.socket.nio.AbstractNioChannel):void");
    }

    static boolean isIoThread(AbstractNioChannel<?> channel) {
        return Thread.currentThread() == channel.worker.thread;
    }

    /* access modifiers changed from: protected */
    public void setOpWrite(AbstractNioChannel<?> channel) {
        SelectionKey key = channel.channel.keyFor(this.selector);
        if (key != null) {
            if (!key.isValid()) {
                close(key);
                return;
            }
            synchronized (channel.interestOpsLock) {
                int interestOps = channel.getRawInterestOps();
                if ((interestOps & 4) == 0) {
                    int interestOps2 = interestOps | 4;
                    key.interestOps(interestOps2);
                    channel.setRawInterestOpsNow(interestOps2);
                }
            }
        }
    }

    /* access modifiers changed from: protected */
    public void clearOpWrite(AbstractNioChannel<?> channel) {
        SelectionKey key = channel.channel.keyFor(this.selector);
        if (key != null) {
            if (!key.isValid()) {
                close(key);
                return;
            }
            synchronized (channel.interestOpsLock) {
                int interestOps = channel.getRawInterestOps();
                if ((interestOps & 4) != 0) {
                    int interestOps2 = interestOps & -5;
                    key.interestOps(interestOps2);
                    channel.setRawInterestOpsNow(interestOps2);
                }
            }
        }
    }

    /* access modifiers changed from: package-private */
    public void close(AbstractNioChannel<?> channel, ChannelFuture future) {
        boolean connected = channel.isConnected();
        boolean bound = channel.isBound();
        boolean iothread = isIoThread(channel);
        try {
            channel.channel.close();
            this.cancelledKeys++;
            if (channel.setClosed()) {
                future.setSuccess();
                if (connected) {
                    if (iothread) {
                        Channels.fireChannelDisconnected((Channel) channel);
                    } else {
                        Channels.fireChannelDisconnectedLater(channel);
                    }
                }
                if (bound) {
                    if (iothread) {
                        Channels.fireChannelUnbound((Channel) channel);
                    } else {
                        Channels.fireChannelUnboundLater(channel);
                    }
                }
                cleanUpWriteBuffer(channel);
                if (iothread) {
                    Channels.fireChannelClosed((Channel) channel);
                } else {
                    Channels.fireChannelClosedLater(channel);
                }
            } else {
                future.setSuccess();
            }
        } catch (Throwable t) {
            future.setFailure(t);
            if (iothread) {
                Channels.fireExceptionCaught((Channel) channel, t);
            } else {
                Channels.fireExceptionCaughtLater((Channel) channel, t);
            }
        }
    }

    /* access modifiers changed from: protected */
    public void cleanUpWriteBuffer(AbstractNioChannel<?> channel) {
        Exception cause;
        Exception cause2 = null;
        boolean fireExceptionCaught = false;
        synchronized (channel.writeLock) {
            MessageEvent evt = channel.currentWriteEvent;
            if (evt != null) {
                if (channel.isOpen()) {
                    cause2 = new NotYetConnectedException();
                } else {
                    cause2 = new ClosedChannelException();
                }
                ChannelFuture future = evt.getFuture();
                channel.currentWriteBuffer.release();
                channel.currentWriteBuffer = null;
                channel.currentWriteEvent = null;
                future.setFailure(cause2);
                fireExceptionCaught = true;
            }
            Queue<MessageEvent> writeBuffer = channel.writeBufferQueue;
            while (true) {
                MessageEvent evt2 = writeBuffer.poll();
                if (evt2 != null) {
                    if (cause2 == null) {
                        if (channel.isOpen()) {
                            cause = new NotYetConnectedException();
                        } else {
                            cause = new ClosedChannelException();
                        }
                        fireExceptionCaught = true;
                    }
                    evt2.getFuture().setFailure(cause2);
                }
            }
        }
        if (!fireExceptionCaught) {
            return;
        }
        if (isIoThread(channel)) {
            Channels.fireExceptionCaught((Channel) channel, (Throwable) cause2);
        } else {
            Channels.fireExceptionCaughtLater((Channel) channel, (Throwable) cause2);
        }
    }

    /* JADX INFO: finally extract failed */
    /* access modifiers changed from: package-private */
    /* JADX WARNING: Code restructure failed: missing block: B:41:?, code lost:
        r11.setSuccess();
     */
    /* JADX WARNING: Code restructure failed: missing block: B:42:0x0090, code lost:
        if (r0 == false) goto L_?;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:43:0x0092, code lost:
        if (r1 == false) goto L_0x0098;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:44:0x0094, code lost:
        org.jboss.netty.channel.Channels.fireChannelInterestChanged((org.jboss.netty.channel.Channel) r10);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:45:0x0098, code lost:
        org.jboss.netty.channel.Channels.fireChannelInterestChangedLater(r10);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:58:0x00b9, code lost:
        return;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:76:?, code lost:
        return;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:77:?, code lost:
        return;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:79:?, code lost:
        return;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public void setInterestOps(org.jboss.netty.channel.socket.nio.AbstractNioChannel<?> r10, org.jboss.netty.channel.ChannelFuture r11, int r12) {
        /*
            r9 = this;
            r0 = 0
            boolean r1 = isIoThread(r10)
            java.lang.Object r2 = r10.interestOpsLock     // Catch:{ CancelledKeyException -> 0x00cb, Throwable -> 0x00bd }
            monitor-enter(r2)     // Catch:{ CancelledKeyException -> 0x00cb, Throwable -> 0x00bd }
            java.nio.channels.Selector r3 = r9.selector     // Catch:{ all -> 0x00ba }
            C r4 = r10.channel     // Catch:{ all -> 0x00ba }
            java.nio.channels.SelectionKey r4 = r4.keyFor(r3)     // Catch:{ all -> 0x00ba }
            r12 = r12 & -5
            int r5 = r10.getRawInterestOps()     // Catch:{ all -> 0x00ba }
            r5 = r5 & 4
            r12 = r12 | r5
            if (r4 == 0) goto L_0x00a0
            if (r3 != 0) goto L_0x001f
            goto L_0x00a0
        L_0x001f:
            int r5 = CONSTRAINT_LEVEL     // Catch:{ all -> 0x00ba }
            r6 = 1
            r7 = 0
            switch(r5) {
                case 0: goto L_0x006a;
                case 1: goto L_0x002a;
                case 2: goto L_0x002a;
                default: goto L_0x0026;
            }     // Catch:{ all -> 0x00ba }
        L_0x0026:
            java.lang.Error r5 = new java.lang.Error     // Catch:{ all -> 0x00ba }
            goto L_0x009c
        L_0x002a:
            int r5 = r10.getRawInterestOps()     // Catch:{ all -> 0x00ba }
            if (r5 == r12) goto L_0x0087
            java.lang.Thread r5 = java.lang.Thread.currentThread()     // Catch:{ all -> 0x00ba }
            java.lang.Thread r8 = r9.thread     // Catch:{ all -> 0x00ba }
            if (r5 != r8) goto L_0x003d
            r4.interestOps(r12)     // Catch:{ all -> 0x00ba }
            r0 = 1
            goto L_0x0087
        L_0x003d:
            java.util.concurrent.locks.ReadWriteLock r5 = r9.selectorGuard     // Catch:{ all -> 0x00ba }
            java.util.concurrent.locks.Lock r5 = r5.readLock()     // Catch:{ all -> 0x00ba }
            r5.lock()     // Catch:{ all -> 0x00ba }
            java.util.concurrent.atomic.AtomicBoolean r5 = r9.wakenUp     // Catch:{ all -> 0x005f }
            boolean r5 = r5.compareAndSet(r7, r6)     // Catch:{ all -> 0x005f }
            if (r5 == 0) goto L_0x0051
            r3.wakeup()     // Catch:{ all -> 0x005f }
        L_0x0051:
            r4.interestOps(r12)     // Catch:{ all -> 0x005f }
            r0 = 1
            java.util.concurrent.locks.ReadWriteLock r5 = r9.selectorGuard     // Catch:{ all -> 0x00ba }
            java.util.concurrent.locks.Lock r5 = r5.readLock()     // Catch:{ all -> 0x00ba }
            r5.unlock()     // Catch:{ all -> 0x00ba }
            goto L_0x0087
        L_0x005f:
            r5 = move-exception
            java.util.concurrent.locks.ReadWriteLock r6 = r9.selectorGuard     // Catch:{ all -> 0x00ba }
            java.util.concurrent.locks.Lock r6 = r6.readLock()     // Catch:{ all -> 0x00ba }
            r6.unlock()     // Catch:{ all -> 0x00ba }
            throw r5     // Catch:{ all -> 0x00ba }
        L_0x006a:
            int r5 = r10.getRawInterestOps()     // Catch:{ all -> 0x00ba }
            if (r5 == r12) goto L_0x0087
            r4.interestOps(r12)     // Catch:{ all -> 0x00ba }
            java.lang.Thread r5 = java.lang.Thread.currentThread()     // Catch:{ all -> 0x00ba }
            java.lang.Thread r8 = r9.thread     // Catch:{ all -> 0x00ba }
            if (r5 == r8) goto L_0x0086
            java.util.concurrent.atomic.AtomicBoolean r5 = r9.wakenUp     // Catch:{ all -> 0x00ba }
            boolean r5 = r5.compareAndSet(r7, r6)     // Catch:{ all -> 0x00ba }
            if (r5 == 0) goto L_0x0086
            r3.wakeup()     // Catch:{ all -> 0x00ba }
        L_0x0086:
            r0 = 1
        L_0x0087:
            if (r0 == 0) goto L_0x008c
            r10.setRawInterestOpsNow(r12)     // Catch:{ all -> 0x00ba }
        L_0x008c:
            monitor-exit(r2)     // Catch:{ all -> 0x00ba }
            r11.setSuccess()     // Catch:{ CancelledKeyException -> 0x00cb, Throwable -> 0x00bd }
            if (r0 == 0) goto L_0x00dd
            if (r1 == 0) goto L_0x0098
            org.jboss.netty.channel.Channels.fireChannelInterestChanged((org.jboss.netty.channel.Channel) r10)     // Catch:{ CancelledKeyException -> 0x00cb, Throwable -> 0x00bd }
            goto L_0x00dd
        L_0x0098:
            org.jboss.netty.channel.Channels.fireChannelInterestChangedLater(r10)     // Catch:{ CancelledKeyException -> 0x00cb, Throwable -> 0x00bd }
            goto L_0x00dd
        L_0x009c:
            r5.<init>()     // Catch:{ all -> 0x00ba }
            throw r5     // Catch:{ all -> 0x00ba }
        L_0x00a0:
            int r5 = r10.getRawInterestOps()     // Catch:{ all -> 0x00ba }
            if (r5 == r12) goto L_0x00a7
            r0 = 1
        L_0x00a7:
            r10.setRawInterestOpsNow(r12)     // Catch:{ all -> 0x00ba }
            r11.setSuccess()     // Catch:{ all -> 0x00ba }
            if (r0 == 0) goto L_0x00b8
            if (r1 == 0) goto L_0x00b5
            org.jboss.netty.channel.Channels.fireChannelInterestChanged((org.jboss.netty.channel.Channel) r10)     // Catch:{ all -> 0x00ba }
            goto L_0x00b8
        L_0x00b5:
            org.jboss.netty.channel.Channels.fireChannelInterestChangedLater(r10)     // Catch:{ all -> 0x00ba }
        L_0x00b8:
            monitor-exit(r2)     // Catch:{ all -> 0x00ba }
            return
        L_0x00ba:
            r3 = move-exception
            monitor-exit(r2)     // Catch:{ all -> 0x00ba }
            throw r3     // Catch:{ CancelledKeyException -> 0x00cb, Throwable -> 0x00bd }
        L_0x00bd:
            r2 = move-exception
            r11.setFailure(r2)
            if (r1 == 0) goto L_0x00c7
            org.jboss.netty.channel.Channels.fireExceptionCaught((org.jboss.netty.channel.Channel) r10, (java.lang.Throwable) r2)
            goto L_0x00de
        L_0x00c7:
            org.jboss.netty.channel.Channels.fireExceptionCaughtLater((org.jboss.netty.channel.Channel) r10, (java.lang.Throwable) r2)
            goto L_0x00de
        L_0x00cb:
            r2 = move-exception
            java.nio.channels.ClosedChannelException r3 = new java.nio.channels.ClosedChannelException
            r3.<init>()
            r11.setFailure(r3)
            if (r1 == 0) goto L_0x00da
            org.jboss.netty.channel.Channels.fireExceptionCaught((org.jboss.netty.channel.Channel) r10, (java.lang.Throwable) r3)
            goto L_0x00dd
        L_0x00da:
            org.jboss.netty.channel.Channels.fireExceptionCaughtLater((org.jboss.netty.channel.Channel) r10, (java.lang.Throwable) r3)
        L_0x00dd:
        L_0x00de:
            return
        */
        throw new UnsupportedOperationException("Method not decompiled: org.jboss.netty.channel.socket.nio.AbstractNioWorker.setInterestOps(org.jboss.netty.channel.socket.nio.AbstractNioChannel, org.jboss.netty.channel.ChannelFuture, int):void");
    }
}
