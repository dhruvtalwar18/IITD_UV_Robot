package org.jboss.netty.channel.group;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;
import org.jboss.netty.channel.Channel;
import org.jboss.netty.channel.ChannelFuture;
import org.jboss.netty.channel.ChannelFutureListener;
import org.jboss.netty.logging.InternalLogger;
import org.jboss.netty.logging.InternalLoggerFactory;
import org.jboss.netty.util.internal.DeadLockProofWorker;

public class DefaultChannelGroupFuture implements ChannelGroupFuture {
    private static final InternalLogger logger = InternalLoggerFactory.getInstance((Class<?>) DefaultChannelGroupFuture.class);
    private final ChannelFutureListener childListener = new ChannelFutureListener() {
        static final /* synthetic */ boolean $assertionsDisabled = false;

        static {
            Class<DefaultChannelGroupFuture> cls = DefaultChannelGroupFuture.class;
        }

        public void operationComplete(ChannelFuture future) throws Exception {
            boolean callSetDone;
            boolean success = future.isSuccess();
            synchronized (DefaultChannelGroupFuture.this) {
                boolean z = true;
                if (success) {
                    try {
                        DefaultChannelGroupFuture.this.successCount++;
                    } catch (Throwable th) {
                        while (true) {
                            throw th;
                        }
                    }
                } else {
                    DefaultChannelGroupFuture.this.failureCount++;
                }
                if (DefaultChannelGroupFuture.this.successCount + DefaultChannelGroupFuture.this.failureCount != DefaultChannelGroupFuture.this.futures.size()) {
                    z = false;
                }
                callSetDone = z;
            }
            if (callSetDone) {
                DefaultChannelGroupFuture.this.setDone();
            }
        }
    };
    private boolean done;
    int failureCount;
    private ChannelGroupFutureListener firstListener;
    final Map<Integer, ChannelFuture> futures;
    private final ChannelGroup group;
    private List<ChannelGroupFutureListener> otherListeners;
    int successCount;
    private int waiters;

    public DefaultChannelGroupFuture(ChannelGroup group2, Collection<ChannelFuture> futures2) {
        if (group2 == null) {
            throw new NullPointerException("group");
        } else if (futures2 != null) {
            this.group = group2;
            Map<Integer, ChannelFuture> futureMap = new LinkedHashMap<>();
            for (ChannelFuture f : futures2) {
                futureMap.put(f.getChannel().getId(), f);
            }
            this.futures = Collections.unmodifiableMap(futureMap);
            for (ChannelFuture f2 : this.futures.values()) {
                f2.addListener(this.childListener);
            }
            if (this.futures.isEmpty()) {
                setDone();
            }
        } else {
            throw new NullPointerException("futures");
        }
    }

    DefaultChannelGroupFuture(ChannelGroup group2, Map<Integer, ChannelFuture> futures2) {
        this.group = group2;
        this.futures = Collections.unmodifiableMap(futures2);
        for (ChannelFuture f : this.futures.values()) {
            f.addListener(this.childListener);
        }
        if (this.futures.isEmpty()) {
            setDone();
        }
    }

    public ChannelGroup getGroup() {
        return this.group;
    }

    public ChannelFuture find(Integer channelId) {
        return this.futures.get(channelId);
    }

    public ChannelFuture find(Channel channel) {
        return this.futures.get(channel.getId());
    }

    public Iterator<ChannelFuture> iterator() {
        return this.futures.values().iterator();
    }

    public synchronized boolean isDone() {
        return this.done;
    }

    public synchronized boolean isCompleteSuccess() {
        return this.successCount == this.futures.size();
    }

    public synchronized boolean isPartialSuccess() {
        return (this.successCount == 0 || this.successCount == this.futures.size()) ? false : true;
    }

    public synchronized boolean isPartialFailure() {
        return (this.failureCount == 0 || this.failureCount == this.futures.size()) ? false : true;
    }

    public synchronized boolean isCompleteFailure() {
        int futureCnt;
        futureCnt = this.futures.size();
        return futureCnt != 0 && this.failureCount == futureCnt;
    }

    public void addListener(ChannelGroupFutureListener listener) {
        if (listener != null) {
            boolean notifyNow = false;
            synchronized (this) {
                if (this.done) {
                    notifyNow = true;
                } else if (this.firstListener == null) {
                    this.firstListener = listener;
                } else {
                    if (this.otherListeners == null) {
                        this.otherListeners = new ArrayList(1);
                    }
                    this.otherListeners.add(listener);
                }
            }
            if (notifyNow) {
                notifyListener(listener);
                return;
            }
            return;
        }
        throw new NullPointerException("listener");
    }

    public void removeListener(ChannelGroupFutureListener listener) {
        if (listener != null) {
            synchronized (this) {
                if (!this.done) {
                    if (listener == this.firstListener) {
                        if (this.otherListeners == null || this.otherListeners.isEmpty()) {
                            this.firstListener = null;
                        } else {
                            this.firstListener = this.otherListeners.remove(0);
                        }
                    } else if (this.otherListeners != null) {
                        this.otherListeners.remove(listener);
                    }
                }
            }
            return;
        }
        throw new NullPointerException("listener");
    }

    /* JADX INFO: finally extract failed */
    public ChannelGroupFuture await() throws InterruptedException {
        if (!Thread.interrupted()) {
            synchronized (this) {
                while (!this.done) {
                    checkDeadLock();
                    this.waiters++;
                    try {
                        wait();
                        this.waiters--;
                    } catch (Throwable th) {
                        this.waiters--;
                        throw th;
                    }
                }
            }
            return this;
        }
        throw new InterruptedException();
    }

    public boolean await(long timeout, TimeUnit unit) throws InterruptedException {
        return await0(unit.toNanos(timeout), true);
    }

    public boolean await(long timeoutMillis) throws InterruptedException {
        return await0(TimeUnit.MILLISECONDS.toNanos(timeoutMillis), true);
    }

    public ChannelGroupFuture awaitUninterruptibly() {
        boolean interrupted;
        int i;
        synchronized (this) {
            interrupted = false;
            while (!this.done) {
                checkDeadLock();
                this.waiters++;
                try {
                    wait();
                    i = this.waiters;
                } catch (InterruptedException e) {
                    interrupted = true;
                    i = this.waiters;
                } catch (Throwable th) {
                    this.waiters--;
                    throw th;
                }
                this.waiters = i - 1;
            }
        }
        if (interrupted) {
            Thread.currentThread().interrupt();
        }
        return this;
    }

    public boolean awaitUninterruptibly(long timeout, TimeUnit unit) {
        try {
            return await0(unit.toNanos(timeout), false);
        } catch (InterruptedException e) {
            throw new InternalError();
        }
    }

    public boolean awaitUninterruptibly(long timeoutMillis) {
        try {
            return await0(TimeUnit.MILLISECONDS.toNanos(timeoutMillis), false);
        } catch (InterruptedException e) {
            throw new InternalError();
        }
    }

    /* JADX WARNING: Code restructure failed: missing block: B:18:0x0028, code lost:
        if (0 == 0) goto L_0x0031;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:19:0x002a, code lost:
        java.lang.Thread.currentThread().interrupt();
     */
    /* JADX WARNING: Code restructure failed: missing block: B:20:0x0031, code lost:
        return r0;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:26:0x0039, code lost:
        if (0 == 0) goto L_0x0042;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:27:0x003b, code lost:
        java.lang.Thread.currentThread().interrupt();
     */
    /* JADX WARNING: Code restructure failed: missing block: B:28:0x0042, code lost:
        return r0;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:45:0x0072, code lost:
        if (r10 == false) goto L_0x007b;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:46:0x0074, code lost:
        java.lang.Thread.currentThread().interrupt();
     */
    /* JADX WARNING: Code restructure failed: missing block: B:47:0x007b, code lost:
        return true;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:56:0x0093, code lost:
        if (r5 == false) goto L_0x009c;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:57:0x0095, code lost:
        java.lang.Thread.currentThread().interrupt();
     */
    /* JADX WARNING: Code restructure failed: missing block: B:58:0x009c, code lost:
        return r0;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:76:0x00c1, code lost:
        r0 = th;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    private boolean await0(long r16, boolean r18) throws java.lang.InterruptedException {
        /*
            r15 = this;
            r1 = r15
            if (r18 == 0) goto L_0x0010
            boolean r0 = java.lang.Thread.interrupted()
            if (r0 != 0) goto L_0x000a
            goto L_0x0010
        L_0x000a:
            java.lang.InterruptedException r0 = new java.lang.InterruptedException
            r0.<init>()
            throw r0
        L_0x0010:
            r5 = 0
            int r0 = (r16 > r5 ? 1 : (r16 == r5 ? 0 : -1))
            if (r0 > 0) goto L_0x0018
            r7 = r5
            goto L_0x001c
        L_0x0018:
            long r7 = java.lang.System.nanoTime()
        L_0x001c:
            r9 = r16
            r0 = 0
            r11 = r0
            monitor-enter(r15)     // Catch:{ all -> 0x00c5 }
            boolean r0 = r1.done     // Catch:{ all -> 0x00b9 }
            if (r0 == 0) goto L_0x0032
            boolean r0 = r1.done     // Catch:{ all -> 0x00b9 }
            monitor-exit(r15)     // Catch:{ all -> 0x00b9 }
            if (r11 == 0) goto L_0x0031
            java.lang.Thread r5 = java.lang.Thread.currentThread()
            r5.interrupt()
        L_0x0031:
            return r0
        L_0x0032:
            int r0 = (r9 > r5 ? 1 : (r9 == r5 ? 0 : -1))
            if (r0 > 0) goto L_0x0043
            boolean r0 = r1.done     // Catch:{ all -> 0x00b9 }
            monitor-exit(r15)     // Catch:{ all -> 0x00b9 }
            if (r11 == 0) goto L_0x0042
            java.lang.Thread r5 = java.lang.Thread.currentThread()
            r5.interrupt()
        L_0x0042:
            return r0
        L_0x0043:
            checkDeadLock()     // Catch:{ all -> 0x00b9 }
            int r0 = r1.waiters     // Catch:{ all -> 0x00b9 }
            r12 = 1
            int r0 = r0 + r12
            r1.waiters = r0     // Catch:{ all -> 0x00b9 }
        L_0x004c:
            r13 = 1000000(0xf4240, double:4.940656E-318)
            long r5 = r9 / r13
            long r13 = r9 % r13
            int r0 = (int) r13     // Catch:{ InterruptedException -> 0x005a }
            r15.wait(r5, r0)     // Catch:{ InterruptedException -> 0x005a }
            goto L_0x005f
        L_0x0058:
            r0 = move-exception
            goto L_0x00a5
        L_0x005a:
            r0 = move-exception
            if (r18 != 0) goto L_0x00a4
            r0 = 1
            r11 = r0
        L_0x005f:
            boolean r0 = r1.done     // Catch:{ all -> 0x0058 }
            if (r0 == 0) goto L_0x007c
            r5 = r15
            r2 = r16
            r4 = r18
            r6 = r7
            r8 = r9
            r10 = r11
            int r0 = r5.waiters     // Catch:{ all -> 0x00b4 }
            int r0 = r0 - r12
            r5.waiters = r0     // Catch:{ all -> 0x00b4 }
            monitor-exit(r15)     // Catch:{ all -> 0x00b4 }
            if (r10 == 0) goto L_0x007b
            java.lang.Thread r0 = java.lang.Thread.currentThread()
            r0.interrupt()
        L_0x007b:
            return r12
        L_0x007c:
            long r5 = java.lang.System.nanoTime()     // Catch:{ all -> 0x0058 }
            r0 = 0
            long r5 = r5 - r7
            long r9 = r16 - r5
            r5 = 0
            int r0 = (r9 > r5 ? 1 : (r9 == r5 ? 0 : -1))
            if (r0 > 0) goto L_0x004c
            boolean r0 = r1.done     // Catch:{ all -> 0x0058 }
            r5 = r11
            int r6 = r1.waiters     // Catch:{ all -> 0x009d }
            int r6 = r6 - r12
            r1.waiters = r6     // Catch:{ all -> 0x009d }
            monitor-exit(r15)     // Catch:{ all -> 0x009d }
            if (r5 == 0) goto L_0x009c
            java.lang.Thread r6 = java.lang.Thread.currentThread()
            r6.interrupt()
        L_0x009c:
            return r0
        L_0x009d:
            r0 = move-exception
            r2 = r16
            r4 = r18
            r11 = r5
            goto L_0x00be
        L_0x00a4:
            throw r0     // Catch:{ all -> 0x0058 }
        L_0x00a5:
            r5 = r15
            r2 = r16
            r4 = r18
            r6 = r7
            r8 = r9
            r10 = r11
            int r11 = r5.waiters     // Catch:{ all -> 0x00b4 }
            int r11 = r11 - r12
            r5.waiters = r11     // Catch:{ all -> 0x00b4 }
            throw r0     // Catch:{ all -> 0x00b4 }
        L_0x00b4:
            r0 = move-exception
            r11 = r10
            r9 = r8
            r7 = r6
            goto L_0x00bf
        L_0x00b9:
            r0 = move-exception
            r2 = r16
            r4 = r18
        L_0x00be:
            r5 = r1
        L_0x00bf:
            monitor-exit(r15)     // Catch:{ all -> 0x00c3 }
            throw r0     // Catch:{ all -> 0x00c1 }
        L_0x00c1:
            r0 = move-exception
            goto L_0x00cb
        L_0x00c3:
            r0 = move-exception
            goto L_0x00bf
        L_0x00c5:
            r0 = move-exception
            r2 = r16
            r4 = r18
            r5 = r1
        L_0x00cb:
            r6 = r7
            r8 = r9
            r10 = r11
            if (r10 == 0) goto L_0x00d7
            java.lang.Thread r11 = java.lang.Thread.currentThread()
            r11.interrupt()
        L_0x00d7:
            throw r0
        */
        throw new UnsupportedOperationException("Method not decompiled: org.jboss.netty.channel.group.DefaultChannelGroupFuture.await0(long, boolean):boolean");
    }

    private static void checkDeadLock() {
        if (DeadLockProofWorker.PARENT.get() != null) {
            throw new IllegalStateException("await*() in I/O thread causes a dead lock or sudden performance drop. Use addListener() instead or call await*() from a different thread.");
        }
    }

    /* access modifiers changed from: package-private */
    /* JADX WARNING: Code restructure failed: missing block: B:11:0x0013, code lost:
        notifyListeners();
     */
    /* JADX WARNING: Code restructure failed: missing block: B:12:0x0016, code lost:
        return true;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public boolean setDone() {
        /*
            r2 = this;
            monitor-enter(r2)
            boolean r0 = r2.done     // Catch:{ all -> 0x0017 }
            if (r0 == 0) goto L_0x0008
            r0 = 0
            monitor-exit(r2)     // Catch:{ all -> 0x0017 }
            return r0
        L_0x0008:
            r0 = 1
            r2.done = r0     // Catch:{ all -> 0x0017 }
            int r1 = r2.waiters     // Catch:{ all -> 0x0017 }
            if (r1 <= 0) goto L_0x0012
            r2.notifyAll()     // Catch:{ all -> 0x0017 }
        L_0x0012:
            monitor-exit(r2)     // Catch:{ all -> 0x0017 }
            r2.notifyListeners()
            return r0
        L_0x0017:
            r0 = move-exception
            monitor-exit(r2)     // Catch:{ all -> 0x0017 }
            throw r0
        */
        throw new UnsupportedOperationException("Method not decompiled: org.jboss.netty.channel.group.DefaultChannelGroupFuture.setDone():boolean");
    }

    private void notifyListeners() {
        if (this.firstListener != null) {
            notifyListener(this.firstListener);
            this.firstListener = null;
            if (this.otherListeners != null) {
                for (ChannelGroupFutureListener l : this.otherListeners) {
                    notifyListener(l);
                }
                this.otherListeners = null;
            }
        }
    }

    private void notifyListener(ChannelGroupFutureListener l) {
        try {
            l.operationComplete(this);
        } catch (Throwable t) {
            if (logger.isWarnEnabled()) {
                InternalLogger internalLogger = logger;
                internalLogger.warn("An exception was thrown by " + ChannelFutureListener.class.getSimpleName() + ".", t);
            }
        }
    }
}
