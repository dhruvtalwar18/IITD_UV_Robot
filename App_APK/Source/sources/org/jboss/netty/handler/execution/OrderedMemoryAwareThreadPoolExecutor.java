package org.jboss.netty.handler.execution;

import java.util.Queue;
import java.util.Set;
import java.util.concurrent.ConcurrentMap;
import java.util.concurrent.Executor;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import org.jboss.netty.channel.ChannelEvent;
import org.jboss.netty.util.ObjectSizeEstimator;
import org.jboss.netty.util.internal.ConcurrentIdentityWeakKeyHashMap;
import org.jboss.netty.util.internal.QueueFactory;

public class OrderedMemoryAwareThreadPoolExecutor extends MemoryAwareThreadPoolExecutor {
    protected final ConcurrentMap<Object, Executor> childExecutors = newChildExecutorMap();

    public OrderedMemoryAwareThreadPoolExecutor(int corePoolSize, long maxChannelMemorySize, long maxTotalMemorySize) {
        super(corePoolSize, maxChannelMemorySize, maxTotalMemorySize);
    }

    public OrderedMemoryAwareThreadPoolExecutor(int corePoolSize, long maxChannelMemorySize, long maxTotalMemorySize, long keepAliveTime, TimeUnit unit) {
        super(corePoolSize, maxChannelMemorySize, maxTotalMemorySize, keepAliveTime, unit);
    }

    public OrderedMemoryAwareThreadPoolExecutor(int corePoolSize, long maxChannelMemorySize, long maxTotalMemorySize, long keepAliveTime, TimeUnit unit, ThreadFactory threadFactory) {
        super(corePoolSize, maxChannelMemorySize, maxTotalMemorySize, keepAliveTime, unit, threadFactory);
    }

    public OrderedMemoryAwareThreadPoolExecutor(int corePoolSize, long maxChannelMemorySize, long maxTotalMemorySize, long keepAliveTime, TimeUnit unit, ObjectSizeEstimator objectSizeEstimator, ThreadFactory threadFactory) {
        super(corePoolSize, maxChannelMemorySize, maxTotalMemorySize, keepAliveTime, unit, objectSizeEstimator, threadFactory);
    }

    /* access modifiers changed from: protected */
    public ConcurrentMap<Object, Executor> newChildExecutorMap() {
        return new ConcurrentIdentityWeakKeyHashMap();
    }

    /* access modifiers changed from: protected */
    public Object getChildExecutorKey(ChannelEvent e) {
        return e.getChannel();
    }

    /* access modifiers changed from: protected */
    public Set<Object> getChildExecutorKeySet() {
        return this.childExecutors.keySet();
    }

    /* access modifiers changed from: protected */
    public boolean removeChildExecutor(Object key) {
        return this.childExecutors.remove(key) != null;
    }

    /* access modifiers changed from: protected */
    public void doExecute(Runnable task) {
        if (!(task instanceof ChannelEventRunnable)) {
            doUnorderedExecute(task);
        } else {
            getChildExecutor(((ChannelEventRunnable) task).getEvent()).execute(task);
        }
    }

    /* access modifiers changed from: protected */
    /* JADX WARNING: Code restructure failed: missing block: B:2:0x000e, code lost:
        r1 = new org.jboss.netty.handler.execution.OrderedMemoryAwareThreadPoolExecutor.ChildExecutor(r6);
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public java.util.concurrent.Executor getChildExecutor(org.jboss.netty.channel.ChannelEvent r7) {
        /*
            r6 = this;
            java.lang.Object r0 = r6.getChildExecutorKey(r7)
            java.util.concurrent.ConcurrentMap<java.lang.Object, java.util.concurrent.Executor> r1 = r6.childExecutors
            java.lang.Object r1 = r1.get(r0)
            java.util.concurrent.Executor r1 = (java.util.concurrent.Executor) r1
            if (r1 != 0) goto L_0x001f
            org.jboss.netty.handler.execution.OrderedMemoryAwareThreadPoolExecutor$ChildExecutor r2 = new org.jboss.netty.handler.execution.OrderedMemoryAwareThreadPoolExecutor$ChildExecutor
            r2.<init>()
            r1 = r2
            java.util.concurrent.ConcurrentMap<java.lang.Object, java.util.concurrent.Executor> r2 = r6.childExecutors
            java.lang.Object r2 = r2.putIfAbsent(r0, r1)
            java.util.concurrent.Executor r2 = (java.util.concurrent.Executor) r2
            if (r2 == 0) goto L_0x001f
            r1 = r2
        L_0x001f:
            boolean r2 = r7 instanceof org.jboss.netty.channel.ChannelStateEvent
            if (r2 == 0) goto L_0x003b
            org.jboss.netty.channel.Channel r2 = r7.getChannel()
            r3 = r7
            org.jboss.netty.channel.ChannelStateEvent r3 = (org.jboss.netty.channel.ChannelStateEvent) r3
            org.jboss.netty.channel.ChannelState r4 = r3.getState()
            org.jboss.netty.channel.ChannelState r5 = org.jboss.netty.channel.ChannelState.OPEN
            if (r4 != r5) goto L_0x003b
            boolean r4 = r2.isOpen()
            if (r4 != 0) goto L_0x003b
            r6.removeChildExecutor(r0)
        L_0x003b:
            return r1
        */
        throw new UnsupportedOperationException("Method not decompiled: org.jboss.netty.handler.execution.OrderedMemoryAwareThreadPoolExecutor.getChildExecutor(org.jboss.netty.channel.ChannelEvent):java.util.concurrent.Executor");
    }

    /* access modifiers changed from: protected */
    public boolean shouldCount(Runnable task) {
        if (task instanceof ChildExecutor) {
            return false;
        }
        return super.shouldCount(task);
    }

    /* access modifiers changed from: package-private */
    public void onAfterExecute(Runnable r, Throwable t) {
        afterExecute(r, t);
    }

    protected final class ChildExecutor implements Executor, Runnable {
        private final AtomicBoolean isRunning = new AtomicBoolean();
        private final Queue<Runnable> tasks = QueueFactory.createQueue(Runnable.class);

        protected ChildExecutor() {
        }

        public void execute(Runnable command) {
            this.tasks.add(command);
            if (!this.isRunning.get()) {
                OrderedMemoryAwareThreadPoolExecutor.this.doUnorderedExecute(this);
            }
        }

        public void run() {
            Runnable task;
            boolean ran;
            if (this.isRunning.compareAndSet(false, true)) {
                try {
                    Thread thread = Thread.currentThread();
                    while (true) {
                        task = this.tasks.poll();
                        if (task == null) {
                            break;
                        }
                        ran = false;
                        OrderedMemoryAwareThreadPoolExecutor.this.beforeExecute(thread, task);
                        task.run();
                        ran = true;
                        OrderedMemoryAwareThreadPoolExecutor.this.onAfterExecute(task, (Throwable) null);
                    }
                    this.isRunning.set(false);
                    if (1 != 0 && !this.isRunning.get() && this.tasks.peek() != null) {
                        OrderedMemoryAwareThreadPoolExecutor.this.doUnorderedExecute(this);
                    }
                } catch (RuntimeException e) {
                    if (!ran) {
                        OrderedMemoryAwareThreadPoolExecutor.this.onAfterExecute(task, e);
                    }
                    throw e;
                } catch (Throwable th) {
                    this.isRunning.set(false);
                    throw th;
                }
            }
        }
    }
}
