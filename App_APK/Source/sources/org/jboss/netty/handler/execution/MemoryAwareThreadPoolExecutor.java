package org.jboss.netty.handler.execution;

import java.io.IOException;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.concurrent.ConcurrentMap;
import java.util.concurrent.Executors;
import java.util.concurrent.RejectedExecutionException;
import java.util.concurrent.RejectedExecutionHandler;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicLong;
import org.jboss.netty.channel.Channel;
import org.jboss.netty.channel.ChannelEvent;
import org.jboss.netty.channel.ChannelHandlerContext;
import org.jboss.netty.channel.ChannelState;
import org.jboss.netty.channel.ChannelStateEvent;
import org.jboss.netty.channel.Channels;
import org.jboss.netty.channel.WriteCompletionEvent;
import org.jboss.netty.logging.InternalLogger;
import org.jboss.netty.logging.InternalLoggerFactory;
import org.jboss.netty.util.DefaultObjectSizeEstimator;
import org.jboss.netty.util.ObjectSizeEstimator;
import org.jboss.netty.util.internal.ConcurrentIdentityHashMap;
import org.jboss.netty.util.internal.QueueFactory;
import org.jboss.netty.util.internal.SharedResourceMisuseDetector;

public class MemoryAwareThreadPoolExecutor extends ThreadPoolExecutor {
    private static final InternalLogger logger = InternalLoggerFactory.getInstance((Class<?>) MemoryAwareThreadPoolExecutor.class);
    private static final SharedResourceMisuseDetector misuseDetector = new SharedResourceMisuseDetector(MemoryAwareThreadPoolExecutor.class);
    private final ConcurrentMap<Channel, AtomicLong> channelCounters;
    private volatile boolean notifyOnShutdown;
    private volatile Settings settings;
    private final Limiter totalLimiter;

    public MemoryAwareThreadPoolExecutor(int corePoolSize, long maxChannelMemorySize, long maxTotalMemorySize) {
        this(corePoolSize, maxChannelMemorySize, maxTotalMemorySize, 30, TimeUnit.SECONDS);
    }

    public MemoryAwareThreadPoolExecutor(int corePoolSize, long maxChannelMemorySize, long maxTotalMemorySize, long keepAliveTime, TimeUnit unit) {
        this(corePoolSize, maxChannelMemorySize, maxTotalMemorySize, keepAliveTime, unit, Executors.defaultThreadFactory());
    }

    public MemoryAwareThreadPoolExecutor(int corePoolSize, long maxChannelMemorySize, long maxTotalMemorySize, long keepAliveTime, TimeUnit unit, ThreadFactory threadFactory) {
        this(corePoolSize, maxChannelMemorySize, maxTotalMemorySize, keepAliveTime, unit, new DefaultObjectSizeEstimator(), threadFactory);
    }

    /* JADX INFO: super call moved to the top of the method (can break code semantics) */
    public MemoryAwareThreadPoolExecutor(int corePoolSize, long maxChannelMemorySize, long maxTotalMemorySize, long keepAliveTime, TimeUnit unit, ObjectSizeEstimator objectSizeEstimator, ThreadFactory threadFactory) {
        super(corePoolSize, corePoolSize, keepAliveTime, unit, QueueFactory.createQueue(Runnable.class), threadFactory, new NewThreadRunsPolicy());
        long j = maxChannelMemorySize;
        long j2 = maxTotalMemorySize;
        ObjectSizeEstimator objectSizeEstimator2 = objectSizeEstimator;
        this.channelCounters = new ConcurrentIdentityHashMap();
        if (objectSizeEstimator2 == null) {
            throw new NullPointerException("objectSizeEstimator");
        } else if (j < 0) {
            throw new IllegalArgumentException("maxChannelMemorySize: " + j);
        } else if (j2 >= 0) {
            try {
                getClass().getMethod("allowCoreThreadTimeOut", new Class[]{Boolean.TYPE}).invoke(this, new Object[]{Boolean.TRUE});
            } catch (Throwable th) {
                logger.debug("ThreadPoolExecutor.allowCoreThreadTimeOut() is not supported in this platform.");
            }
            this.settings = new Settings(objectSizeEstimator2, j);
            if (j2 == 0) {
                this.totalLimiter = null;
            } else {
                this.totalLimiter = new Limiter(j2);
            }
            misuseDetector.increase();
        } else {
            throw new IllegalArgumentException("maxTotalMemorySize: " + j2);
        }
    }

    /* access modifiers changed from: protected */
    public void terminated() {
        super.terminated();
        misuseDetector.decrease();
    }

    public List<Runnable> shutdownNow() {
        return shutdownNow(this.notifyOnShutdown);
    }

    public List<Runnable> shutdownNow(boolean notify) {
        if (!notify) {
            return super.shutdownNow();
        }
        Throwable cause = null;
        Set<Channel> channels = null;
        List<Runnable> tasks = super.shutdownNow();
        for (Runnable task : tasks) {
            if (task instanceof ChannelEventRunnable) {
                if (cause == null) {
                    cause = new IOException("Unable to process queued event");
                }
                ChannelEvent event = ((ChannelEventRunnable) task).getEvent();
                event.getFuture().setFailure(cause);
                if (channels == null) {
                    channels = new HashSet<>();
                }
                channels.add(event.getChannel());
            }
        }
        if (channels != null) {
            for (Channel channel : channels) {
                Channels.fireExceptionCaughtLater(channel, cause);
            }
        }
        return tasks;
    }

    public ObjectSizeEstimator getObjectSizeEstimator() {
        return this.settings.objectSizeEstimator;
    }

    public void setObjectSizeEstimator(ObjectSizeEstimator objectSizeEstimator) {
        if (objectSizeEstimator != null) {
            this.settings = new Settings(objectSizeEstimator, this.settings.maxChannelMemorySize);
            return;
        }
        throw new NullPointerException("objectSizeEstimator");
    }

    public long getMaxChannelMemorySize() {
        return this.settings.maxChannelMemorySize;
    }

    public void setMaxChannelMemorySize(long maxChannelMemorySize) {
        if (maxChannelMemorySize < 0) {
            throw new IllegalArgumentException("maxChannelMemorySize: " + maxChannelMemorySize);
        } else if (getTaskCount() <= 0) {
            this.settings = new Settings(this.settings.objectSizeEstimator, maxChannelMemorySize);
        } else {
            throw new IllegalStateException("can't be changed after a task is executed");
        }
    }

    public long getMaxTotalMemorySize() {
        return this.totalLimiter.limit;
    }

    @Deprecated
    public void setMaxTotalMemorySize(long maxTotalMemorySize) {
        if (maxTotalMemorySize < 0) {
            throw new IllegalArgumentException("maxTotalMemorySize: " + maxTotalMemorySize);
        } else if (getTaskCount() > 0) {
            throw new IllegalStateException("can't be changed after a task is executed");
        }
    }

    public void setNotifyChannelFuturesOnShutdown(boolean notifyOnShutdown2) {
        this.notifyOnShutdown = notifyOnShutdown2;
    }

    public boolean getNotifyChannelFuturesOnShutdown() {
        return this.notifyOnShutdown;
    }

    public void execute(Runnable command) {
        if (!(command instanceof ChannelDownstreamEventRunnable)) {
            if (!(command instanceof ChannelEventRunnable)) {
                command = new MemoryAwareRunnable(command);
            }
            increaseCounter(command);
            doExecute(command);
            return;
        }
        throw new RejectedExecutionException("command must be enclosed with an upstream event.");
    }

    /* access modifiers changed from: protected */
    public void doExecute(Runnable task) {
        doUnorderedExecute(task);
    }

    /* access modifiers changed from: protected */
    public final void doUnorderedExecute(Runnable task) {
        super.execute(task);
    }

    public boolean remove(Runnable task) {
        boolean removed = super.remove(task);
        if (removed) {
            decreaseCounter(task);
        }
        return removed;
    }

    /* access modifiers changed from: protected */
    public void beforeExecute(Thread t, Runnable r) {
        super.beforeExecute(t, r);
        decreaseCounter(r);
    }

    /* access modifiers changed from: protected */
    public void increaseCounter(Runnable task) {
        if (shouldCount(task)) {
            Settings settings2 = this.settings;
            long maxChannelMemorySize = settings2.maxChannelMemorySize;
            int increment = settings2.objectSizeEstimator.estimateSize(task);
            if (task instanceof ChannelEventRunnable) {
                ChannelEventRunnable eventTask = (ChannelEventRunnable) task;
                eventTask.estimatedSize = increment;
                Channel channel = eventTask.getEvent().getChannel();
                long channelCounter = getChannelCounter(channel).addAndGet((long) increment);
                if (maxChannelMemorySize != 0 && channelCounter >= maxChannelMemorySize && channel.isOpen() && channel.isReadable()) {
                    ChannelHandlerContext ctx = eventTask.getContext();
                    if (ctx.getHandler() instanceof ExecutionHandler) {
                        ctx.setAttachment(Boolean.TRUE);
                    }
                    channel.setReadable(false);
                }
            } else {
                ((MemoryAwareRunnable) task).estimatedSize = increment;
            }
            if (this.totalLimiter != null) {
                this.totalLimiter.increase((long) increment);
            }
        }
    }

    /* access modifiers changed from: protected */
    public void decreaseCounter(Runnable task) {
        int increment;
        if (shouldCount(task)) {
            long maxChannelMemorySize = this.settings.maxChannelMemorySize;
            if (task instanceof ChannelEventRunnable) {
                increment = ((ChannelEventRunnable) task).estimatedSize;
            } else {
                increment = ((MemoryAwareRunnable) task).estimatedSize;
            }
            if (this.totalLimiter != null) {
                this.totalLimiter.decrease((long) increment);
            }
            if (task instanceof ChannelEventRunnable) {
                ChannelEventRunnable eventTask = (ChannelEventRunnable) task;
                Channel channel = eventTask.getEvent().getChannel();
                long channelCounter = getChannelCounter(channel).addAndGet((long) (-increment));
                if (maxChannelMemorySize != 0 && channelCounter < maxChannelMemorySize && channel.isOpen() && !channel.isReadable()) {
                    ChannelHandlerContext ctx = eventTask.getContext();
                    if (!(ctx.getHandler() instanceof ExecutionHandler)) {
                        channel.setReadable(true);
                    } else if (ctx.getAttachment() != null) {
                        ctx.setAttachment((Object) null);
                        channel.setReadable(true);
                    }
                }
            }
        }
    }

    /* JADX WARNING: Code restructure failed: missing block: B:2:0x000a, code lost:
        r0 = new java.util.concurrent.atomic.AtomicLong();
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    private java.util.concurrent.atomic.AtomicLong getChannelCounter(org.jboss.netty.channel.Channel r3) {
        /*
            r2 = this;
            java.util.concurrent.ConcurrentMap<org.jboss.netty.channel.Channel, java.util.concurrent.atomic.AtomicLong> r0 = r2.channelCounters
            java.lang.Object r0 = r0.get(r3)
            java.util.concurrent.atomic.AtomicLong r0 = (java.util.concurrent.atomic.AtomicLong) r0
            if (r0 != 0) goto L_0x001b
            java.util.concurrent.atomic.AtomicLong r1 = new java.util.concurrent.atomic.AtomicLong
            r1.<init>()
            r0 = r1
            java.util.concurrent.ConcurrentMap<org.jboss.netty.channel.Channel, java.util.concurrent.atomic.AtomicLong> r1 = r2.channelCounters
            java.lang.Object r1 = r1.putIfAbsent(r3, r0)
            java.util.concurrent.atomic.AtomicLong r1 = (java.util.concurrent.atomic.AtomicLong) r1
            if (r1 == 0) goto L_0x001b
            r0 = r1
        L_0x001b:
            boolean r1 = r3.isOpen()
            if (r1 != 0) goto L_0x0026
            java.util.concurrent.ConcurrentMap<org.jboss.netty.channel.Channel, java.util.concurrent.atomic.AtomicLong> r1 = r2.channelCounters
            r1.remove(r3)
        L_0x0026:
            return r0
        */
        throw new UnsupportedOperationException("Method not decompiled: org.jboss.netty.handler.execution.MemoryAwareThreadPoolExecutor.getChannelCounter(org.jboss.netty.channel.Channel):java.util.concurrent.atomic.AtomicLong");
    }

    /* access modifiers changed from: protected */
    public boolean shouldCount(Runnable task) {
        if (!(task instanceof ChannelUpstreamEventRunnable)) {
            return true;
        }
        ChannelEvent e = ((ChannelUpstreamEventRunnable) task).getEvent();
        if (e instanceof WriteCompletionEvent) {
            return false;
        }
        if (!(e instanceof ChannelStateEvent) || ((ChannelStateEvent) e).getState() != ChannelState.INTEREST_OPS) {
            return true;
        }
        return false;
    }

    private static final class Settings {
        final long maxChannelMemorySize;
        final ObjectSizeEstimator objectSizeEstimator;

        Settings(ObjectSizeEstimator objectSizeEstimator2, long maxChannelMemorySize2) {
            this.objectSizeEstimator = objectSizeEstimator2;
            this.maxChannelMemorySize = maxChannelMemorySize2;
        }
    }

    private static final class NewThreadRunsPolicy implements RejectedExecutionHandler {
        private NewThreadRunsPolicy() {
        }

        public void rejectedExecution(Runnable r, ThreadPoolExecutor executor) {
            try {
                new Thread(r, "Temporary task executor").start();
            } catch (Throwable e) {
                throw new RejectedExecutionException("Failed to start a new thread", e);
            }
        }
    }

    private static final class MemoryAwareRunnable implements Runnable {
        int estimatedSize;
        final Runnable task;

        MemoryAwareRunnable(Runnable task2) {
            this.task = task2;
        }

        public void run() {
            this.task.run();
        }
    }

    private static class Limiter {
        private long counter;
        final long limit;
        private int waiters;

        Limiter(long limit2) {
            this.limit = limit2;
        }

        /* access modifiers changed from: package-private */
        public synchronized void increase(long amount) {
            int i;
            long amount2 = amount;
            while (this.counter >= this.limit) {
                this.waiters++;
                try {
                    wait();
                    i = this.waiters;
                } catch (InterruptedException e) {
                    try {
                        Thread.currentThread().interrupt();
                        i = this.waiters;
                    } catch (Throwable th) {
                        this.waiters--;
                        throw th;
                    }
                }
                this.waiters = i - 1;
            }
            this.counter += amount2;
        }

        /* access modifiers changed from: package-private */
        public synchronized void decrease(long amount) {
            this.counter -= amount;
            if (this.counter < this.limit && this.waiters > 0) {
                notifyAll();
            }
        }
    }
}
