package org.jboss.netty.util;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.concurrent.Executors;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import org.jboss.netty.logging.InternalLogger;
import org.jboss.netty.logging.InternalLoggerFactory;
import org.jboss.netty.util.internal.ConcurrentIdentityHashMap;
import org.jboss.netty.util.internal.DetectionUtil;
import org.jboss.netty.util.internal.ReusableIterator;
import org.jboss.netty.util.internal.SharedResourceMisuseDetector;

public class HashedWheelTimer implements Timer {
    private static final AtomicInteger id = new AtomicInteger();
    static final InternalLogger logger = InternalLoggerFactory.getInstance((Class<?>) HashedWheelTimer.class);
    private static final SharedResourceMisuseDetector misuseDetector = new SharedResourceMisuseDetector(HashedWheelTimer.class);
    final ReusableIterator<HashedWheelTimeout>[] iterators;
    final ReadWriteLock lock;
    final int mask;
    private final long roundDuration;
    final AtomicBoolean shutdown;
    final long tickDuration;
    final Set<HashedWheelTimeout>[] wheel;
    volatile int wheelCursor;
    private final Worker worker;
    final Thread workerThread;

    public HashedWheelTimer() {
        this(Executors.defaultThreadFactory());
    }

    public HashedWheelTimer(long tickDuration2, TimeUnit unit) {
        this(Executors.defaultThreadFactory(), tickDuration2, unit);
    }

    public HashedWheelTimer(long tickDuration2, TimeUnit unit, int ticksPerWheel) {
        this(Executors.defaultThreadFactory(), tickDuration2, unit, ticksPerWheel);
    }

    public HashedWheelTimer(ThreadFactory threadFactory) {
        this(threadFactory, 100, TimeUnit.MILLISECONDS);
    }

    public HashedWheelTimer(ThreadFactory threadFactory, long tickDuration2, TimeUnit unit) {
        this(threadFactory, tickDuration2, unit, 512);
    }

    public HashedWheelTimer(ThreadFactory threadFactory, long tickDuration2, TimeUnit unit, int ticksPerWheel) {
        this.worker = new Worker();
        this.shutdown = new AtomicBoolean();
        this.lock = new ReentrantReadWriteLock();
        if (threadFactory == null) {
            throw new NullPointerException("threadFactory");
        } else if (unit == null) {
            throw new NullPointerException("unit");
        } else if (tickDuration2 <= 0) {
            throw new IllegalArgumentException("tickDuration must be greater than 0: " + tickDuration2);
        } else if (ticksPerWheel > 0) {
            this.wheel = createWheel(ticksPerWheel);
            this.iterators = createIterators(this.wheel);
            this.mask = this.wheel.length - 1;
            long millis = unit.toMillis(tickDuration2);
            long tickDuration3 = millis;
            this.tickDuration = millis;
            if (tickDuration3 == Long.MAX_VALUE || tickDuration3 >= Long.MAX_VALUE / ((long) this.wheel.length)) {
                throw new IllegalArgumentException("tickDuration is too long: " + tickDuration3 + ' ' + unit);
            }
            this.roundDuration = ((long) this.wheel.length) * tickDuration3;
            Worker worker2 = this.worker;
            this.workerThread = threadFactory.newThread(new ThreadRenamingRunnable(worker2, "Hashed wheel timer #" + id.incrementAndGet()));
            misuseDetector.increase();
        } else {
            throw new IllegalArgumentException("ticksPerWheel must be greater than 0: " + ticksPerWheel);
        }
    }

    private static Set<HashedWheelTimeout>[] createWheel(int ticksPerWheel) {
        if (ticksPerWheel <= 0) {
            throw new IllegalArgumentException("ticksPerWheel must be greater than 0: " + ticksPerWheel);
        } else if (ticksPerWheel <= 1073741824) {
            Set<HashedWheelTimeout>[] wheel2 = new Set[normalizeTicksPerWheel(ticksPerWheel)];
            for (int i = 0; i < wheel2.length; i++) {
                wheel2[i] = new MapBackedSet(new ConcurrentIdentityHashMap(16, 0.95f, 4));
            }
            return wheel2;
        } else {
            throw new IllegalArgumentException("ticksPerWheel may not be greater than 2^30: " + ticksPerWheel);
        }
    }

    private static ReusableIterator<HashedWheelTimeout>[] createIterators(Set<HashedWheelTimeout>[] wheel2) {
        ReusableIterator<HashedWheelTimeout>[] iterators2 = new ReusableIterator[wheel2.length];
        for (int i = 0; i < wheel2.length; i++) {
            iterators2[i] = (ReusableIterator) wheel2[i].iterator();
        }
        return iterators2;
    }

    private static int normalizeTicksPerWheel(int ticksPerWheel) {
        int normalizedTicksPerWheel = 1;
        while (normalizedTicksPerWheel < ticksPerWheel) {
            normalizedTicksPerWheel <<= 1;
        }
        return normalizedTicksPerWheel;
    }

    public synchronized void start() {
        if (this.shutdown.get()) {
            throw new IllegalStateException("cannot be started once stopped");
        } else if (!this.workerThread.isAlive()) {
            this.workerThread.start();
        }
    }

    public synchronized Set<Timeout> stop() {
        if (Thread.currentThread() != this.workerThread) {
            if (!this.shutdown.compareAndSet(false, true)) {
                return Collections.emptySet();
            }
            boolean interrupted = false;
            while (this.workerThread.isAlive()) {
                this.workerThread.interrupt();
                try {
                    this.workerThread.join(100);
                } catch (InterruptedException e) {
                    interrupted = true;
                }
            }
            if (interrupted) {
                Thread.currentThread().interrupt();
            }
            misuseDetector.decrease();
            Set<Timeout> unprocessedTimeouts = new HashSet<>();
            for (Set<HashedWheelTimeout> bucket : this.wheel) {
                unprocessedTimeouts.addAll(bucket);
                bucket.clear();
            }
            return Collections.unmodifiableSet(unprocessedTimeouts);
        }
        throw new IllegalStateException(HashedWheelTimer.class.getSimpleName() + ".stop() cannot be called from " + TimerTask.class.getSimpleName());
    }

    public Timeout newTimeout(TimerTask task, long delay, TimeUnit unit) {
        long currentTime = System.currentTimeMillis();
        if (task == null) {
            throw new NullPointerException("task");
        } else if (unit != null) {
            if (!this.workerThread.isAlive()) {
                start();
            }
            long delay2 = unit.toMillis(delay);
            HashedWheelTimeout timeout = new HashedWheelTimeout(task, currentTime + delay2);
            scheduleTimeout(timeout, delay2);
            return timeout;
        } else {
            throw new NullPointerException("unit");
        }
    }

    /* access modifiers changed from: package-private */
    public void scheduleTimeout(HashedWheelTimeout timeout, long delay) {
        long delay2;
        HashedWheelTimeout hashedWheelTimeout = timeout;
        if (delay < this.tickDuration) {
            delay2 = this.tickDuration;
        } else {
            delay2 = delay;
        }
        long lastTickDelay = delay2 % this.tickDuration;
        int i = 0;
        long relativeIndex = ((delay2 % this.roundDuration) / this.tickDuration) + ((long) (lastTickDelay != 0 ? 1 : 0));
        long j = delay2 / this.roundDuration;
        long lastTickDelay2 = lastTickDelay;
        if (delay2 % this.roundDuration == 0) {
            i = 1;
        }
        long remainingRounds = j - ((long) i);
        this.lock.readLock().lock();
        try {
            int stopIndex = (int) ((((long) this.wheelCursor) + relativeIndex) & ((long) this.mask));
            hashedWheelTimeout.stopIndex = stopIndex;
            hashedWheelTimeout.remainingRounds = remainingRounds;
            this.wheel[stopIndex].add(hashedWheelTimeout);
            this.lock.readLock().unlock();
        } catch (Throwable th) {
            HashedWheelTimeout hashedWheelTimeout2 = timeout;
            long j2 = lastTickDelay2;
            long j3 = relativeIndex;
            long j4 = remainingRounds;
            this.lock.readLock().unlock();
            throw th;
        }
    }

    private final class Worker implements Runnable {
        private long startTime;
        private long tick;

        Worker() {
        }

        public void run() {
            List<HashedWheelTimeout> expiredTimeouts = new ArrayList<>();
            this.startTime = System.currentTimeMillis();
            this.tick = 1;
            while (!HashedWheelTimer.this.shutdown.get()) {
                long deadline = waitForNextTick();
                if (deadline > 0) {
                    fetchExpiredTimeouts(expiredTimeouts, deadline);
                    notifyExpiredTimeouts(expiredTimeouts);
                }
            }
        }

        private void fetchExpiredTimeouts(List<HashedWheelTimeout> expiredTimeouts, long deadline) {
            HashedWheelTimer.this.lock.writeLock().lock();
            try {
                HashedWheelTimer hashedWheelTimer = HashedWheelTimer.this;
                int newWheelCursor = (HashedWheelTimer.this.wheelCursor + 1) & HashedWheelTimer.this.mask;
                hashedWheelTimer.wheelCursor = newWheelCursor;
                fetchExpiredTimeouts(expiredTimeouts, HashedWheelTimer.this.iterators[newWheelCursor], deadline);
                HashedWheelTimer.this.lock.writeLock().unlock();
            } catch (Throwable th) {
                HashedWheelTimer.this.lock.writeLock().unlock();
                throw th;
            }
        }

        private void fetchExpiredTimeouts(List<HashedWheelTimeout> expiredTimeouts, ReusableIterator<HashedWheelTimeout> i, long deadline) {
            List<HashedWheelTimeout> slipped = null;
            i.rewind();
            while (i.hasNext()) {
                HashedWheelTimeout timeout = (HashedWheelTimeout) i.next();
                if (timeout.remainingRounds <= 0) {
                    i.remove();
                    if (timeout.deadline <= deadline) {
                        expiredTimeouts.add(timeout);
                    } else {
                        if (slipped == null) {
                            slipped = new ArrayList<>();
                        }
                        slipped.add(timeout);
                    }
                } else {
                    timeout.remainingRounds--;
                }
            }
            if (slipped != null) {
                for (HashedWheelTimeout timeout2 : slipped) {
                    HashedWheelTimer.this.scheduleTimeout(timeout2, timeout2.deadline - deadline);
                }
            }
        }

        private void notifyExpiredTimeouts(List<HashedWheelTimeout> expiredTimeouts) {
            for (int i = expiredTimeouts.size() - 1; i >= 0; i--) {
                expiredTimeouts.get(i).expire();
            }
            expiredTimeouts.clear();
        }

        private long waitForNextTick() {
            long deadline = this.startTime + (HashedWheelTimer.this.tickDuration * this.tick);
            while (true) {
                long sleepTime = (HashedWheelTimer.this.tickDuration * this.tick) - (System.currentTimeMillis() - this.startTime);
                if (DetectionUtil.isWindows()) {
                    sleepTime = (sleepTime / 10) * 10;
                }
                if (sleepTime <= 0) {
                    this.tick++;
                    return deadline;
                }
                try {
                    Thread.sleep(sleepTime);
                } catch (InterruptedException e) {
                    if (HashedWheelTimer.this.shutdown.get()) {
                        return -1;
                    }
                }
            }
        }
    }

    private final class HashedWheelTimeout implements Timeout {
        private static final int ST_CANCELLED = 1;
        private static final int ST_EXPIRED = 2;
        private static final int ST_INIT = 0;
        final long deadline;
        volatile long remainingRounds;
        private final AtomicInteger state = new AtomicInteger(0);
        volatile int stopIndex;
        private final TimerTask task;

        HashedWheelTimeout(TimerTask task2, long deadline2) {
            this.task = task2;
            this.deadline = deadline2;
        }

        public Timer getTimer() {
            return HashedWheelTimer.this;
        }

        public TimerTask getTask() {
            return this.task;
        }

        public void cancel() {
            if (this.state.compareAndSet(0, 1)) {
                HashedWheelTimer.this.wheel[this.stopIndex].remove(this);
            }
        }

        public boolean isCancelled() {
            return this.state.get() == 1;
        }

        public boolean isExpired() {
            return this.state.get() != 0;
        }

        public void expire() {
            if (this.state.compareAndSet(0, 2)) {
                try {
                    this.task.run(this);
                } catch (Throwable t) {
                    if (HashedWheelTimer.logger.isWarnEnabled()) {
                        InternalLogger internalLogger = HashedWheelTimer.logger;
                        internalLogger.warn("An exception was thrown by " + TimerTask.class.getSimpleName() + ".", t);
                    }
                }
            }
        }

        public String toString() {
            long remaining = this.deadline - System.currentTimeMillis();
            StringBuilder buf = new StringBuilder(192);
            buf.append(getClass().getSimpleName());
            buf.append('(');
            buf.append("deadline: ");
            if (remaining > 0) {
                buf.append(remaining);
                buf.append(" ms later, ");
            } else if (remaining < 0) {
                buf.append(-remaining);
                buf.append(" ms ago, ");
            } else {
                buf.append("now, ");
            }
            if (isCancelled()) {
                buf.append(", cancelled");
            }
            buf.append(')');
            return buf.toString();
        }
    }
}
