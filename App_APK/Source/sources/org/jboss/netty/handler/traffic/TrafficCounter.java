package org.jboss.netty.handler.traffic;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;
import org.jboss.netty.channel.ChannelHandlerContext;
import org.jboss.netty.util.Timeout;
import org.jboss.netty.util.Timer;
import org.jboss.netty.util.TimerTask;

public class TrafficCounter {
    AtomicLong checkInterval = new AtomicLong(1000);
    private final AtomicLong cumulativeReadBytes = new AtomicLong();
    private final AtomicLong cumulativeWrittenBytes = new AtomicLong();
    private final AtomicLong currentReadBytes = new AtomicLong();
    private final AtomicLong currentWrittenBytes = new AtomicLong();
    private long lastCumulativeTime;
    private long lastReadBytes;
    private long lastReadThroughput;
    private final AtomicLong lastTime = new AtomicLong();
    private long lastWriteThroughput;
    private long lastWrittenBytes;
    AtomicBoolean monitorActive = new AtomicBoolean();
    final String name;
    private volatile Timeout timeout;
    /* access modifiers changed from: private */
    public final Timer timer;
    private TimerTask timerTask;
    private final AbstractTrafficShapingHandler trafficShapingHandler;

    private static class TrafficMonitoringTask implements TimerTask {
        private final TrafficCounter counter;
        private final AbstractTrafficShapingHandler trafficShapingHandler1;

        protected TrafficMonitoringTask(AbstractTrafficShapingHandler trafficShapingHandler, TrafficCounter counter2) {
            this.trafficShapingHandler1 = trafficShapingHandler;
            this.counter = counter2;
        }

        public void run(Timeout timeout) throws Exception {
            if (this.counter.monitorActive.get()) {
                this.counter.resetAccounting(System.currentTimeMillis());
                if (this.trafficShapingHandler1 != null) {
                    this.trafficShapingHandler1.doAccounting(this.counter);
                }
                Timeout timeout2 = this.counter.timer.newTimeout(this, this.counter.checkInterval.get(), TimeUnit.MILLISECONDS);
            }
        }
    }

    /* JADX WARNING: Code restructure failed: missing block: B:11:0x0044, code lost:
        return;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public void start() {
        /*
            r6 = this;
            java.util.concurrent.atomic.AtomicLong r0 = r6.lastTime
            monitor-enter(r0)
            java.util.concurrent.atomic.AtomicBoolean r1 = r6.monitorActive     // Catch:{ all -> 0x0045 }
            boolean r1 = r1.get()     // Catch:{ all -> 0x0045 }
            if (r1 == 0) goto L_0x000d
            monitor-exit(r0)     // Catch:{ all -> 0x0045 }
            return
        L_0x000d:
            java.util.concurrent.atomic.AtomicLong r1 = r6.lastTime     // Catch:{ all -> 0x0045 }
            long r2 = java.lang.System.currentTimeMillis()     // Catch:{ all -> 0x0045 }
            r1.set(r2)     // Catch:{ all -> 0x0045 }
            java.util.concurrent.atomic.AtomicLong r1 = r6.checkInterval     // Catch:{ all -> 0x0045 }
            long r1 = r1.get()     // Catch:{ all -> 0x0045 }
            r3 = 0
            int r5 = (r1 > r3 ? 1 : (r1 == r3 ? 0 : -1))
            if (r5 <= 0) goto L_0x0043
            java.util.concurrent.atomic.AtomicBoolean r1 = r6.monitorActive     // Catch:{ all -> 0x0045 }
            r2 = 1
            r1.set(r2)     // Catch:{ all -> 0x0045 }
            org.jboss.netty.handler.traffic.TrafficCounter$TrafficMonitoringTask r1 = new org.jboss.netty.handler.traffic.TrafficCounter$TrafficMonitoringTask     // Catch:{ all -> 0x0045 }
            org.jboss.netty.handler.traffic.AbstractTrafficShapingHandler r2 = r6.trafficShapingHandler     // Catch:{ all -> 0x0045 }
            r1.<init>(r2, r6)     // Catch:{ all -> 0x0045 }
            r6.timerTask = r1     // Catch:{ all -> 0x0045 }
            org.jboss.netty.util.Timer r1 = r6.timer     // Catch:{ all -> 0x0045 }
            org.jboss.netty.util.TimerTask r2 = r6.timerTask     // Catch:{ all -> 0x0045 }
            java.util.concurrent.atomic.AtomicLong r3 = r6.checkInterval     // Catch:{ all -> 0x0045 }
            long r3 = r3.get()     // Catch:{ all -> 0x0045 }
            java.util.concurrent.TimeUnit r5 = java.util.concurrent.TimeUnit.MILLISECONDS     // Catch:{ all -> 0x0045 }
            org.jboss.netty.util.Timeout r1 = r1.newTimeout(r2, r3, r5)     // Catch:{ all -> 0x0045 }
            r6.timeout = r1     // Catch:{ all -> 0x0045 }
        L_0x0043:
            monitor-exit(r0)     // Catch:{ all -> 0x0045 }
            return
        L_0x0045:
            r1 = move-exception
            monitor-exit(r0)     // Catch:{ all -> 0x0045 }
            throw r1
        */
        throw new UnsupportedOperationException("Method not decompiled: org.jboss.netty.handler.traffic.TrafficCounter.start():void");
    }

    /* JADX WARNING: Code restructure failed: missing block: B:14:0x002d, code lost:
        return;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public void stop() {
        /*
            r3 = this;
            java.util.concurrent.atomic.AtomicLong r0 = r3.lastTime
            monitor-enter(r0)
            java.util.concurrent.atomic.AtomicBoolean r1 = r3.monitorActive     // Catch:{ all -> 0x002e }
            boolean r1 = r1.get()     // Catch:{ all -> 0x002e }
            if (r1 != 0) goto L_0x000d
            monitor-exit(r0)     // Catch:{ all -> 0x002e }
            return
        L_0x000d:
            java.util.concurrent.atomic.AtomicBoolean r1 = r3.monitorActive     // Catch:{ all -> 0x002e }
            r2 = 0
            r1.set(r2)     // Catch:{ all -> 0x002e }
            long r1 = java.lang.System.currentTimeMillis()     // Catch:{ all -> 0x002e }
            r3.resetAccounting(r1)     // Catch:{ all -> 0x002e }
            org.jboss.netty.handler.traffic.AbstractTrafficShapingHandler r1 = r3.trafficShapingHandler     // Catch:{ all -> 0x002e }
            if (r1 == 0) goto L_0x0023
            org.jboss.netty.handler.traffic.AbstractTrafficShapingHandler r1 = r3.trafficShapingHandler     // Catch:{ all -> 0x002e }
            r1.doAccounting(r3)     // Catch:{ all -> 0x002e }
        L_0x0023:
            org.jboss.netty.util.Timeout r1 = r3.timeout     // Catch:{ all -> 0x002e }
            if (r1 == 0) goto L_0x002c
            org.jboss.netty.util.Timeout r1 = r3.timeout     // Catch:{ all -> 0x002e }
            r1.cancel()     // Catch:{ all -> 0x002e }
        L_0x002c:
            monitor-exit(r0)     // Catch:{ all -> 0x002e }
            return
        L_0x002e:
            r1 = move-exception
            monitor-exit(r0)     // Catch:{ all -> 0x002e }
            throw r1
        */
        throw new UnsupportedOperationException("Method not decompiled: org.jboss.netty.handler.traffic.TrafficCounter.stop():void");
    }

    /* access modifiers changed from: package-private */
    public void resetAccounting(long newLastTime) {
        synchronized (this.lastTime) {
            long interval = newLastTime - this.lastTime.getAndSet(newLastTime);
            if (interval != 0) {
                this.lastReadBytes = this.currentReadBytes.getAndSet(0);
                this.lastWrittenBytes = this.currentWrittenBytes.getAndSet(0);
                this.lastReadThroughput = (this.lastReadBytes / interval) * 1000;
                this.lastWriteThroughput = (this.lastWrittenBytes / interval) * 1000;
            }
        }
    }

    public TrafficCounter(AbstractTrafficShapingHandler trafficShapingHandler2, Timer timer2, String name2, long checkInterval2) {
        this.trafficShapingHandler = trafficShapingHandler2;
        this.timer = timer2;
        this.name = name2;
        this.lastCumulativeTime = System.currentTimeMillis();
        configure(checkInterval2);
    }

    public void configure(long newcheckInterval) {
        long newInterval = (newcheckInterval / 10) * 10;
        if (this.checkInterval.get() != newInterval) {
            this.checkInterval.set(newInterval);
            if (newInterval <= 0) {
                stop();
                this.lastTime.set(System.currentTimeMillis());
                return;
            }
            start();
        }
    }

    /* access modifiers changed from: package-private */
    public void bytesRecvFlowControl(ChannelHandlerContext ctx, long recv) {
        this.currentReadBytes.addAndGet(recv);
        this.cumulativeReadBytes.addAndGet(recv);
    }

    /* access modifiers changed from: package-private */
    public void bytesWriteFlowControl(long write) {
        this.currentWrittenBytes.addAndGet(write);
        this.cumulativeWrittenBytes.addAndGet(write);
    }

    public long getCheckInterval() {
        return this.checkInterval.get();
    }

    public long getLastReadThroughput() {
        return this.lastReadThroughput;
    }

    public long getLastWriteThroughput() {
        return this.lastWriteThroughput;
    }

    public long getLastReadBytes() {
        return this.lastReadBytes;
    }

    public long getLastWrittenBytes() {
        return this.lastWrittenBytes;
    }

    public long getCurrentReadBytes() {
        return this.currentReadBytes.get();
    }

    public long getCurrentWrittenBytes() {
        return this.currentWrittenBytes.get();
    }

    public long getLastTime() {
        return this.lastTime.get();
    }

    public long getCumulativeWrittenBytes() {
        return this.cumulativeWrittenBytes.get();
    }

    public long getCumulativeReadBytes() {
        return this.cumulativeReadBytes.get();
    }

    public long getLastCumulativeTime() {
        return this.lastCumulativeTime;
    }

    public void resetCumulativeTime() {
        this.lastCumulativeTime = System.currentTimeMillis();
        this.cumulativeReadBytes.set(0);
        this.cumulativeWrittenBytes.set(0);
    }

    public String getName() {
        return this.name;
    }

    public String toString() {
        return "Monitor " + this.name + " Current Speed Read: " + (this.lastReadThroughput >> 10) + " KB/s, Write: " + (this.lastWriteThroughput >> 10) + " KB/s Current Read: " + (this.currentReadBytes.get() >> 10) + " KB Current Write: " + (this.currentWrittenBytes.get() >> 10) + " KB";
    }
}
