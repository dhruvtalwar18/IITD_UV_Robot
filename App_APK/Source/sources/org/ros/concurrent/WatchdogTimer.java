package org.ros.concurrent;

import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

public class WatchdogTimer {
    private final long period;
    /* access modifiers changed from: private */
    public boolean pulsed = false;
    private final Runnable runnable;
    private final ScheduledExecutorService scheduledExecutorService;
    private ScheduledFuture<?> scheduledFuture;
    private final TimeUnit unit;

    public WatchdogTimer(ScheduledExecutorService scheduledExecutorService2, long period2, TimeUnit unit2, final Runnable runnable2) {
        this.scheduledExecutorService = scheduledExecutorService2;
        this.period = period2;
        this.unit = unit2;
        this.runnable = new Runnable() {
            public void run() {
                try {
                    if (!WatchdogTimer.this.pulsed) {
                        runnable2.run();
                    }
                } finally {
                    boolean unused = WatchdogTimer.this.pulsed = false;
                }
            }
        };
    }

    public void start() {
        this.scheduledFuture = this.scheduledExecutorService.scheduleAtFixedRate(this.runnable, this.period, this.period, this.unit);
    }

    public void pulse() {
        this.pulsed = true;
    }

    public void cancel() {
        this.scheduledFuture.cancel(true);
    }
}
