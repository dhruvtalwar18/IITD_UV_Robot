package org.ros.concurrent;

import com.google.common.base.Preconditions;

public abstract class CancellableLoop implements Runnable {
    private final Object mutex = new Object();
    private boolean ranOnce = false;
    private Thread thread;

    /* access modifiers changed from: protected */
    public abstract void loop() throws InterruptedException;

    public void run() {
        synchronized (this.mutex) {
            Preconditions.checkState(!this.ranOnce, "CancellableLoops cannot be restarted.");
            this.ranOnce = true;
            this.thread = Thread.currentThread();
        }
        try {
            setup();
            while (!this.thread.isInterrupted()) {
                loop();
            }
        } catch (InterruptedException e) {
            handleInterruptedException(e);
        } catch (Throwable th) {
            this.thread = null;
            throw th;
        }
        this.thread = null;
    }

    /* access modifiers changed from: protected */
    public void setup() {
    }

    /* access modifiers changed from: protected */
    public void handleInterruptedException(InterruptedException e) {
    }

    public void cancel() {
        if (this.thread != null) {
            this.thread.interrupt();
        }
    }

    public boolean isRunning() {
        return this.thread != null && !this.thread.isInterrupted();
    }
}
