package org.apache.commons.pool.impl;

import java.security.AccessController;
import java.security.PrivilegedAction;
import java.util.Timer;
import java.util.TimerTask;

class EvictionTimer {
    private static Timer _timer;
    private static int _usageCount;

    private EvictionTimer() {
    }

    /* JADX INFO: finally extract failed */
    static synchronized void schedule(TimerTask task, long delay, long period) {
        synchronized (EvictionTimer.class) {
            if (_timer == null) {
                ClassLoader ccl = (ClassLoader) AccessController.doPrivileged(new PrivilegedGetTccl());
                try {
                    AccessController.doPrivileged(new PrivilegedSetTccl(EvictionTimer.class.getClassLoader()));
                    _timer = new Timer(true);
                    AccessController.doPrivileged(new PrivilegedSetTccl(ccl));
                } catch (Throwable th) {
                    AccessController.doPrivileged(new PrivilegedSetTccl(ccl));
                    throw th;
                }
            }
            _usageCount++;
            _timer.schedule(task, delay, period);
        }
    }

    static synchronized void cancel(TimerTask task) {
        synchronized (EvictionTimer.class) {
            task.cancel();
            _usageCount--;
            if (_usageCount == 0) {
                _timer.cancel();
                _timer = null;
            }
        }
    }

    private static class PrivilegedGetTccl implements PrivilegedAction<ClassLoader> {
        private PrivilegedGetTccl() {
        }

        public ClassLoader run() {
            return Thread.currentThread().getContextClassLoader();
        }
    }

    private static class PrivilegedSetTccl implements PrivilegedAction<ClassLoader> {
        private final ClassLoader cl;

        PrivilegedSetTccl(ClassLoader cl2) {
            this.cl = cl2;
        }

        public ClassLoader run() {
            Thread.currentThread().setContextClassLoader(this.cl);
            return null;
        }
    }
}
