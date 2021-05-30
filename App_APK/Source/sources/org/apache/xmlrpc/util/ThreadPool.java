package org.apache.xmlrpc.util;

import java.util.ArrayList;
import java.util.List;

public class ThreadPool {
    private final int maxSize;
    private int num;
    private final List runningThreads = new ArrayList();
    private final ThreadGroup threadGroup;
    private final List waitingTasks = new ArrayList();
    private final List waitingThreads = new ArrayList();

    public interface InterruptableTask extends Task {
        void shutdown() throws Throwable;
    }

    public interface Task {
        void run() throws Throwable;
    }

    private class Poolable {
        /* access modifiers changed from: private */
        public volatile boolean shuttingDown;
        private Task task;
        private Thread thread;

        Poolable(ThreadGroup pGroup, int pNum) {
            this.thread = new Thread(pGroup, pGroup.getName() + "-" + pNum, ThreadPool.this) {
                public void run() {
                    while (!Poolable.this.shuttingDown) {
                        Task t = Poolable.this.getTask();
                        if (t == null) {
                            try {
                                synchronized (this) {
                                    if (!Poolable.this.shuttingDown && Poolable.this.getTask() == null) {
                                        wait();
                                    }
                                }
                            } catch (InterruptedException e) {
                            }
                        } else {
                            try {
                                t.run();
                                Poolable.this.resetTask();
                                ThreadPool.this.repool(Poolable.this);
                            } catch (Throwable th) {
                                ThreadPool.this.remove(Poolable.this);
                                Poolable.this.shutdown();
                                Poolable.this.resetTask();
                            }
                        }
                    }
                }
            };
            this.thread.start();
        }

        /* access modifiers changed from: package-private */
        public synchronized void shutdown() {
            this.shuttingDown = true;
            Task t = getTask();
            if (t != null && (t instanceof InterruptableTask)) {
                try {
                    ((InterruptableTask) t).shutdown();
                } catch (Throwable th) {
                }
            }
            this.task = null;
            synchronized (this.thread) {
                this.thread.notify();
            }
        }

        /* access modifiers changed from: private */
        public Task getTask() {
            return this.task;
        }

        /* access modifiers changed from: private */
        public void resetTask() {
            this.task = null;
        }

        /* access modifiers changed from: package-private */
        public void start(Task pTask) {
            this.task = pTask;
            synchronized (this.thread) {
                this.thread.notify();
            }
        }
    }

    public ThreadPool(int pMaxSize, String pName) {
        this.maxSize = pMaxSize;
        this.threadGroup = new ThreadGroup(pName);
    }

    /* access modifiers changed from: private */
    public synchronized void remove(Poolable pPoolable) {
        this.runningThreads.remove(pPoolable);
        this.waitingThreads.remove(pPoolable);
    }

    /* access modifiers changed from: package-private */
    public void repool(Poolable pPoolable) {
        boolean discarding = false;
        Task task = null;
        Poolable poolable = null;
        synchronized (this) {
            if (!this.runningThreads.remove(pPoolable)) {
                discarding = true;
            } else if (this.maxSize == 0 || this.runningThreads.size() + this.waitingThreads.size() < this.maxSize) {
                this.waitingThreads.add(pPoolable);
                if (this.waitingTasks.size() > 0) {
                    task = (Task) this.waitingTasks.remove(this.waitingTasks.size() - 1);
                    poolable = getPoolable(task, false);
                }
            } else {
                discarding = true;
            }
            if (discarding) {
                remove(pPoolable);
            }
        }
        if (poolable != null) {
            poolable.start(task);
        }
        if (discarding) {
            pPoolable.shutdown();
        }
    }

    public boolean startTask(Task pTask) {
        Poolable poolable = getPoolable(pTask, false);
        if (poolable == null) {
            return false;
        }
        poolable.start(pTask);
        return true;
    }

    private synchronized Poolable getPoolable(Task pTask, boolean pQueue) {
        Poolable poolable;
        if (this.maxSize == 0 || this.runningThreads.size() < this.maxSize) {
            if (this.waitingThreads.size() > 0) {
                poolable = (Poolable) this.waitingThreads.remove(this.waitingThreads.size() - 1);
            } else {
                ThreadGroup threadGroup2 = this.threadGroup;
                int i = this.num;
                this.num = i + 1;
                poolable = new Poolable(threadGroup2, i);
            }
            this.runningThreads.add(poolable);
            return poolable;
        }
        if (pQueue) {
            this.waitingTasks.add(pTask);
        }
        return null;
    }

    public boolean addTask(Task pTask) {
        Poolable poolable = getPoolable(pTask, true);
        if (poolable == null) {
            return false;
        }
        poolable.start(pTask);
        return true;
    }

    public synchronized void shutdown() {
        while (!this.waitingThreads.isEmpty()) {
            ((Poolable) this.waitingThreads.remove(this.waitingThreads.size() - 1)).shutdown();
        }
        while (!this.runningThreads.isEmpty()) {
            ((Poolable) this.runningThreads.remove(this.runningThreads.size() - 1)).shutdown();
        }
    }

    public int getMaxThreads() {
        return this.maxSize;
    }

    public synchronized int getNumThreads() {
        return this.num;
    }
}
