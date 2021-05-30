package com.google.common.util.concurrent;

import com.google.common.base.Preconditions;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorCompletionService;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

abstract class AbstractListeningExecutorService implements ListeningExecutorService {
    AbstractListeningExecutorService() {
    }

    public ListenableFuture<?> submit(Runnable task) {
        ListenableFutureTask<Void> ftask = ListenableFutureTask.create(task, null);
        execute(ftask);
        return ftask;
    }

    public <T> ListenableFuture<T> submit(Runnable task, T result) {
        ListenableFutureTask<T> ftask = ListenableFutureTask.create(task, result);
        execute(ftask);
        return ftask;
    }

    public <T> ListenableFuture<T> submit(Callable<T> task) {
        ListenableFutureTask<T> ftask = ListenableFutureTask.create(task);
        execute(ftask);
        return ftask;
    }

    private <T> T doInvokeAny(Collection<? extends Callable<T>> tasks, boolean timed, long nanos) throws InterruptedException, ExecutionException, TimeoutException {
        long lastTime;
        int i;
        ExecutionException executionException;
        int ntasks = tasks.size();
        Preconditions.checkArgument(ntasks > 0);
        List<Future<T>> futures = new ArrayList<>(ntasks);
        ExecutorCompletionService<T> ecs = new ExecutorCompletionService<>(this);
        if (timed) {
            try {
                lastTime = System.nanoTime();
            } catch (Throwable th) {
                ee = th;
                long j = nanos;
                for (Future<T> f : futures) {
                    f.cancel(true);
                }
                throw ee;
            }
        } else {
            lastTime = 0;
        }
        Iterator<? extends Callable<T>> it = tasks.iterator();
        futures.add(ecs.submit((Callable) it.next()));
        int ntasks2 = ntasks - 1;
        long nanos2 = nanos;
        long lastTime2 = lastTime;
        ExecutionException ee = null;
        int active = 1;
        while (true) {
            try {
                Future<T> f2 = ecs.poll();
                if (f2 == null) {
                    if (ntasks2 > 0) {
                        ntasks2--;
                        futures.add(ecs.submit((Callable) it.next()));
                        active++;
                    } else if (active == 0) {
                        if (ee == null) {
                            ee = new ExecutionException((Throwable) null);
                        }
                        throw ee;
                    } else if (timed) {
                        f2 = ecs.poll(nanos2, TimeUnit.NANOSECONDS);
                        if (f2 != null) {
                            long now = System.nanoTime();
                            nanos2 -= now - lastTime2;
                            lastTime2 = now;
                        } else {
                            throw new TimeoutException();
                        }
                    } else {
                        f2 = ecs.take();
                    }
                }
                if (f2 != null) {
                    i = active - 1;
                    T t = f2.get();
                    for (Future<T> f3 : futures) {
                        f3.cancel(true);
                    }
                    return t;
                }
            } catch (ExecutionException e) {
                executionException = e;
            } catch (RuntimeException e2) {
                executionException = new ExecutionException(e2);
            } catch (Throwable th2) {
                ee = th2;
            }
        }
        ee = executionException;
        active = i;
    }

    public <T> T invokeAny(Collection<? extends Callable<T>> tasks) throws InterruptedException, ExecutionException {
        try {
            return doInvokeAny(tasks, false, 0);
        } catch (TimeoutException e) {
            throw new AssertionError();
        }
    }

    public <T> T invokeAny(Collection<? extends Callable<T>> tasks, long timeout, TimeUnit unit) throws InterruptedException, ExecutionException, TimeoutException {
        return doInvokeAny(tasks, true, unit.toNanos(timeout));
    }

    /*  JADX ERROR: StackOverflow in pass: MarkFinallyVisitor
        jadx.core.utils.exceptions.JadxOverflowException: 
        	at jadx.core.utils.ErrorsCounter.addError(ErrorsCounter.java:47)
        	at jadx.core.utils.ErrorsCounter.methodError(ErrorsCounter.java:81)
        */
    public <T> java.util.List<java.util.concurrent.Future<T>> invokeAll(java.util.Collection<? extends java.util.concurrent.Callable<T>> r7) throws java.lang.InterruptedException {
        /*
            r6 = this;
            if (r7 == 0) goto L_0x0078
            java.util.ArrayList r0 = new java.util.ArrayList
            int r1 = r7.size()
            r0.<init>(r1)
            r1 = 0
            r2 = 1
            java.util.Iterator r3 = r7.iterator()     // Catch:{ all -> 0x0060 }
        L_0x0011:
            boolean r4 = r3.hasNext()     // Catch:{ all -> 0x0060 }
            if (r4 == 0) goto L_0x0028
            java.lang.Object r4 = r3.next()     // Catch:{ all -> 0x0060 }
            java.util.concurrent.Callable r4 = (java.util.concurrent.Callable) r4     // Catch:{ all -> 0x0060 }
            com.google.common.util.concurrent.ListenableFutureTask r5 = com.google.common.util.concurrent.ListenableFutureTask.create(r4)     // Catch:{ all -> 0x0060 }
            r0.add(r5)     // Catch:{ all -> 0x0060 }
            r6.execute(r5)     // Catch:{ all -> 0x0060 }
            goto L_0x0011
        L_0x0028:
            java.util.Iterator r3 = r0.iterator()     // Catch:{ all -> 0x0060 }
        L_0x002c:
            boolean r4 = r3.hasNext()     // Catch:{ all -> 0x0060 }
            if (r4 == 0) goto L_0x0047
            java.lang.Object r4 = r3.next()     // Catch:{ all -> 0x0060 }
            java.util.concurrent.Future r4 = (java.util.concurrent.Future) r4     // Catch:{ all -> 0x0060 }
            boolean r5 = r4.isDone()     // Catch:{ all -> 0x0060 }
            if (r5 != 0) goto L_0x0046
            r4.get()     // Catch:{ CancellationException -> 0x0044, ExecutionException -> 0x0042 }
        L_0x0041:
            goto L_0x0046
        L_0x0042:
            r5 = move-exception
            goto L_0x0046
        L_0x0044:
            r5 = move-exception
            goto L_0x0041
        L_0x0046:
            goto L_0x002c
        L_0x0047:
            r1 = 1
            if (r1 != 0) goto L_0x005f
            java.util.Iterator r3 = r0.iterator()
        L_0x004f:
            boolean r4 = r3.hasNext()
            if (r4 == 0) goto L_0x005f
            java.lang.Object r4 = r3.next()
            java.util.concurrent.Future r4 = (java.util.concurrent.Future) r4
            r4.cancel(r2)
            goto L_0x004f
        L_0x005f:
            return r0
        L_0x0060:
            r3 = move-exception
            if (r1 != 0) goto L_0x0077
            java.util.Iterator r4 = r0.iterator()
        L_0x0067:
            boolean r5 = r4.hasNext()
            if (r5 == 0) goto L_0x0077
            java.lang.Object r5 = r4.next()
            java.util.concurrent.Future r5 = (java.util.concurrent.Future) r5
            r5.cancel(r2)
            goto L_0x0067
        L_0x0077:
            throw r3
        L_0x0078:
            java.lang.NullPointerException r0 = new java.lang.NullPointerException
            r0.<init>()
            throw r0
        */
        throw new UnsupportedOperationException("Method not decompiled: com.google.common.util.concurrent.AbstractListeningExecutorService.invokeAll(java.util.Collection):java.util.List");
    }

    /* JADX WARNING: Code restructure failed: missing block: B:25:0x0078, code lost:
        r14 = r21;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:27:?, code lost:
        r0 = r6.iterator();
     */
    /* JADX WARNING: Code restructure failed: missing block: B:28:0x007e, code lost:
        r15 = r0;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:29:0x0083, code lost:
        if (r15.hasNext() == false) goto L_0x00f8;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:30:0x0085, code lost:
        r19 = r15.next();
     */
    /* JADX WARNING: Code restructure failed: missing block: B:31:0x0091, code lost:
        if (r19.isDone() != false) goto L_0x00f3;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:33:0x0095, code lost:
        if (r4 > r12) goto L_0x00af;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:34:0x0098, code lost:
        if (0 != 0) goto L_0x00ae;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:35:0x009a, code lost:
        r0 = r6.iterator();
     */
    /* JADX WARNING: Code restructure failed: missing block: B:37:0x00a2, code lost:
        if (r0.hasNext() == false) goto L_0x00ae;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:38:0x00a4, code lost:
        r0.next().cancel(true);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:39:0x00ae, code lost:
        return r6;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:44:?, code lost:
        r19.get(r4, java.util.concurrent.TimeUnit.NANOSECONDS);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:45:0x00b7, code lost:
        r0 = e;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:47:0x00bd, code lost:
        r0 = e;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:48:0x00be, code lost:
        r12 = r19;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:49:0x00c1, code lost:
        if (0 == 0) goto L_0x00c3;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:50:0x00c3, code lost:
        r13 = r6.iterator();
     */
    /* JADX WARNING: Code restructure failed: missing block: B:52:0x00cb, code lost:
        if (r13.hasNext() != false) goto L_0x00cd;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:53:0x00cd, code lost:
        r13.next().cancel(true);
        r0 = r0;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:54:0x00dd, code lost:
        r20 = r0;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:55:0x00df, code lost:
        return r6;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:57:0x00e1, code lost:
        r12 = r19;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:59:0x00e5, code lost:
        r12 = r19;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:65:0x00fa, code lost:
        if (1 != 0) goto L_0x0110;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:66:0x00fc, code lost:
        r7 = r6.iterator();
     */
    /* JADX WARNING: Code restructure failed: missing block: B:68:0x0104, code lost:
        if (r7.hasNext() == false) goto L_0x0110;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:69:0x0106, code lost:
        r7.next().cancel(true);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:70:0x0110, code lost:
        return r6;
     */
    /* JADX WARNING: Removed duplicated region for block: B:50:0x00c3  */
    /* JADX WARNING: Removed duplicated region for block: B:75:0x0118  */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public <T> java.util.List<java.util.concurrent.Future<T>> invokeAll(java.util.Collection<? extends java.util.concurrent.Callable<T>> r22, long r23, java.util.concurrent.TimeUnit r25) throws java.lang.InterruptedException {
        /*
            r21 = this;
            r1 = r25
            if (r22 == 0) goto L_0x012d
            if (r1 == 0) goto L_0x012d
            r2 = r23
            long r4 = r1.toNanos(r2)
            java.util.ArrayList r0 = new java.util.ArrayList
            int r6 = r22.size()
            r0.<init>(r6)
            r6 = r0
            r0 = 0
            r7 = r0
            r8 = 1
            java.util.Iterator r0 = r22.iterator()     // Catch:{ all -> 0x0113 }
        L_0x001d:
            boolean r9 = r0.hasNext()     // Catch:{ all -> 0x0113 }
            if (r9 == 0) goto L_0x0031
            java.lang.Object r9 = r0.next()     // Catch:{ all -> 0x0113 }
            java.util.concurrent.Callable r9 = (java.util.concurrent.Callable) r9     // Catch:{ all -> 0x0113 }
            com.google.common.util.concurrent.ListenableFutureTask r10 = com.google.common.util.concurrent.ListenableFutureTask.create(r9)     // Catch:{ all -> 0x0113 }
            r6.add(r10)     // Catch:{ all -> 0x0113 }
            goto L_0x001d
        L_0x0031:
            long r9 = java.lang.System.nanoTime()     // Catch:{ all -> 0x0113 }
            java.util.Iterator r0 = r6.iterator()     // Catch:{ all -> 0x0113 }
        L_0x0039:
            r11 = r0
            boolean r0 = r11.hasNext()     // Catch:{ all -> 0x0113 }
            r12 = 0
            if (r0 == 0) goto L_0x0078
            java.lang.Object r0 = r11.next()     // Catch:{ all -> 0x0113 }
            java.lang.Runnable r0 = (java.lang.Runnable) r0     // Catch:{ all -> 0x0113 }
            java.lang.Runnable r0 = (java.lang.Runnable) r0     // Catch:{ all -> 0x0113 }
            r14 = r21
            r14.execute(r0)     // Catch:{ all -> 0x0111 }
            long r15 = java.lang.System.nanoTime()     // Catch:{ all -> 0x0111 }
            r0 = 0
            long r17 = r15 - r9
            long r4 = r4 - r17
            r9 = r15
            int r0 = (r4 > r12 ? 1 : (r4 == r12 ? 0 : -1))
            if (r0 > 0) goto L_0x0075
            if (r7 != 0) goto L_0x0074
            java.util.Iterator r0 = r6.iterator()
        L_0x0064:
            boolean r12 = r0.hasNext()
            if (r12 == 0) goto L_0x0074
            java.lang.Object r12 = r0.next()
            java.util.concurrent.Future r12 = (java.util.concurrent.Future) r12
            r12.cancel(r8)
            goto L_0x0064
        L_0x0074:
            return r6
        L_0x0075:
            r0 = r11
            goto L_0x0039
        L_0x0078:
            r14 = r21
            java.util.Iterator r0 = r6.iterator()     // Catch:{ all -> 0x0111 }
        L_0x007e:
            r15 = r0
            boolean r0 = r15.hasNext()     // Catch:{ all -> 0x0111 }
            if (r0 == 0) goto L_0x00f8
            java.lang.Object r0 = r15.next()     // Catch:{ all -> 0x0111 }
            java.util.concurrent.Future r0 = (java.util.concurrent.Future) r0     // Catch:{ all -> 0x0111 }
            r19 = r0
            boolean r0 = r19.isDone()     // Catch:{ all -> 0x0111 }
            if (r0 != 0) goto L_0x00f3
            int r0 = (r4 > r12 ? 1 : (r4 == r12 ? 0 : -1))
            if (r0 > 0) goto L_0x00af
            if (r7 != 0) goto L_0x00ae
            java.util.Iterator r0 = r6.iterator()
        L_0x009e:
            boolean r12 = r0.hasNext()
            if (r12 == 0) goto L_0x00ae
            java.lang.Object r12 = r0.next()
            java.util.concurrent.Future r12 = (java.util.concurrent.Future) r12
            r12.cancel(r8)
            goto L_0x009e
        L_0x00ae:
            return r6
        L_0x00af:
            java.util.concurrent.TimeUnit r0 = java.util.concurrent.TimeUnit.NANOSECONDS     // Catch:{ CancellationException -> 0x00e4, ExecutionException -> 0x00e0, TimeoutException -> 0x00bd }
            r12 = r19
            r12.get(r4, r0)     // Catch:{ CancellationException -> 0x00bb, ExecutionException -> 0x00b9, TimeoutException -> 0x00b7 }
            goto L_0x00e7
        L_0x00b7:
            r0 = move-exception
            goto L_0x00c0
        L_0x00b9:
            r0 = move-exception
            goto L_0x00e3
        L_0x00bb:
            r0 = move-exception
            goto L_0x00e7
        L_0x00bd:
            r0 = move-exception
            r12 = r19
        L_0x00c0:
            if (r7 != 0) goto L_0x00dd
            java.util.Iterator r13 = r6.iterator()
        L_0x00c7:
            boolean r16 = r13.hasNext()
            if (r16 == 0) goto L_0x00dd
            java.lang.Object r16 = r13.next()
            r20 = r0
            r0 = r16
            java.util.concurrent.Future r0 = (java.util.concurrent.Future) r0
            r0.cancel(r8)
            r0 = r20
            goto L_0x00c7
        L_0x00dd:
            r20 = r0
            return r6
        L_0x00e0:
            r0 = move-exception
            r12 = r19
        L_0x00e3:
            goto L_0x00e7
        L_0x00e4:
            r0 = move-exception
            r12 = r19
        L_0x00e7:
            long r16 = java.lang.System.nanoTime()     // Catch:{ all -> 0x0111 }
            r0 = 0
            long r18 = r16 - r9
            long r4 = r4 - r18
            r9 = r16
        L_0x00f3:
            r0 = r15
            r12 = 0
            goto L_0x007e
        L_0x00f8:
            r0 = 1
            if (r0 != 0) goto L_0x0110
            java.util.Iterator r7 = r6.iterator()
        L_0x0100:
            boolean r12 = r7.hasNext()
            if (r12 == 0) goto L_0x0110
            java.lang.Object r12 = r7.next()
            java.util.concurrent.Future r12 = (java.util.concurrent.Future) r12
            r12.cancel(r8)
            goto L_0x0100
        L_0x0110:
            return r6
        L_0x0111:
            r0 = move-exception
            goto L_0x0116
        L_0x0113:
            r0 = move-exception
            r14 = r21
        L_0x0116:
            if (r7 != 0) goto L_0x012c
            java.util.Iterator r9 = r6.iterator()
        L_0x011c:
            boolean r10 = r9.hasNext()
            if (r10 == 0) goto L_0x012c
            java.lang.Object r10 = r9.next()
            java.util.concurrent.Future r10 = (java.util.concurrent.Future) r10
            r10.cancel(r8)
            goto L_0x011c
        L_0x012c:
            throw r0
        L_0x012d:
            r14 = r21
            r2 = r23
            java.lang.NullPointerException r0 = new java.lang.NullPointerException
            r0.<init>()
            throw r0
        */
        throw new UnsupportedOperationException("Method not decompiled: com.google.common.util.concurrent.AbstractListeningExecutorService.invokeAll(java.util.Collection, long, java.util.concurrent.TimeUnit):java.util.List");
    }
}
