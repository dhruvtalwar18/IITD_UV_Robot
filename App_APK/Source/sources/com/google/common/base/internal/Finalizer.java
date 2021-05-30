package com.google.common.base.internal;

import java.lang.ref.PhantomReference;
import java.lang.ref.Reference;
import java.lang.ref.ReferenceQueue;
import java.lang.ref.WeakReference;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.logging.Level;
import java.util.logging.Logger;

public class Finalizer implements Runnable {
    private static final String FINALIZABLE_REFERENCE = "com.google.common.base.FinalizableReference";
    private static final Field inheritableThreadLocals = getInheritableThreadLocalsField();
    private static final Logger logger = Logger.getLogger(Finalizer.class.getName());
    private final WeakReference<Class<?>> finalizableReferenceClassReference;
    private final PhantomReference<Object> frqReference;
    private final ReferenceQueue<Object> queue = new ReferenceQueue<>();

    public static ReferenceQueue<Object> startFinalizer(Class<?> finalizableReferenceClass, Object frq) {
        if (finalizableReferenceClass.getName().equals(FINALIZABLE_REFERENCE)) {
            Finalizer finalizer = new Finalizer(finalizableReferenceClass, frq);
            Thread thread = new Thread(finalizer);
            thread.setName(Finalizer.class.getName());
            thread.setDaemon(true);
            try {
                if (inheritableThreadLocals != null) {
                    inheritableThreadLocals.set(thread, (Object) null);
                }
            } catch (Throwable t) {
                logger.log(Level.INFO, "Failed to clear thread local values inherited by reference finalizer thread.", t);
            }
            thread.start();
            return finalizer.queue;
        }
        throw new IllegalArgumentException("Expected com.google.common.base.FinalizableReference.");
    }

    private Finalizer(Class<?> finalizableReferenceClass, Object frq) {
        this.finalizableReferenceClassReference = new WeakReference<>(finalizableReferenceClass);
        this.frqReference = new PhantomReference<>(frq, this.queue);
    }

    public void run() {
        while (true) {
            try {
                cleanUp(this.queue.remove());
            } catch (InterruptedException e) {
            } catch (ShutDown e2) {
                return;
            }
        }
    }

    private void cleanUp(Reference<?> reference) throws ShutDown {
        Reference<? extends Object> poll;
        Method finalizeReferentMethod = getFinalizeReferentMethod();
        do {
            reference.clear();
            if (reference != this.frqReference) {
                try {
                    finalizeReferentMethod.invoke(reference, new Object[0]);
                } catch (Throwable t) {
                    logger.log(Level.SEVERE, "Error cleaning up after reference.", t);
                }
                poll = this.queue.poll();
                reference = poll;
            } else {
                throw new ShutDown();
            }
        } while (poll != null);
    }

    private Method getFinalizeReferentMethod() throws ShutDown {
        Class<?> finalizableReferenceClass = (Class) this.finalizableReferenceClassReference.get();
        if (finalizableReferenceClass != null) {
            try {
                return finalizableReferenceClass.getMethod("finalizeReferent", new Class[0]);
            } catch (NoSuchMethodException e) {
                throw new AssertionError(e);
            }
        } else {
            throw new ShutDown();
        }
    }

    public static Field getInheritableThreadLocalsField() {
        try {
            Field inheritableThreadLocals2 = Thread.class.getDeclaredField("inheritableThreadLocals");
            inheritableThreadLocals2.setAccessible(true);
            return inheritableThreadLocals2;
        } catch (Throwable th) {
            logger.log(Level.INFO, "Couldn't access Thread.inheritableThreadLocals. Reference finalizer threads will inherit thread local values.");
            return null;
        }
    }

    private static class ShutDown extends Exception {
        private ShutDown() {
        }
    }
}
