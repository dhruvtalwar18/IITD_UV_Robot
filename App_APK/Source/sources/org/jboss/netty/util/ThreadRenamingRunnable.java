package org.jboss.netty.util;

import org.jboss.netty.logging.InternalLogger;
import org.jboss.netty.logging.InternalLoggerFactory;

public class ThreadRenamingRunnable implements Runnable {
    private static final InternalLogger logger = InternalLoggerFactory.getInstance((Class<?>) ThreadRenamingRunnable.class);
    private static volatile ThreadNameDeterminer threadNameDeterminer = ThreadNameDeterminer.PROPOSED;
    private final String proposedThreadName;
    private final Runnable runnable;

    public static ThreadNameDeterminer getThreadNameDeterminer() {
        return threadNameDeterminer;
    }

    public static void setThreadNameDeterminer(ThreadNameDeterminer threadNameDeterminer2) {
        if (threadNameDeterminer2 != null) {
            threadNameDeterminer = threadNameDeterminer2;
            return;
        }
        throw new NullPointerException("threadNameDeterminer");
    }

    public ThreadRenamingRunnable(Runnable runnable2, String proposedThreadName2) {
        if (runnable2 == null) {
            throw new NullPointerException("runnable");
        } else if (proposedThreadName2 != null) {
            this.runnable = runnable2;
            this.proposedThreadName = proposedThreadName2;
        } else {
            throw new NullPointerException("proposedThreadName");
        }
    }

    public void run() {
        Thread currentThread = Thread.currentThread();
        String oldThreadName = currentThread.getName();
        String newThreadName = getNewThreadName(oldThreadName);
        boolean renamed = false;
        if (!oldThreadName.equals(newThreadName)) {
            try {
                currentThread.setName(newThreadName);
                renamed = true;
            } catch (SecurityException e) {
                logger.debug("Failed to rename a thread due to security restriction.", e);
            }
        }
        try {
            this.runnable.run();
        } finally {
            if (renamed) {
                currentThread.setName(oldThreadName);
            }
        }
    }

    private String getNewThreadName(String currentThreadName) {
        String newThreadName = null;
        try {
            newThreadName = getThreadNameDeterminer().determineThreadName(currentThreadName, this.proposedThreadName);
        } catch (Throwable t) {
            logger.warn("Failed to determine the thread name", t);
        }
        return newThreadName == null ? currentThreadName : newThreadName;
    }
}
