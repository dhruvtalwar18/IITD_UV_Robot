package org.jboss.netty.util.internal;

import java.util.Collection;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedTransferQueue;
import org.jboss.netty.logging.InternalLogger;
import org.jboss.netty.logging.InternalLoggerFactory;

public final class QueueFactory {
    private static final InternalLogger LOGGER = InternalLoggerFactory.getInstance((Class<?>) QueueFactory.class);
    private static final boolean useUnsafe = DetectionUtil.hasUnsafe();

    private QueueFactory() {
    }

    public static <T> BlockingQueue<T> createQueue(Class<T> cls) {
        if (DetectionUtil.javaVersion() >= 7) {
            return new LinkedTransferQueue();
        }
        try {
            if (useUnsafe) {
                return new LinkedTransferQueue();
            }
        } catch (Throwable t) {
            if (LOGGER.isDebugEnabled()) {
                LOGGER.debug("Unable to instance LinkedTransferQueue, fallback to LegacyLinkedTransferQueue", t);
            }
        }
        return new LegacyLinkedTransferQueue();
    }

    public static <T> BlockingQueue<T> createQueue(Collection<? extends T> collection, Class<T> cls) {
        if (DetectionUtil.javaVersion() >= 7) {
            return new LinkedTransferQueue();
        }
        try {
            if (useUnsafe) {
                return new LinkedTransferQueue(collection);
            }
        } catch (Throwable t) {
            if (LOGGER.isDebugEnabled()) {
                LOGGER.debug("Unable to instance LinkedTransferQueue, fallback to LegacyLinkedTransferQueue", t);
            }
        }
        return new LegacyLinkedTransferQueue(collection);
    }
}
