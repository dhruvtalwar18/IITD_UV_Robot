package javax.jmdns.impl;

import java.util.Collection;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentMap;
import java.util.concurrent.Semaphore;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.ReentrantLock;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.jmdns.impl.constants.DNSState;
import javax.jmdns.impl.tasks.DNSTask;

public interface DNSStatefulObject {
    boolean advanceState(DNSTask dNSTask);

    void associateWithTask(DNSTask dNSTask, DNSState dNSState);

    boolean cancelState();

    boolean closeState();

    JmDNSImpl getDns();

    boolean isAnnounced();

    boolean isAnnouncing();

    boolean isAssociatedWithTask(DNSTask dNSTask, DNSState dNSState);

    boolean isCanceled();

    boolean isCanceling();

    boolean isClosed();

    boolean isClosing();

    boolean isProbing();

    boolean recoverState();

    void removeAssociationWithTask(DNSTask dNSTask);

    boolean revertState();

    boolean waitForAnnounced(long j);

    boolean waitForCanceled(long j);

    public static final class DNSStatefulObjectSemaphore {
        private static Logger logger = Logger.getLogger(DNSStatefulObjectSemaphore.class.getName());
        private final String _name;
        private final ConcurrentMap<Thread, Semaphore> _semaphores = new ConcurrentHashMap(50);

        public DNSStatefulObjectSemaphore(String name) {
            this._name = name;
        }

        public void waitForEvent(long timeout) {
            Thread thread = Thread.currentThread();
            if (((Semaphore) this._semaphores.get(thread)) == null) {
                Semaphore semaphore = new Semaphore(1, true);
                semaphore.drainPermits();
                this._semaphores.putIfAbsent(thread, semaphore);
            }
            try {
                ((Semaphore) this._semaphores.get(thread)).tryAcquire(timeout, TimeUnit.MILLISECONDS);
            } catch (InterruptedException exception) {
                logger.log(Level.FINER, "Exception ", exception);
            }
        }

        public void signalEvent() {
            Collection<Semaphore> semaphores = this._semaphores.values();
            for (Semaphore semaphore : semaphores) {
                semaphore.release();
                semaphores.remove(semaphore);
            }
        }

        public String toString() {
            StringBuilder aLog = new StringBuilder(1000);
            aLog.append("Semaphore: ");
            aLog.append(this._name);
            if (this._semaphores.size() == 0) {
                aLog.append(" no semaphores.");
            } else {
                aLog.append(" semaphores:\n");
                for (Thread thread : this._semaphores.keySet()) {
                    aLog.append("\tThread: ");
                    aLog.append(thread.getName());
                    aLog.append(' ');
                    aLog.append(this._semaphores.get(thread));
                    aLog.append(10);
                }
            }
            return aLog.toString();
        }
    }

    public static class DefaultImplementation extends ReentrantLock implements DNSStatefulObject {
        private static Logger logger = Logger.getLogger(DefaultImplementation.class.getName());
        private static final long serialVersionUID = -3264781576883412227L;
        private final DNSStatefulObjectSemaphore _announcing = new DNSStatefulObjectSemaphore("Announce");
        private final DNSStatefulObjectSemaphore _canceling = new DNSStatefulObjectSemaphore("Cancel");
        private volatile JmDNSImpl _dns = null;
        protected volatile DNSState _state = DNSState.PROBING_1;
        protected volatile DNSTask _task = null;

        public JmDNSImpl getDns() {
            return this._dns;
        }

        /* access modifiers changed from: protected */
        public void setDns(JmDNSImpl dns) {
            this._dns = dns;
        }

        public void associateWithTask(DNSTask task, DNSState state) {
            if (this._task == null && this._state == state) {
                lock();
                try {
                    if (this._task == null && this._state == state) {
                        setTask(task);
                    }
                } finally {
                    unlock();
                }
            }
        }

        public void removeAssociationWithTask(DNSTask task) {
            if (this._task == task) {
                lock();
                try {
                    if (this._task == task) {
                        setTask((DNSTask) null);
                    }
                } finally {
                    unlock();
                }
            }
        }

        public boolean isAssociatedWithTask(DNSTask task, DNSState state) {
            lock();
            try {
                return this._task == task && this._state == state;
            } finally {
                unlock();
            }
        }

        /* access modifiers changed from: protected */
        public void setTask(DNSTask task) {
            this._task = task;
        }

        /* access modifiers changed from: protected */
        public void setState(DNSState state) {
            lock();
            try {
                this._state = state;
                if (isAnnounced()) {
                    this._announcing.signalEvent();
                }
                if (isCanceled()) {
                    this._canceling.signalEvent();
                    this._announcing.signalEvent();
                }
            } finally {
                unlock();
            }
        }

        public boolean advanceState(DNSTask task) {
            if (this._task == task) {
                lock();
                try {
                    if (this._task == task) {
                        setState(this._state.advance());
                    } else {
                        Logger logger2 = logger;
                        logger2.warning("Trying to advance state whhen not the owner. owner: " + this._task + " perpetrator: " + task);
                    }
                } finally {
                    unlock();
                }
            }
            return true;
        }

        public boolean revertState() {
            if (!willCancel()) {
                lock();
                try {
                    if (!willCancel()) {
                        setState(this._state.revert());
                        setTask((DNSTask) null);
                    }
                } finally {
                    unlock();
                }
            }
            return true;
        }

        public boolean cancelState() {
            boolean result = false;
            if (!willCancel()) {
                lock();
                try {
                    if (!willCancel()) {
                        setState(DNSState.CANCELING_1);
                        setTask((DNSTask) null);
                        result = true;
                    }
                } finally {
                    unlock();
                }
            }
            return result;
        }

        public boolean closeState() {
            boolean result = false;
            if (!willClose()) {
                lock();
                try {
                    if (!willClose()) {
                        setState(DNSState.CLOSING);
                        setTask((DNSTask) null);
                        result = true;
                    }
                } finally {
                    unlock();
                }
            }
            return result;
        }

        public boolean recoverState() {
            lock();
            try {
                setState(DNSState.PROBING_1);
                setTask((DNSTask) null);
                return false;
            } finally {
                unlock();
            }
        }

        public boolean isProbing() {
            return this._state.isProbing();
        }

        public boolean isAnnouncing() {
            return this._state.isAnnouncing();
        }

        public boolean isAnnounced() {
            return this._state.isAnnounced();
        }

        public boolean isCanceling() {
            return this._state.isCanceling();
        }

        public boolean isCanceled() {
            return this._state.isCanceled();
        }

        public boolean isClosing() {
            return this._state.isClosing();
        }

        public boolean isClosed() {
            return this._state.isClosed();
        }

        private boolean willCancel() {
            return this._state.isCanceled() || this._state.isCanceling();
        }

        private boolean willClose() {
            return this._state.isClosed() || this._state.isClosing();
        }

        public boolean waitForAnnounced(long timeout) {
            if (!isAnnounced() && !willCancel()) {
                this._announcing.waitForEvent(timeout);
            }
            if (!isAnnounced()) {
                if (willCancel() || willClose()) {
                    Logger logger2 = logger;
                    logger2.fine("Wait for announced cancelled: " + this);
                } else {
                    Logger logger3 = logger;
                    logger3.warning("Wait for announced timed out: " + this);
                }
            }
            return isAnnounced();
        }

        public boolean waitForCanceled(long timeout) {
            if (!isCanceled()) {
                this._canceling.waitForEvent(timeout);
            }
            if (!isCanceled() && !willClose()) {
                Logger logger2 = logger;
                logger2.warning("Wait for canceled timed out: " + this);
            }
            return isCanceled();
        }

        public String toString() {
            String str;
            StringBuilder sb = new StringBuilder();
            if (this._dns != null) {
                str = "DNS: " + this._dns.getName();
            } else {
                str = "NO DNS";
            }
            sb.append(str);
            sb.append(" state: ");
            sb.append(this._state);
            sb.append(" task: ");
            sb.append(this._task);
            return sb.toString();
        }
    }
}
