package javax.jmdns.impl;

import java.util.Date;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentMap;
import java.util.concurrent.atomic.AtomicReference;
import javax.jmdns.impl.tasks.RecordReaper;
import javax.jmdns.impl.tasks.Responder;
import javax.jmdns.impl.tasks.resolver.ServiceInfoResolver;
import javax.jmdns.impl.tasks.resolver.ServiceResolver;
import javax.jmdns.impl.tasks.resolver.TypeResolver;
import javax.jmdns.impl.tasks.state.Announcer;
import javax.jmdns.impl.tasks.state.Canceler;
import javax.jmdns.impl.tasks.state.Prober;
import javax.jmdns.impl.tasks.state.Renewer;

public interface DNSTaskStarter {
    void cancelStateTimer();

    void cancelTimer();

    void purgeStateTimer();

    void purgeTimer();

    void startAnnouncer();

    void startCanceler();

    void startProber();

    void startReaper();

    void startRenewer();

    void startResponder(DNSIncoming dNSIncoming, int i);

    void startServiceInfoResolver(ServiceInfoImpl serviceInfoImpl);

    void startServiceResolver(String str);

    void startTypeResolver();

    public static final class Factory {
        private static final AtomicReference<ClassDelegate> _databaseClassDelegate = new AtomicReference<>();
        private static volatile Factory _instance;
        private final ConcurrentMap<JmDNSImpl, DNSTaskStarter> _instances = new ConcurrentHashMap(20);

        public interface ClassDelegate {
            DNSTaskStarter newDNSTaskStarter(JmDNSImpl jmDNSImpl);
        }

        private Factory() {
        }

        public static void setClassDelegate(ClassDelegate delegate) {
            _databaseClassDelegate.set(delegate);
        }

        public static ClassDelegate classDelegate() {
            return _databaseClassDelegate.get();
        }

        protected static DNSTaskStarter newDNSTaskStarter(JmDNSImpl jmDNSImpl) {
            DNSTaskStarter instance = null;
            ClassDelegate delegate = _databaseClassDelegate.get();
            if (delegate != null) {
                instance = delegate.newDNSTaskStarter(jmDNSImpl);
            }
            return instance != null ? instance : new DNSTaskStarterImpl(jmDNSImpl);
        }

        public static Factory getInstance() {
            if (_instance == null) {
                synchronized (Factory.class) {
                    if (_instance == null) {
                        _instance = new Factory();
                    }
                }
            }
            return _instance;
        }

        public DNSTaskStarter getStarter(JmDNSImpl jmDNSImpl) {
            DNSTaskStarter starter = (DNSTaskStarter) this._instances.get(jmDNSImpl);
            if (starter != null) {
                return starter;
            }
            this._instances.putIfAbsent(jmDNSImpl, newDNSTaskStarter(jmDNSImpl));
            return (DNSTaskStarter) this._instances.get(jmDNSImpl);
        }
    }

    public static final class DNSTaskStarterImpl implements DNSTaskStarter {
        private final JmDNSImpl _jmDNSImpl;
        private final Timer _stateTimer = new StarterTimer("JmDNS(" + this._jmDNSImpl.getName() + ").State.Timer", false);
        private final Timer _timer = new StarterTimer("JmDNS(" + this._jmDNSImpl.getName() + ").Timer", true);

        public static class StarterTimer extends Timer {
            private volatile boolean _cancelled = false;

            public StarterTimer() {
            }

            public StarterTimer(boolean isDaemon) {
                super(isDaemon);
            }

            public StarterTimer(String name, boolean isDaemon) {
                super(name, isDaemon);
            }

            public StarterTimer(String name) {
                super(name);
            }

            public synchronized void cancel() {
                if (!this._cancelled) {
                    this._cancelled = true;
                    super.cancel();
                }
            }

            public synchronized void schedule(TimerTask task, long delay) {
                if (!this._cancelled) {
                    super.schedule(task, delay);
                }
            }

            public synchronized void schedule(TimerTask task, Date time) {
                if (!this._cancelled) {
                    super.schedule(task, time);
                }
            }

            public synchronized void schedule(TimerTask task, long delay, long period) {
                if (!this._cancelled) {
                    super.schedule(task, delay, period);
                }
            }

            public synchronized void schedule(TimerTask task, Date firstTime, long period) {
                if (!this._cancelled) {
                    super.schedule(task, firstTime, period);
                }
            }

            public synchronized void scheduleAtFixedRate(TimerTask task, long delay, long period) {
                if (!this._cancelled) {
                    super.scheduleAtFixedRate(task, delay, period);
                }
            }

            public synchronized void scheduleAtFixedRate(TimerTask task, Date firstTime, long period) {
                if (!this._cancelled) {
                    super.scheduleAtFixedRate(task, firstTime, period);
                }
            }
        }

        public DNSTaskStarterImpl(JmDNSImpl jmDNSImpl) {
            this._jmDNSImpl = jmDNSImpl;
        }

        public void purgeTimer() {
            this._timer.purge();
        }

        public void purgeStateTimer() {
            this._stateTimer.purge();
        }

        public void cancelTimer() {
            this._timer.cancel();
        }

        public void cancelStateTimer() {
            this._stateTimer.cancel();
        }

        public void startProber() {
            new Prober(this._jmDNSImpl).start(this._stateTimer);
        }

        public void startAnnouncer() {
            new Announcer(this._jmDNSImpl).start(this._stateTimer);
        }

        public void startRenewer() {
            new Renewer(this._jmDNSImpl).start(this._stateTimer);
        }

        public void startCanceler() {
            new Canceler(this._jmDNSImpl).start(this._stateTimer);
        }

        public void startReaper() {
            new RecordReaper(this._jmDNSImpl).start(this._timer);
        }

        public void startServiceInfoResolver(ServiceInfoImpl info) {
            new ServiceInfoResolver(this._jmDNSImpl, info).start(this._timer);
        }

        public void startTypeResolver() {
            new TypeResolver(this._jmDNSImpl).start(this._timer);
        }

        public void startServiceResolver(String type) {
            new ServiceResolver(this._jmDNSImpl, type).start(this._timer);
        }

        public void startResponder(DNSIncoming in, int port) {
            new Responder(this._jmDNSImpl, in, port).start(this._timer);
        }
    }
}
