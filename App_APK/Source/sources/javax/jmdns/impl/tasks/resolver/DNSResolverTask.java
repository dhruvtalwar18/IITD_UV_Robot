package javax.jmdns.impl.tasks.resolver;

import java.io.IOException;
import java.util.Timer;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.jmdns.impl.DNSOutgoing;
import javax.jmdns.impl.JmDNSImpl;
import javax.jmdns.impl.tasks.DNSTask;

public abstract class DNSResolverTask extends DNSTask {
    private static Logger logger = Logger.getLogger(DNSResolverTask.class.getName());
    protected int _count = 0;

    /* access modifiers changed from: protected */
    public abstract DNSOutgoing addAnswers(DNSOutgoing dNSOutgoing) throws IOException;

    /* access modifiers changed from: protected */
    public abstract DNSOutgoing addQuestions(DNSOutgoing dNSOutgoing) throws IOException;

    /* access modifiers changed from: protected */
    public abstract String description();

    public DNSResolverTask(JmDNSImpl jmDNSImpl) {
        super(jmDNSImpl);
    }

    public String toString() {
        return super.toString() + " count: " + this._count;
    }

    public void start(Timer timer) {
        if (!getDns().isCanceling() && !getDns().isCanceled()) {
            timer.schedule(this, 225, 225);
        }
    }

    public void run() {
        try {
            if (!getDns().isCanceling()) {
                if (!getDns().isCanceled()) {
                    int i = this._count;
                    this._count = i + 1;
                    if (i < 3) {
                        if (logger.isLoggable(Level.FINER)) {
                            Logger logger2 = logger;
                            logger2.finer(getName() + ".run() JmDNS " + description());
                        }
                        DNSOutgoing out = addQuestions(new DNSOutgoing(0));
                        if (getDns().isAnnounced()) {
                            out = addAnswers(out);
                        }
                        if (!out.isEmpty()) {
                            getDns().send(out);
                        }
                        return;
                    }
                    cancel();
                    return;
                }
            }
            cancel();
        } catch (Throwable e) {
            Logger logger3 = logger;
            Level level = Level.WARNING;
            logger3.log(level, getName() + ".run() exception ", e);
            getDns().recover();
        }
    }
}
