package javax.jmdns.impl.tasks;

import java.util.Timer;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.jmdns.impl.JmDNSImpl;

public class RecordReaper extends DNSTask {
    static Logger logger = Logger.getLogger(RecordReaper.class.getName());

    public RecordReaper(JmDNSImpl jmDNSImpl) {
        super(jmDNSImpl);
    }

    public String getName() {
        StringBuilder sb = new StringBuilder();
        sb.append("RecordReaper(");
        sb.append(getDns() != null ? getDns().getName() : "");
        sb.append(")");
        return sb.toString();
    }

    public void start(Timer timer) {
        if (!getDns().isCanceling() && !getDns().isCanceled()) {
            timer.schedule(this, 10000, 10000);
        }
    }

    public void run() {
        if (!getDns().isCanceling() && !getDns().isCanceled()) {
            if (logger.isLoggable(Level.FINEST)) {
                Logger logger2 = logger;
                logger2.finest(getName() + ".run() JmDNS reaping cache");
            }
            getDns().cleanCache();
        }
    }
}
