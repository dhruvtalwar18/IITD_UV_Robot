package javax.jmdns.impl.tasks.state;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.jmdns.ServiceInfo;
import javax.jmdns.impl.DNSOutgoing;
import javax.jmdns.impl.DNSStatefulObject;
import javax.jmdns.impl.JmDNSImpl;
import javax.jmdns.impl.ServiceInfoImpl;
import javax.jmdns.impl.constants.DNSConstants;
import javax.jmdns.impl.constants.DNSState;
import javax.jmdns.impl.tasks.DNSTask;

public abstract class DNSStateTask extends DNSTask {
    private static int _defaultTTL = DNSConstants.DNS_TTL;
    static Logger logger1 = Logger.getLogger(DNSStateTask.class.getName());
    private DNSState _taskState = null;
    private final int _ttl;

    /* access modifiers changed from: protected */
    public abstract void advanceTask();

    /* access modifiers changed from: protected */
    public abstract DNSOutgoing buildOutgoingForDNS(DNSOutgoing dNSOutgoing) throws IOException;

    /* access modifiers changed from: protected */
    public abstract DNSOutgoing buildOutgoingForInfo(ServiceInfoImpl serviceInfoImpl, DNSOutgoing dNSOutgoing) throws IOException;

    /* access modifiers changed from: protected */
    public abstract boolean checkRunCondition();

    /* access modifiers changed from: protected */
    public abstract DNSOutgoing createOugoing();

    public abstract String getTaskDescription();

    /* access modifiers changed from: protected */
    public abstract void recoverTask(Throwable th);

    public static int defaultTTL() {
        return _defaultTTL;
    }

    public static void setDefaultTTL(int value) {
        _defaultTTL = value;
    }

    public DNSStateTask(JmDNSImpl jmDNSImpl, int ttl) {
        super(jmDNSImpl);
        this._ttl = ttl;
    }

    public int getTTL() {
        return this._ttl;
    }

    /* access modifiers changed from: protected */
    public void associate(DNSState state) {
        synchronized (getDns()) {
            getDns().associateWithTask(this, state);
        }
        Iterator<ServiceInfo> it = getDns().getServices().values().iterator();
        while (it.hasNext()) {
            ((ServiceInfoImpl) it.next()).associateWithTask(this, state);
        }
    }

    /* access modifiers changed from: protected */
    public void removeAssociation() {
        synchronized (getDns()) {
            getDns().removeAssociationWithTask(this);
        }
        Iterator<ServiceInfo> it = getDns().getServices().values().iterator();
        while (it.hasNext()) {
            ((ServiceInfoImpl) it.next()).removeAssociationWithTask(this);
        }
    }

    public void run() {
        DNSOutgoing out = createOugoing();
        try {
            if (!checkRunCondition()) {
                cancel();
                return;
            }
            List<DNSStatefulObject> stateObjects = new ArrayList<>();
            synchronized (getDns()) {
                if (getDns().isAssociatedWithTask(this, getTaskState())) {
                    Logger logger = logger1;
                    logger.finer(getName() + ".run() JmDNS " + getTaskDescription() + " " + getDns().getName());
                    stateObjects.add(getDns());
                    out = buildOutgoingForDNS(out);
                }
            }
            Iterator<ServiceInfo> it = getDns().getServices().values().iterator();
            while (it.hasNext()) {
                ServiceInfoImpl info = (ServiceInfoImpl) it.next();
                synchronized (info) {
                    if (info.isAssociatedWithTask(this, getTaskState())) {
                        Logger logger2 = logger1;
                        logger2.fine(getName() + ".run() JmDNS " + getTaskDescription() + " " + info.getQualifiedName());
                        stateObjects.add(info);
                        out = buildOutgoingForInfo(info, out);
                    }
                }
            }
            if (!out.isEmpty()) {
                Logger logger3 = logger1;
                logger3.finer(getName() + ".run() JmDNS " + getTaskDescription() + " #" + getTaskState());
                getDns().send(out);
                advanceObjectsState(stateObjects);
                advanceTask();
                return;
            }
            advanceObjectsState(stateObjects);
            cancel();
        } catch (Throwable e) {
            Logger logger4 = logger1;
            Level level = Level.WARNING;
            logger4.log(level, getName() + ".run() exception ", e);
            recoverTask(e);
        }
    }

    /* access modifiers changed from: protected */
    public void advanceObjectsState(List<DNSStatefulObject> list) {
        if (list != null) {
            for (DNSStatefulObject object : list) {
                synchronized (object) {
                    object.advanceState(this);
                }
            }
        }
    }

    /* access modifiers changed from: protected */
    public DNSState getTaskState() {
        return this._taskState;
    }

    /* access modifiers changed from: protected */
    public void setTaskState(DNSState taskState) {
        this._taskState = taskState;
    }
}
