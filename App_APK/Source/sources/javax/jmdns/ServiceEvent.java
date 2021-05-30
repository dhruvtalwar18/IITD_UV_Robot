package javax.jmdns;

import java.util.EventObject;

public abstract class ServiceEvent extends EventObject implements Cloneable {
    private static final long serialVersionUID = -8558445644541006271L;

    public abstract JmDNS getDNS();

    public abstract ServiceInfo getInfo();

    public abstract String getName();

    public abstract String getType();

    public ServiceEvent(Object eventSource) {
        super(eventSource);
    }

    public ServiceEvent clone() {
        try {
            return (ServiceEvent) super.clone();
        } catch (CloneNotSupportedException e) {
            return null;
        }
    }
}
