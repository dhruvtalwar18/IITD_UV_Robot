package javax.jmdns.impl;

import javax.jmdns.JmDNS;
import javax.jmdns.ServiceEvent;
import javax.jmdns.ServiceInfo;

public class ServiceEventImpl extends ServiceEvent {
    private static final long serialVersionUID = 7107973622016897488L;
    private final ServiceInfo _info;
    private final String _name;
    private final String _type;

    public ServiceEventImpl(JmDNSImpl jmDNS, String type, String name, ServiceInfo info) {
        super(jmDNS);
        this._type = type;
        this._name = name;
        this._info = info;
    }

    public JmDNS getDNS() {
        return (JmDNS) getSource();
    }

    public String getType() {
        return this._type;
    }

    public String getName() {
        return this._name;
    }

    public String toString() {
        StringBuilder buf = new StringBuilder();
        buf.append("[" + getClass().getSimpleName() + "@" + System.identityHashCode(this) + " ");
        buf.append("\n\tname: '");
        buf.append(getName());
        buf.append("' type: '");
        buf.append(getType());
        buf.append("' info: '");
        buf.append(getInfo());
        buf.append("']");
        return buf.toString();
    }

    public ServiceInfo getInfo() {
        return this._info;
    }

    public ServiceEventImpl clone() {
        return new ServiceEventImpl((JmDNSImpl) getDNS(), getType(), getName(), new ServiceInfoImpl(getInfo()));
    }
}
