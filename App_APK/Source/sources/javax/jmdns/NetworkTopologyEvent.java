package javax.jmdns;

import java.net.InetAddress;
import java.util.EventObject;

public abstract class NetworkTopologyEvent extends EventObject {
    private static final long serialVersionUID = -8630033521752540987L;

    public abstract JmDNS getDNS();

    public abstract InetAddress getInetAddress();

    protected NetworkTopologyEvent(Object eventSource) {
        super(eventSource);
    }
}
