package org.ros.internal.transport;

import com.google.common.collect.Sets;
import java.util.Collection;

public interface ProtocolNames {
    public static final Collection<String> SUPPORTED = Sets.newHashSet((E[]) new String[]{TCPROS});
    public static final String TCPROS = "TCPROS";
    public static final String UDPROS = "UDPROS";
}
