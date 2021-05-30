package org.ros.internal.transport.tcp;

import org.ros.address.AdvertiseAddress;
import org.ros.internal.transport.ProtocolDescription;
import org.ros.internal.transport.ProtocolNames;

public class TcpRosProtocolDescription extends ProtocolDescription {
    public TcpRosProtocolDescription(AdvertiseAddress address) {
        super(ProtocolNames.TCPROS, address);
    }
}
