package org.ros.internal.node.response;

import com.google.common.base.Preconditions;
import java.util.Arrays;
import java.util.List;
import org.ros.address.AdvertiseAddress;
import org.ros.internal.transport.ProtocolDescription;
import org.ros.internal.transport.ProtocolNames;
import org.ros.internal.transport.tcp.TcpRosProtocolDescription;

public class ProtocolDescriptionResultFactory implements ResultFactory<ProtocolDescription> {
    public ProtocolDescription newFromValue(Object value) {
        List<Object> protocolParameters = Arrays.asList((Object[]) value);
        Preconditions.checkState(protocolParameters.size() == 3);
        Preconditions.checkState(protocolParameters.get(0).equals(ProtocolNames.TCPROS));
        AdvertiseAddress address = new AdvertiseAddress((String) protocolParameters.get(1));
        address.setStaticPort(((Integer) protocolParameters.get(2)).intValue());
        return new TcpRosProtocolDescription(address);
    }
}
