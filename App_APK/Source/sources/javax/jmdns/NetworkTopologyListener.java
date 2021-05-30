package javax.jmdns;

import java.util.EventListener;

public interface NetworkTopologyListener extends EventListener {
    void inetAddressAdded(NetworkTopologyEvent networkTopologyEvent);

    void inetAddressRemoved(NetworkTopologyEvent networkTopologyEvent);
}
