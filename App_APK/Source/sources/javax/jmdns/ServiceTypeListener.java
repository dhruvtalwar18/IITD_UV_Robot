package javax.jmdns;

import java.util.EventListener;

public interface ServiceTypeListener extends EventListener {
    void serviceTypeAdded(ServiceEvent serviceEvent);

    void subTypeForServiceTypeAdded(ServiceEvent serviceEvent);
}
