package org.ros.master.uri;

import java.net.URI;
import java.util.concurrent.TimeUnit;
import org.ros.exception.RosRuntimeException;

public interface MasterUriProvider {
    URI getMasterUri() throws RosRuntimeException;

    URI getMasterUri(long j, TimeUnit timeUnit);
}
