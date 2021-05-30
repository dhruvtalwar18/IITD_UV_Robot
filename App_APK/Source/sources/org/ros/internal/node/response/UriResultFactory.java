package org.ros.internal.node.response;

import java.net.URI;
import java.net.URISyntaxException;
import org.ros.exception.RosRuntimeException;

public class UriResultFactory implements ResultFactory<URI> {
    public URI newFromValue(Object value) {
        try {
            return new URI((String) value);
        } catch (URISyntaxException e) {
            throw new RosRuntimeException((Throwable) e);
        }
    }
}
