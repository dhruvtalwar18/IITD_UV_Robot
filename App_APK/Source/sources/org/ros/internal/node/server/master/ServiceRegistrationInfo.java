package org.ros.internal.node.server.master;

import java.net.URI;
import org.ros.namespace.GraphName;

public class ServiceRegistrationInfo {
    private final NodeRegistrationInfo node;
    private final GraphName serviceName;
    private final URI serviceUri;

    public ServiceRegistrationInfo(GraphName serviceName2, URI serviceUri2, NodeRegistrationInfo node2) {
        this.serviceName = serviceName2;
        this.serviceUri = serviceUri2;
        this.node = node2;
    }

    public GraphName getServiceName() {
        return this.serviceName;
    }

    public URI getServiceUri() {
        return this.serviceUri;
    }

    public NodeRegistrationInfo getNode() {
        return this.node;
    }

    public int hashCode() {
        return (1 * 31) + this.serviceName.hashCode();
    }

    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (obj != null && getClass() == obj.getClass() && this.serviceName.equals(((ServiceRegistrationInfo) obj).serviceName)) {
            return true;
        }
        return false;
    }
}
