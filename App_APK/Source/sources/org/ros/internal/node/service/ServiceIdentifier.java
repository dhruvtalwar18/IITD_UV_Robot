package org.ros.internal.node.service;

import com.google.common.base.Preconditions;
import java.net.URI;
import org.ros.namespace.GraphName;

public class ServiceIdentifier {
    private final GraphName name;
    private final URI uri;

    public ServiceIdentifier(GraphName name2, URI uri2) {
        Preconditions.checkNotNull(name2);
        Preconditions.checkArgument(name2.isGlobal());
        this.name = name2;
        this.uri = uri2;
    }

    public GraphName getName() {
        return this.name;
    }

    public URI getUri() {
        return this.uri;
    }

    public String toString() {
        return "ServiceIdentifier<" + this.name + ", " + this.uri + ">";
    }

    public int hashCode() {
        int i = 0;
        int result = ((1 * 31) + (this.name == null ? 0 : this.name.hashCode())) * 31;
        if (this.uri != null) {
            i = this.uri.hashCode();
        }
        return result + i;
    }

    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (obj == null || getClass() != obj.getClass()) {
            return false;
        }
        ServiceIdentifier other = (ServiceIdentifier) obj;
        if (this.name == null) {
            if (other.name != null) {
                return false;
            }
        } else if (!this.name.equals(other.name)) {
            return false;
        }
        if (this.uri == null) {
            if (other.uri != null) {
                return false;
            }
        } else if (!this.uri.equals(other.uri)) {
            return false;
        }
        return true;
    }
}
