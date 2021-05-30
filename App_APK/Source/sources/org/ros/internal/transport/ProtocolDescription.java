package org.ros.internal.transport;

import com.google.common.collect.Lists;
import java.net.InetSocketAddress;
import java.util.List;
import org.ros.address.AdvertiseAddress;

public class ProtocolDescription {
    private final AdvertiseAddress address;
    private final String name;

    public ProtocolDescription(String name2, AdvertiseAddress address2) {
        this.name = name2;
        this.address = address2;
    }

    public String getName() {
        return this.name;
    }

    public AdvertiseAddress getAdverstiseAddress() {
        return this.address;
    }

    public InetSocketAddress getAddress() {
        return this.address.toInetSocketAddress();
    }

    public List<Object> toList() {
        return Lists.newArrayList((E[]) new Object[]{this.name, this.address.getHost(), Integer.valueOf(this.address.getPort())});
    }

    public String toString() {
        return "Protocol<" + this.name + ", " + getAdverstiseAddress() + ">";
    }

    public int hashCode() {
        int i = 0;
        int result = ((1 * 31) + (this.address == null ? 0 : this.address.hashCode())) * 31;
        if (this.name != null) {
            i = this.name.hashCode();
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
        ProtocolDescription other = (ProtocolDescription) obj;
        if (this.address == null) {
            if (other.address != null) {
                return false;
            }
        } else if (!this.address.equals(other.address)) {
            return false;
        }
        if (this.name == null) {
            if (other.name != null) {
                return false;
            }
        } else if (!this.name.equals(other.name)) {
            return false;
        }
        return true;
    }
}
