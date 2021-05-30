package org.ros.address;

import java.net.InetSocketAddress;

public class BindAddress {
    private final InetSocketAddress address;

    private BindAddress(InetSocketAddress address2) {
        this.address = address2;
    }

    public static BindAddress newPublic(int port) {
        return new BindAddress(new InetSocketAddress(port));
    }

    public static BindAddress newPublic() {
        return newPublic(0);
    }

    public static BindAddress newPrivate(int port) {
        return new BindAddress(new InetSocketAddress(InetAddressFactory.newLoopback(), port));
    }

    public static BindAddress newPrivate() {
        return newPrivate(0);
    }

    public String toString() {
        return "BindAddress<" + this.address + ">";
    }

    public InetSocketAddress toInetSocketAddress() {
        return this.address;
    }

    public int hashCode() {
        return (1 * 31) + (this.address == null ? 0 : this.address.hashCode());
    }

    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (obj == null || getClass() != obj.getClass()) {
            return false;
        }
        BindAddress other = (BindAddress) obj;
        if (this.address == null) {
            if (other.address != null) {
                return false;
            }
        } else if (!this.address.equals(other.address)) {
            return false;
        }
        return true;
    }
}
