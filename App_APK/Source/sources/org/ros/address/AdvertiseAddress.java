package org.ros.address;

import com.google.common.base.Preconditions;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.URI;
import java.util.concurrent.Callable;
import org.apache.commons.httpclient.cookie.CookieSpec;
import org.ros.exception.RosRuntimeException;

public class AdvertiseAddress {
    private final String host;
    private Callable<Integer> portCallable;

    public static AdvertiseAddress newPrivate() {
        return new PrivateAdvertiseAddressFactory().newDefault();
    }

    public static AdvertiseAddress newPublic() {
        return new PublicAdvertiseAddressFactory().newDefault();
    }

    public AdvertiseAddress(String host2) {
        Preconditions.checkNotNull(host2);
        this.host = host2;
    }

    public String getHost() {
        return this.host;
    }

    public void setStaticPort(final int port) {
        this.portCallable = new Callable<Integer>() {
            public Integer call() throws Exception {
                return Integer.valueOf(port);
            }
        };
    }

    public int getPort() {
        try {
            return this.portCallable.call().intValue();
        } catch (Exception e) {
            throw new RosRuntimeException((Throwable) e);
        }
    }

    public void setPortCallable(Callable<Integer> portCallable2) {
        this.portCallable = portCallable2;
    }

    public InetAddress toInetAddress() {
        return InetAddressFactory.newFromHostString(this.host);
    }

    public InetSocketAddress toInetSocketAddress() {
        Preconditions.checkNotNull(this.portCallable);
        try {
            return new InetSocketAddress(toInetAddress(), this.portCallable.call().intValue());
        } catch (Exception e) {
            throw new RosRuntimeException((Throwable) e);
        }
    }

    public URI toUri(String scheme) {
        Preconditions.checkNotNull(this.portCallable);
        try {
            return new URI(scheme, (String) null, this.host, this.portCallable.call().intValue(), CookieSpec.PATH_DELIM, (String) null, (String) null);
        } catch (Exception e) {
            throw new RosRuntimeException("Failed to create URI: " + this, e);
        }
    }

    public boolean isLoopbackAddress() {
        return toInetAddress().isLoopbackAddress();
    }

    public String toString() {
        Preconditions.checkNotNull(this.portCallable);
        try {
            return "AdvertiseAddress<" + this.host + ", " + this.portCallable.call() + ">";
        } catch (Exception e) {
            throw new RosRuntimeException((Throwable) e);
        }
    }

    public int hashCode() {
        Preconditions.checkNotNull(this.portCallable);
        try {
            return (((1 * 31) + (this.host == null ? 0 : this.host.hashCode())) * 31) + this.portCallable.call().intValue();
        } catch (Exception e) {
            throw new RosRuntimeException((Throwable) e);
        }
    }

    public boolean equals(Object obj) {
        Preconditions.checkNotNull(this.portCallable);
        if (this == obj) {
            return true;
        }
        if (obj == null || getClass() != obj.getClass()) {
            return false;
        }
        AdvertiseAddress other = (AdvertiseAddress) obj;
        if (this.host == null) {
            if (other.host != null) {
                return false;
            }
        } else if (!this.host.equals(other.host)) {
            return false;
        }
        try {
            if (this.portCallable.call() != other.portCallable.call()) {
                return false;
            }
            return true;
        } catch (Exception e) {
            throw new RosRuntimeException((Throwable) e);
        }
    }
}
