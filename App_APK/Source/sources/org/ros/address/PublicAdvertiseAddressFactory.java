package org.ros.address;

public class PublicAdvertiseAddressFactory implements AdvertiseAddressFactory {
    private final String host;

    public PublicAdvertiseAddressFactory() {
        this(InetAddressFactory.newNonLoopback().getCanonicalHostName());
    }

    public PublicAdvertiseAddressFactory(String host2) {
        this.host = host2;
    }

    public AdvertiseAddress newDefault() {
        return new AdvertiseAddress(this.host);
    }
}
