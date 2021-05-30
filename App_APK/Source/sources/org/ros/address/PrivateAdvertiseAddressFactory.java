package org.ros.address;

public class PrivateAdvertiseAddressFactory implements AdvertiseAddressFactory {
    public AdvertiseAddress newDefault() {
        return new AdvertiseAddress(Address.LOOPBACK);
    }
}
