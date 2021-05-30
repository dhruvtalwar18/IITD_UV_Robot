package org.jboss.netty.handler.ipfilter;

import java.net.Inet4Address;
import java.net.Inet6Address;
import java.net.InetAddress;
import java.net.UnknownHostException;

public class CIDR4 extends CIDR {
    private final int addressEndInt;
    private int addressInt;

    protected CIDR4(Inet4Address newaddr, int mask) {
        this.cidrMask = mask;
        this.addressInt = ipv4AddressToInt((InetAddress) newaddr);
        this.addressInt &= ipv4PrefixLengthToMask(mask);
        try {
            this.baseAddress = intToIPv4Address(this.addressInt);
        } catch (UnknownHostException e) {
        }
        this.addressEndInt = (this.addressInt + ipv4PrefixLengthToLength(this.cidrMask)) - 1;
    }

    public InetAddress getEndAddress() {
        try {
            return intToIPv4Address(this.addressEndInt);
        } catch (UnknownHostException e) {
            return null;
        }
    }

    public int compareTo(CIDR arg) {
        if (arg instanceof CIDR6) {
            int net = ipv4AddressToInt(getIpV4FromIpV6((Inet6Address) arg.baseAddress));
            if (net == this.addressInt && arg.cidrMask == this.cidrMask) {
                return 0;
            }
            if (net < this.addressInt) {
                return 1;
            }
            return (net <= this.addressInt && arg.cidrMask >= this.cidrMask) ? 1 : -1;
        }
        CIDR4 o = (CIDR4) arg;
        if (o.addressInt == this.addressInt && o.cidrMask == this.cidrMask) {
            return 0;
        }
        if (o.addressInt < this.addressInt) {
            return 1;
        }
        return (o.addressInt <= this.addressInt && o.cidrMask >= this.cidrMask) ? 1 : -1;
    }

    public boolean contains(InetAddress inetAddress) {
        int search = ipv4AddressToInt(inetAddress);
        return search >= this.addressInt && search <= this.addressEndInt;
    }

    private static int ipv4PrefixLengthToLength(int prefix_length) {
        return 1 << (32 - prefix_length);
    }

    private static int ipv4PrefixLengthToMask(int prefix_length) {
        return ((1 << (32 - prefix_length)) - 1) ^ -1;
    }

    private static InetAddress intToIPv4Address(int addr) throws UnknownHostException {
        return InetAddress.getByAddress(new byte[]{(byte) ((addr >> 24) & 255), (byte) ((addr >> 16) & 255), (byte) ((addr >> 8) & 255), (byte) (addr & 255)});
    }

    private static int ipv4AddressToInt(InetAddress addr) {
        byte[] address;
        if (addr instanceof Inet6Address) {
            address = getIpV4FromIpV6((Inet6Address) addr);
        } else {
            address = addr.getAddress();
        }
        return ipv4AddressToInt(address);
    }

    private static int ipv4AddressToInt(byte[] address) {
        int net = 0;
        for (byte addres : address) {
            net = (net << 8) | (addres & 255);
        }
        return net;
    }
}
