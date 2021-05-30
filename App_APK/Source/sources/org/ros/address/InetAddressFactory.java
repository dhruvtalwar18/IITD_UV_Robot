package org.ros.address;

import com.google.common.collect.Lists;
import com.google.common.net.InetAddresses;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import org.ros.exception.RosRuntimeException;
import org.xbill.DNS.Address;

public class InetAddressFactory {
    private InetAddressFactory() {
    }

    private static boolean isIpv4(InetAddress address) {
        return address.getAddress().length == 4;
    }

    private static Collection<InetAddress> getAllInetAddresses() {
        try {
            List<NetworkInterface> networkInterfaces = Collections.list(NetworkInterface.getNetworkInterfaces());
            List<InetAddress> inetAddresses = Lists.newArrayList();
            for (NetworkInterface networkInterface : networkInterfaces) {
                try {
                    if (networkInterface.isUp()) {
                        inetAddresses.addAll(Collections.list(networkInterface.getInetAddresses()));
                    }
                } catch (SocketException e) {
                    throw new RosRuntimeException((Throwable) e);
                }
            }
            return inetAddresses;
        } catch (SocketException e2) {
            throw new RosRuntimeException((Throwable) e2);
        }
    }

    private static InetAddress filterInetAddresses(Collection<InetAddress> inetAddresses) {
        for (InetAddress address : inetAddresses) {
            if (!address.isLoopbackAddress() && isIpv4(address)) {
                return address;
            }
        }
        throw new RosRuntimeException("No non-loopback interface found.");
    }

    public static InetAddress newNonLoopback() {
        return filterInetAddresses(getAllInetAddresses());
    }

    public static InetAddress newNonLoopbackForNetworkInterface(NetworkInterface networkInterface) {
        return filterInetAddresses(Collections.list(networkInterface.getInetAddresses()));
    }

    private static Collection<InetAddress> getAllInetAddressesByName(String host) {
        InetAddress[] allAddressesByName;
        try {
            allAddressesByName = Address.getAllByName(host);
        } catch (UnknownHostException e) {
            try {
                allAddressesByName = InetAddress.getAllByName(host);
            } catch (UnknownHostException e2) {
                throw new RosRuntimeException((Throwable) e2);
            }
        }
        return Arrays.asList(allAddressesByName);
    }

    public static InetAddress newFromHostString(String host) {
        try {
            if (InetAddresses.isInetAddress(host)) {
                return InetAddress.getByAddress(host, InetAddresses.forString(host).getAddress());
            }
            if (host.equals(Address.LOCALHOST)) {
                return InetAddress.getByAddress(Address.LOCALHOST, InetAddresses.forString(Address.LOOPBACK).getAddress());
            }
            Collection<InetAddress> allAddressesByName = getAllInetAddressesByName(host);
            for (InetAddress address : allAddressesByName) {
                if (!address.isLoopbackAddress() && isIpv4(address)) {
                    return address;
                }
            }
            for (InetAddress address2 : allAddressesByName) {
                if (isIpv4(address2)) {
                    return address2;
                }
            }
            throw new RosRuntimeException("Unable to construct InetAddress for host: " + host);
        } catch (UnknownHostException e) {
            throw new RosRuntimeException((Throwable) e);
        }
    }

    public static InetAddress newLoopback() {
        return newFromHostString(Address.LOOPBACK);
    }
}
