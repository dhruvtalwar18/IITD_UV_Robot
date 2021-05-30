package org.jboss.netty.handler.ipfilter;

import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.StringTokenizer;
import java.util.Vector;
import org.apache.commons.httpclient.cookie.CookieSpec;
import org.jboss.netty.logging.InternalLogger;
import org.jboss.netty.logging.InternalLoggerFactory;

public class IpV4Subnet implements IpSet, Comparable<IpV4Subnet> {
    private static final int BYTE_ADDRESS_MASK = 255;
    private static final int SUBNET_MASK = Integer.MIN_VALUE;
    private static final InternalLogger logger = InternalLoggerFactory.getInstance((Class<?>) IpV4Subnet.class);
    private int cidrMask;
    private InetAddress inetAddress;
    private int mask;
    private int subnet;

    public IpV4Subnet() {
        this.mask = -1;
        this.inetAddress = null;
        this.subnet = 0;
        this.cidrMask = 0;
    }

    public IpV4Subnet(String netAddress) throws UnknownHostException {
        setNetAddress(netAddress);
    }

    public IpV4Subnet(InetAddress inetAddress2, int cidrNetMask) {
        setNetAddress(inetAddress2, cidrNetMask);
    }

    public IpV4Subnet(InetAddress inetAddress2, String netMask) {
        setNetAddress(inetAddress2, netMask);
    }

    private void setNetAddress(String netAddress) throws UnknownHostException {
        Vector<Object> vec = new Vector<>();
        StringTokenizer st = new StringTokenizer(netAddress, CookieSpec.PATH_DELIM);
        while (st.hasMoreTokens()) {
            vec.add(st.nextElement());
        }
        if (vec.get(1).toString().length() < 3) {
            setNetId(vec.get(0).toString());
            setCidrNetMask(Integer.parseInt(vec.get(1).toString()));
            return;
        }
        setNetId(vec.get(0).toString());
        setNetMask(vec.get(1).toString());
    }

    private void setNetAddress(InetAddress inetAddress2, int cidrNetMask) {
        setNetId(inetAddress2);
        setCidrNetMask(cidrNetMask);
    }

    private void setNetAddress(InetAddress inetAddress2, String netMask) {
        setNetId(inetAddress2);
        setNetMask(netMask);
    }

    private void setNetId(String netId) throws UnknownHostException {
        setNetId(InetAddress.getByName(netId));
    }

    private static int toInt(InetAddress inetAddress1) {
        int net = 0;
        for (byte addres : inetAddress1.getAddress()) {
            net = (net << 8) | (addres & 255);
        }
        return net;
    }

    private void setNetId(InetAddress inetAddress2) {
        this.inetAddress = inetAddress2;
        this.subnet = toInt(inetAddress2);
    }

    private void setNetMask(String netMask) {
        StringTokenizer nm = new StringTokenizer(netMask, ".");
        int i = 0;
        int[] netmask = new int[4];
        while (nm.hasMoreTokens()) {
            netmask[i] = Integer.parseInt(nm.nextToken());
            i++;
        }
        int mask1 = 0;
        for (int i2 = 0; i2 < 4; i2++) {
            mask1 += Integer.bitCount(netmask[i2]);
        }
        setCidrNetMask(mask1);
    }

    private void setCidrNetMask(int cidrNetMask) {
        this.cidrMask = cidrNetMask;
        this.mask = Integer.MIN_VALUE >> (this.cidrMask - 1);
    }

    public boolean contains(String ipAddr) throws UnknownHostException {
        return contains(InetAddress.getByName(ipAddr));
    }

    public boolean contains(InetAddress inetAddress1) {
        if (this.mask == -1 || (toInt(inetAddress1) & this.mask) == this.subnet) {
            return true;
        }
        return false;
    }

    public String toString() {
        return this.inetAddress.getHostAddress() + CookieSpec.PATH_DELIM + this.cidrMask;
    }

    public boolean equals(Object o) {
        if (!(o instanceof IpV4Subnet)) {
            return false;
        }
        IpV4Subnet ipV4Subnet = (IpV4Subnet) o;
        if (ipV4Subnet.subnet == this.subnet && ipV4Subnet.cidrMask == this.cidrMask) {
            return true;
        }
        return false;
    }

    public int hashCode() {
        return this.subnet;
    }

    public int compareTo(IpV4Subnet o) {
        if (o.subnet == this.subnet && o.cidrMask == this.cidrMask) {
            return 0;
        }
        if (o.subnet < this.subnet) {
            return 1;
        }
        if (o.subnet <= this.subnet && o.cidrMask >= this.cidrMask) {
            return 1;
        }
        return -1;
    }
}
