package org.apache.commons.net.util;

import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.apache.commons.httpclient.cookie.CookieSpec;

public class SubnetUtils {
    private static final String IP_ADDRESS = "(\\d{1,3})\\.(\\d{1,3})\\.(\\d{1,3})\\.(\\d{1,3})";
    private static final int NBITS = 32;
    private static final String SLASH_FORMAT = "(\\d{1,3})\\.(\\d{1,3})\\.(\\d{1,3})\\.(\\d{1,3})/(\\d{1,3})";
    private static final Pattern addressPattern = Pattern.compile(IP_ADDRESS);
    private static final Pattern cidrPattern = Pattern.compile(SLASH_FORMAT);
    /* access modifiers changed from: private */
    public int address = 0;
    /* access modifiers changed from: private */
    public int broadcast = 0;
    /* access modifiers changed from: private */
    public int netmask = 0;
    /* access modifiers changed from: private */
    public int network = 0;

    public SubnetUtils(String cidrNotation) {
        calculate(cidrNotation);
    }

    public SubnetUtils(String address2, String mask) {
        calculate(toCidrNotation(address2, mask));
    }

    public final class SubnetInfo {
        private SubnetInfo() {
        }

        private int netmask() {
            return SubnetUtils.this.netmask;
        }

        private int network() {
            return SubnetUtils.this.network;
        }

        private int address() {
            return SubnetUtils.this.address;
        }

        private int broadcast() {
            return SubnetUtils.this.broadcast;
        }

        private int low() {
            return network() + 1;
        }

        private int high() {
            return broadcast() - 1;
        }

        public boolean isInRange(String address) {
            return isInRange(SubnetUtils.this.toInteger(address));
        }

        private boolean isInRange(int address) {
            return address - low() <= high() - low();
        }

        public String getBroadcastAddress() {
            return SubnetUtils.this.format(SubnetUtils.this.toArray(broadcast()));
        }

        public String getNetworkAddress() {
            return SubnetUtils.this.format(SubnetUtils.this.toArray(network()));
        }

        public String getNetmask() {
            return SubnetUtils.this.format(SubnetUtils.this.toArray(netmask()));
        }

        public String getAddress() {
            return SubnetUtils.this.format(SubnetUtils.this.toArray(address()));
        }

        public String getLowAddress() {
            return SubnetUtils.this.format(SubnetUtils.this.toArray(low()));
        }

        public String getHighAddress() {
            return SubnetUtils.this.format(SubnetUtils.this.toArray(high()));
        }

        public int getAddressCount() {
            return broadcast() - low();
        }

        public int asInteger(String address) {
            return SubnetUtils.this.toInteger(address);
        }

        public String getCidrSignature() {
            return SubnetUtils.this.toCidrNotation(SubnetUtils.this.format(SubnetUtils.this.toArray(address())), SubnetUtils.this.format(SubnetUtils.this.toArray(netmask())));
        }

        public String[] getAllAddresses() {
            String[] addresses = new String[getAddressCount()];
            int add = low();
            int j = 0;
            while (add <= high()) {
                addresses[j] = SubnetUtils.this.format(SubnetUtils.this.toArray(add));
                add++;
                j++;
            }
            return addresses;
        }
    }

    public final SubnetInfo getInfo() {
        return new SubnetInfo();
    }

    private void calculate(String mask) {
        Matcher matcher = cidrPattern.matcher(mask);
        if (matcher.matches()) {
            this.address = matchAddress(matcher);
            int cidrPart = rangeCheck(Integer.parseInt(matcher.group(5)), 0, 31);
            for (int j = 0; j < cidrPart; j++) {
                this.netmask |= 1 << (31 - j);
            }
            this.network = this.address & this.netmask;
            this.broadcast = this.network | (this.netmask ^ -1);
            return;
        }
        throw new IllegalArgumentException("Could not parse [" + mask + "]");
    }

    /* access modifiers changed from: private */
    public int toInteger(String address2) {
        Matcher matcher = addressPattern.matcher(address2);
        if (matcher.matches()) {
            return matchAddress(matcher);
        }
        throw new IllegalArgumentException("Could not parse [" + address2 + "]");
    }

    private int matchAddress(Matcher matcher) {
        int addr = 0;
        for (int i = 1; i <= 4; i++) {
            addr |= (rangeCheck(Integer.parseInt(matcher.group(i)), 0, 255) & 255) << ((4 - i) * 8);
        }
        return addr;
    }

    /* access modifiers changed from: private */
    public int[] toArray(int val) {
        int[] ret = new int[4];
        for (int j = 3; j >= 0; j--) {
            ret[j] = ret[j] | ((val >>> ((3 - j) * 8)) & 255);
        }
        return ret;
    }

    /* access modifiers changed from: private */
    public String format(int[] octets) {
        StringBuilder str = new StringBuilder();
        for (int i = 0; i < octets.length; i++) {
            str.append(octets[i]);
            if (i != octets.length - 1) {
                str.append(".");
            }
        }
        return str.toString();
    }

    private int rangeCheck(int value, int begin, int end) {
        if (value >= begin && value <= end) {
            return value;
        }
        throw new IllegalArgumentException("Value out of range: [" + value + "]");
    }

    /* access modifiers changed from: package-private */
    public int pop(int x) {
        int x2 = x - ((x >>> 1) & 1431655765);
        int x3 = (x2 & 858993459) + (858993459 & (x2 >>> 2));
        int x4 = ((x3 >>> 4) + x3) & 252645135;
        int x5 = x4 + (x4 >>> 8);
        return (x5 + (x5 >>> 16)) & 63;
    }

    /* access modifiers changed from: private */
    public String toCidrNotation(String addr, String mask) {
        return addr + CookieSpec.PATH_DELIM + pop(toInteger(mask));
    }
}
