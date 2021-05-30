package org.apache.commons.net.ntp;

public final class NtpUtils {
    public static String getHostAddress(int address) {
        return ((address >>> 24) & 255) + "." + ((address >>> 16) & 255) + "." + ((address >>> 8) & 255) + "." + ((address >>> 0) & 255);
    }

    public static String getRefAddress(NtpV3Packet packet) {
        return getHostAddress(packet == null ? 0 : packet.getReferenceId());
    }

    public static String getReferenceClock(NtpV3Packet message) {
        int refId;
        if (message == null || (refId = message.getReferenceId()) == 0) {
            return "";
        }
        StringBuffer buf = new StringBuffer(4);
        int shiftBits = 24;
        while (shiftBits >= 0) {
            char c = (char) ((refId >>> shiftBits) & 255);
            if (c == 0) {
                break;
            } else if (!Character.isLetterOrDigit(c)) {
                return "";
            } else {
                buf.append(c);
                shiftBits -= 8;
            }
        }
        return buf.toString();
    }

    public static String getModeName(int mode) {
        switch (mode) {
            case 0:
                return "Reserved";
            case 1:
                return "Symmetric Active";
            case 2:
                return "Symmetric Passive";
            case 3:
                return "Client";
            case 4:
                return "Server";
            case 5:
                return "Broadcast";
            case 6:
                return "Control";
            case 7:
                return "Private";
            default:
                return "Unknown";
        }
    }
}
