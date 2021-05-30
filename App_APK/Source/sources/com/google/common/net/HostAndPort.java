package com.google.common.net;

import com.google.common.annotations.Beta;
import com.google.common.base.Objects;
import com.google.common.base.Preconditions;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import javax.annotation.concurrent.Immutable;

@Immutable
@Beta
public final class HostAndPort {
    private static final Pattern BRACKET_PATTERN = Pattern.compile("^\\[(.*:.*)\\](?::(\\d*))?$");
    private static final int NO_PORT = -1;
    private final boolean hasBracketlessColons;
    private final String host;
    private final int port;

    private HostAndPort(String host2, int port2, boolean hasBracketlessColons2) {
        this.host = host2;
        this.port = port2;
        this.hasBracketlessColons = hasBracketlessColons2;
    }

    public String getHostText() {
        return this.host;
    }

    public boolean hasPort() {
        return this.port >= 0;
    }

    public int getPort() {
        Preconditions.checkState(hasPort());
        return this.port;
    }

    public int getPortOrDefault(int defaultPort) {
        return hasPort() ? this.port : defaultPort;
    }

    public static HostAndPort fromParts(String host2, int port2) {
        Preconditions.checkArgument(isValidPort(port2));
        HostAndPort parsedHost = fromString(host2);
        Preconditions.checkArgument(!parsedHost.hasPort());
        return new HostAndPort(parsedHost.host, port2, parsedHost.hasBracketlessColons);
    }

    public static HostAndPort fromString(String hostPortString) {
        String host2;
        Preconditions.checkNotNull(hostPortString);
        String portString = null;
        boolean hasBracketlessColons2 = false;
        if (hostPortString.startsWith("[")) {
            Matcher matcher = BRACKET_PATTERN.matcher(hostPortString);
            Preconditions.checkArgument(matcher.matches(), "Invalid bracketed host/port: %s", hostPortString);
            String host3 = matcher.group(1);
            portString = matcher.group(2);
            host2 = host3;
        } else {
            int colonPos = hostPortString.indexOf(58);
            if (colonPos < 0 || hostPortString.indexOf(58, colonPos + 1) != -1) {
                host2 = hostPortString;
                hasBracketlessColons2 = colonPos >= 0;
            } else {
                host2 = hostPortString.substring(0, colonPos);
                portString = hostPortString.substring(colonPos + 1);
            }
        }
        int port2 = -1;
        if (portString != null) {
            Preconditions.checkArgument(!portString.startsWith("+"), "Unparseable port number: %s", hostPortString);
            try {
                port2 = Integer.parseInt(portString);
                Preconditions.checkArgument(isValidPort(port2), "Port number out of range: %s", hostPortString);
            } catch (NumberFormatException e) {
                throw new IllegalArgumentException("Unparseable port number: " + hostPortString);
            }
        }
        return new HostAndPort(host2, port2, hasBracketlessColons2);
    }

    public HostAndPort withDefaultPort(int defaultPort) {
        Preconditions.checkArgument(isValidPort(defaultPort));
        if (hasPort() || this.port == defaultPort) {
            return this;
        }
        return new HostAndPort(this.host, defaultPort, this.hasBracketlessColons);
    }

    public HostAndPort requireBracketsForIPv6() {
        Preconditions.checkArgument(!this.hasBracketlessColons, "Possible bracketless IPv6 literal: %s", this.host);
        return this;
    }

    public boolean equals(Object other) {
        if (this == other) {
            return true;
        }
        if (!(other instanceof HostAndPort)) {
            return false;
        }
        HostAndPort that = (HostAndPort) other;
        if (Objects.equal(this.host, that.host) && this.port == that.port && this.hasBracketlessColons == that.hasBracketlessColons) {
            return true;
        }
        return false;
    }

    public int hashCode() {
        return Objects.hashCode(this.host, Integer.valueOf(this.port), Boolean.valueOf(this.hasBracketlessColons));
    }

    public String toString() {
        StringBuilder builder = new StringBuilder(this.host.length() + 7);
        if (this.host.indexOf(58) >= 0) {
            builder.append('[');
            builder.append(this.host);
            builder.append(']');
        } else {
            builder.append(this.host);
        }
        if (hasPort()) {
            builder.append(':');
            builder.append(this.port);
        }
        return builder.toString();
    }

    private static boolean isValidPort(int port2) {
        return port2 >= 0 && port2 <= 65535;
    }
}
