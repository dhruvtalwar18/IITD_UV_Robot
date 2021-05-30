package javax.jmdns;

import java.net.Inet4Address;
import java.net.Inet6Address;
import java.net.InetAddress;
import java.util.Enumeration;
import java.util.Map;
import javax.jmdns.impl.ServiceInfoImpl;

public abstract class ServiceInfo implements Cloneable {
    public static final byte[] NO_VALUE = new byte[0];

    public enum Fields {
        Domain,
        Protocol,
        Application,
        Instance,
        Subtype
    }

    @Deprecated
    public abstract InetAddress getAddress();

    public abstract String getApplication();

    public abstract String getDomain();

    @Deprecated
    public abstract String getHostAddress();

    public abstract String[] getHostAddresses();

    @Deprecated
    public abstract Inet4Address getInet4Address();

    public abstract Inet4Address[] getInet4Addresses();

    @Deprecated
    public abstract Inet6Address getInet6Address();

    public abstract Inet6Address[] getInet6Addresses();

    @Deprecated
    public abstract InetAddress getInetAddress();

    public abstract InetAddress[] getInetAddresses();

    public abstract String getKey();

    public abstract String getName();

    public abstract String getNiceTextString();

    public abstract int getPort();

    public abstract int getPriority();

    public abstract byte[] getPropertyBytes(String str);

    public abstract Enumeration<String> getPropertyNames();

    public abstract String getPropertyString(String str);

    public abstract String getProtocol();

    public abstract String getQualifiedName();

    public abstract Map<Fields, String> getQualifiedNameMap();

    public abstract String getServer();

    public abstract String getSubtype();

    public abstract byte[] getTextBytes();

    @Deprecated
    public abstract String getTextString();

    public abstract String getType();

    public abstract String getTypeWithSubtype();

    @Deprecated
    public abstract String getURL();

    @Deprecated
    public abstract String getURL(String str);

    public abstract String[] getURLs();

    public abstract String[] getURLs(String str);

    public abstract int getWeight();

    public abstract boolean hasData();

    public abstract boolean isPersistent();

    public abstract void setText(Map<String, ?> map) throws IllegalStateException;

    public abstract void setText(byte[] bArr) throws IllegalStateException;

    public static ServiceInfo create(String type, String name, int port, String text) {
        return new ServiceInfoImpl(type, name, "", port, 0, 0, false, text);
    }

    public static ServiceInfo create(String type, String name, String subtype, int port, String text) {
        return new ServiceInfoImpl(type, name, subtype, port, 0, 0, false, text);
    }

    public static ServiceInfo create(String type, String name, int port, int weight, int priority, String text) {
        return new ServiceInfoImpl(type, name, "", port, weight, priority, false, text);
    }

    public static ServiceInfo create(String type, String name, String subtype, int port, int weight, int priority, String text) {
        return new ServiceInfoImpl(type, name, subtype, port, weight, priority, false, text);
    }

    public static ServiceInfo create(String type, String name, int port, int weight, int priority, Map<String, ?> props) {
        return new ServiceInfoImpl(type, name, "", port, weight, priority, false, props);
    }

    public static ServiceInfo create(String type, String name, String subtype, int port, int weight, int priority, Map<String, ?> props) {
        return new ServiceInfoImpl(type, name, subtype, port, weight, priority, false, props);
    }

    public static ServiceInfo create(String type, String name, int port, int weight, int priority, byte[] text) {
        return new ServiceInfoImpl(type, name, "", port, weight, priority, false, text);
    }

    public static ServiceInfo create(String type, String name, String subtype, int port, int weight, int priority, byte[] text) {
        return new ServiceInfoImpl(type, name, subtype, port, weight, priority, false, text);
    }

    public static ServiceInfo create(String type, String name, int port, int weight, int priority, boolean persistent, String text) {
        return new ServiceInfoImpl(type, name, "", port, weight, priority, persistent, text);
    }

    public static ServiceInfo create(String type, String name, String subtype, int port, int weight, int priority, boolean persistent, String text) {
        return new ServiceInfoImpl(type, name, subtype, port, weight, priority, persistent, text);
    }

    public static ServiceInfo create(String type, String name, int port, int weight, int priority, boolean persistent, Map<String, ?> props) {
        return new ServiceInfoImpl(type, name, "", port, weight, priority, persistent, props);
    }

    public static ServiceInfo create(String type, String name, String subtype, int port, int weight, int priority, boolean persistent, Map<String, ?> props) {
        return new ServiceInfoImpl(type, name, subtype, port, weight, priority, persistent, props);
    }

    public static ServiceInfo create(String type, String name, int port, int weight, int priority, boolean persistent, byte[] text) {
        return new ServiceInfoImpl(type, name, "", port, weight, priority, persistent, text);
    }

    public static ServiceInfo create(String type, String name, String subtype, int port, int weight, int priority, boolean persistent, byte[] text) {
        return new ServiceInfoImpl(type, name, subtype, port, weight, priority, persistent, text);
    }

    public static ServiceInfo create(Map<Fields, String> qualifiedNameMap, int port, int weight, int priority, boolean persistent, Map<String, ?> props) {
        return new ServiceInfoImpl(qualifiedNameMap, port, weight, priority, persistent, props);
    }

    public ServiceInfo clone() {
        try {
            return (ServiceInfo) super.clone();
        } catch (CloneNotSupportedException e) {
            return null;
        }
    }
}
