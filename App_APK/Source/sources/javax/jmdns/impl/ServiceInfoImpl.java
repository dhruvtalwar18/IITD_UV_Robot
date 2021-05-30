package javax.jmdns.impl;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.net.Inet4Address;
import java.net.Inet6Address;
import java.net.InetAddress;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.Vector;
import java.util.logging.Logger;
import javax.jmdns.ServiceEvent;
import javax.jmdns.ServiceInfo;
import javax.jmdns.impl.DNSRecord;
import javax.jmdns.impl.DNSStatefulObject;
import javax.jmdns.impl.constants.DNSRecordClass;
import javax.jmdns.impl.constants.DNSRecordType;
import javax.jmdns.impl.constants.DNSState;
import javax.jmdns.impl.tasks.DNSTask;
import org.apache.commons.httpclient.cookie.Cookie2;
import org.apache.commons.httpclient.cookie.CookieSpec;
import org.apache.xmlrpc.serializer.ObjectArraySerializer;

public class ServiceInfoImpl extends ServiceInfo implements DNSListener, DNSStatefulObject {
    private static Logger logger = Logger.getLogger(ServiceInfoImpl.class.getName());
    private String _application;
    private Delegate _delegate;
    private String _domain;
    private final Set<Inet4Address> _ipv4Addresses;
    private final Set<Inet6Address> _ipv6Addresses;
    private transient String _key;
    private String _name;
    private boolean _needTextAnnouncing;
    private boolean _persistent;
    private int _port;
    private int _priority;
    private Map<String, byte[]> _props;
    private String _protocol;
    private String _server;
    private final ServiceInfoState _state;
    private String _subtype;
    private byte[] _text;
    private int _weight;

    public interface Delegate {
        void textValueUpdated(ServiceInfo serviceInfo, byte[] bArr);
    }

    private static final class ServiceInfoState extends DNSStatefulObject.DefaultImplementation {
        private static final long serialVersionUID = 1104131034952196820L;
        private final ServiceInfoImpl _info;

        public ServiceInfoState(ServiceInfoImpl info) {
            this._info = info;
        }

        /* access modifiers changed from: protected */
        public void setTask(DNSTask task) {
            super.setTask(task);
            if (this._task == null && this._info.needTextAnnouncing()) {
                lock();
                try {
                    if (this._task == null && this._info.needTextAnnouncing()) {
                        if (this._state.isAnnounced()) {
                            setState(DNSState.ANNOUNCING_1);
                            if (getDns() != null) {
                                getDns().startAnnouncer();
                            }
                        }
                        this._info.setNeedTextAnnouncing(false);
                    }
                } finally {
                    unlock();
                }
            }
        }

        public void setDns(JmDNSImpl dns) {
            super.setDns(dns);
        }
    }

    public ServiceInfoImpl(String type, String name, String subtype, int port, int weight, int priority, boolean persistent, String text) {
        this(decodeQualifiedNameMap(type, name, subtype), port, weight, priority, persistent, (byte[]) null);
        this._server = text;
        try {
            ByteArrayOutputStream out = new ByteArrayOutputStream(text.length());
            writeUTF(out, text);
            this._text = out.toByteArray();
        } catch (IOException e) {
            throw new RuntimeException("unexpected exception: " + e);
        }
    }

    public ServiceInfoImpl(String type, String name, String subtype, int port, int weight, int priority, boolean persistent, Map<String, ?> props) {
        this(decodeQualifiedNameMap(type, name, subtype), port, weight, priority, persistent, textFromProperties(props));
    }

    public ServiceInfoImpl(String type, String name, String subtype, int port, int weight, int priority, boolean persistent, byte[] text) {
        this(decodeQualifiedNameMap(type, name, subtype), port, weight, priority, persistent, text);
    }

    public ServiceInfoImpl(Map<ServiceInfo.Fields, String> qualifiedNameMap, int port, int weight, int priority, boolean persistent, Map<String, ?> props) {
        this(qualifiedNameMap, port, weight, priority, persistent, textFromProperties(props));
    }

    ServiceInfoImpl(Map<ServiceInfo.Fields, String> qualifiedNameMap, int port, int weight, int priority, boolean persistent, String text) {
        this(qualifiedNameMap, port, weight, priority, persistent, (byte[]) null);
        this._server = text;
        try {
            ByteArrayOutputStream out = new ByteArrayOutputStream(text.length());
            writeUTF(out, text);
            this._text = out.toByteArray();
        } catch (IOException e) {
            throw new RuntimeException("unexpected exception: " + e);
        }
    }

    ServiceInfoImpl(Map<ServiceInfo.Fields, String> qualifiedNameMap, int port, int weight, int priority, boolean persistent, byte[] text) {
        Map<ServiceInfo.Fields, String> map = checkQualifiedNameMap(qualifiedNameMap);
        this._domain = map.get(ServiceInfo.Fields.Domain);
        this._protocol = map.get(ServiceInfo.Fields.Protocol);
        this._application = map.get(ServiceInfo.Fields.Application);
        this._name = map.get(ServiceInfo.Fields.Instance);
        this._subtype = map.get(ServiceInfo.Fields.Subtype);
        this._port = port;
        this._weight = weight;
        this._priority = priority;
        this._text = text;
        setNeedTextAnnouncing(false);
        this._state = new ServiceInfoState(this);
        this._persistent = persistent;
        this._ipv4Addresses = Collections.synchronizedSet(new LinkedHashSet());
        this._ipv6Addresses = Collections.synchronizedSet(new LinkedHashSet());
    }

    ServiceInfoImpl(ServiceInfo info) {
        this._ipv4Addresses = Collections.synchronizedSet(new LinkedHashSet());
        this._ipv6Addresses = Collections.synchronizedSet(new LinkedHashSet());
        if (info != null) {
            this._domain = info.getDomain();
            this._protocol = info.getProtocol();
            this._application = info.getApplication();
            this._name = info.getName();
            this._subtype = info.getSubtype();
            this._port = info.getPort();
            this._weight = info.getWeight();
            this._priority = info.getPriority();
            this._text = info.getTextBytes();
            this._persistent = info.isPersistent();
            for (Inet6Address address : info.getInet6Addresses()) {
                this._ipv6Addresses.add(address);
            }
            for (Inet4Address address2 : info.getInet4Addresses()) {
                this._ipv4Addresses.add(address2);
            }
        }
        this._state = new ServiceInfoState(this);
    }

    public static Map<ServiceInfo.Fields, String> decodeQualifiedNameMap(String type, String name, String subtype) {
        Map<ServiceInfo.Fields, String> qualifiedNameMap = decodeQualifiedNameMapForType(type);
        qualifiedNameMap.put(ServiceInfo.Fields.Instance, name);
        qualifiedNameMap.put(ServiceInfo.Fields.Subtype, subtype);
        return checkQualifiedNameMap(qualifiedNameMap);
    }

    public static Map<ServiceInfo.Fields, String> decodeQualifiedNameMapForType(String type) {
        int index;
        String casePreservedType = type;
        String aType = type.toLowerCase();
        String application = aType;
        String protocol = "";
        String subtype = "";
        String name = "";
        String domain = "";
        if (aType.contains("in-addr.arpa") || aType.contains("ip6.arpa")) {
            int index2 = aType.indexOf(aType.contains("in-addr.arpa") ? "in-addr.arpa" : "ip6.arpa");
            name = removeSeparators(casePreservedType.substring(0, index2));
            domain = casePreservedType.substring(index2);
            application = "";
        } else if (aType.contains("_") || !aType.contains(".")) {
            if ((!aType.startsWith("_") || aType.startsWith("_services")) && (index = aType.indexOf(46)) > 0) {
                name = casePreservedType.substring(0, index);
                if (index + 1 < aType.length()) {
                    aType = aType.substring(index + 1);
                    casePreservedType = casePreservedType.substring(index + 1);
                }
            }
            int index3 = aType.lastIndexOf("._");
            if (index3 > 0) {
                int start = index3 + 2;
                protocol = casePreservedType.substring(start, aType.indexOf(46, start));
            }
            if (protocol.length() > 0) {
                int index4 = aType.indexOf("_" + protocol.toLowerCase() + ".");
                int start2 = protocol.length() + index4 + 2;
                int end = aType.length() - (aType.endsWith(".") ? 1 : 0);
                if (end > start2) {
                    domain = casePreservedType.substring(start2, end);
                }
                if (index4 > 0) {
                    application = casePreservedType.substring(0, index4 - 1);
                } else {
                    application = "";
                }
            }
            int index5 = application.toLowerCase().indexOf("._sub");
            if (index5 > 0) {
                subtype = removeSeparators(application.substring(0, index5));
                application = application.substring(index5 + 5);
            }
        } else {
            int index6 = aType.indexOf(46);
            name = removeSeparators(casePreservedType.substring(0, index6));
            domain = removeSeparators(casePreservedType.substring(index6));
            application = "";
        }
        Map<ServiceInfo.Fields, String> qualifiedNameMap = new HashMap<>(5);
        qualifiedNameMap.put(ServiceInfo.Fields.Domain, removeSeparators(domain));
        qualifiedNameMap.put(ServiceInfo.Fields.Protocol, protocol);
        qualifiedNameMap.put(ServiceInfo.Fields.Application, removeSeparators(application));
        qualifiedNameMap.put(ServiceInfo.Fields.Instance, name);
        qualifiedNameMap.put(ServiceInfo.Fields.Subtype, subtype);
        return qualifiedNameMap;
    }

    protected static Map<ServiceInfo.Fields, String> checkQualifiedNameMap(Map<ServiceInfo.Fields, String> qualifiedNameMap) {
        Map<ServiceInfo.Fields, String> checkedQualifiedNameMap = new HashMap<>(5);
        String domain = qualifiedNameMap.containsKey(ServiceInfo.Fields.Domain) ? qualifiedNameMap.get(ServiceInfo.Fields.Domain) : "local";
        if (domain == null || domain.length() == 0) {
            domain = "local";
        }
        checkedQualifiedNameMap.put(ServiceInfo.Fields.Domain, removeSeparators(domain));
        String protocol = qualifiedNameMap.containsKey(ServiceInfo.Fields.Protocol) ? qualifiedNameMap.get(ServiceInfo.Fields.Protocol) : "tcp";
        if (protocol == null || protocol.length() == 0) {
            protocol = "tcp";
        }
        checkedQualifiedNameMap.put(ServiceInfo.Fields.Protocol, removeSeparators(protocol));
        String application = qualifiedNameMap.containsKey(ServiceInfo.Fields.Application) ? qualifiedNameMap.get(ServiceInfo.Fields.Application) : "";
        if (application == null || application.length() == 0) {
            application = "";
        }
        checkedQualifiedNameMap.put(ServiceInfo.Fields.Application, removeSeparators(application));
        String instance = qualifiedNameMap.containsKey(ServiceInfo.Fields.Instance) ? qualifiedNameMap.get(ServiceInfo.Fields.Instance) : "";
        if (instance == null || instance.length() == 0) {
            instance = "";
        }
        checkedQualifiedNameMap.put(ServiceInfo.Fields.Instance, removeSeparators(instance));
        String subtype = qualifiedNameMap.containsKey(ServiceInfo.Fields.Subtype) ? qualifiedNameMap.get(ServiceInfo.Fields.Subtype) : "";
        if (subtype == null || subtype.length() == 0) {
            subtype = "";
        }
        checkedQualifiedNameMap.put(ServiceInfo.Fields.Subtype, removeSeparators(subtype));
        return checkedQualifiedNameMap;
    }

    private static String removeSeparators(String name) {
        if (name == null) {
            return "";
        }
        String newName = name.trim();
        if (newName.startsWith(".")) {
            newName = newName.substring(1);
        }
        if (newName.startsWith("_")) {
            newName = newName.substring(1);
        }
        if (newName.endsWith(".")) {
            return newName.substring(0, newName.length() - 1);
        }
        return newName;
    }

    public String getType() {
        String str;
        String str2;
        String domain = getDomain();
        String protocol = getProtocol();
        String application = getApplication();
        StringBuilder sb = new StringBuilder();
        if (application.length() > 0) {
            str = "_" + application + ".";
        } else {
            str = "";
        }
        sb.append(str);
        if (protocol.length() > 0) {
            str2 = "_" + protocol + ".";
        } else {
            str2 = "";
        }
        sb.append(str2);
        sb.append(domain);
        sb.append(".");
        return sb.toString();
    }

    public String getTypeWithSubtype() {
        String str;
        String subtype = getSubtype();
        StringBuilder sb = new StringBuilder();
        if (subtype.length() > 0) {
            str = "_" + subtype.toLowerCase() + "._sub.";
        } else {
            str = "";
        }
        sb.append(str);
        sb.append(getType());
        return sb.toString();
    }

    public String getName() {
        return this._name != null ? this._name : "";
    }

    public String getKey() {
        if (this._key == null) {
            this._key = getQualifiedName().toLowerCase();
        }
        return this._key;
    }

    /* access modifiers changed from: package-private */
    public void setName(String name) {
        this._name = name;
        this._key = null;
    }

    public String getQualifiedName() {
        String str;
        String str2;
        String str3;
        String domain = getDomain();
        String protocol = getProtocol();
        String application = getApplication();
        String instance = getName();
        StringBuilder sb = new StringBuilder();
        if (instance.length() > 0) {
            str = instance + ".";
        } else {
            str = "";
        }
        sb.append(str);
        if (application.length() > 0) {
            str2 = "_" + application + ".";
        } else {
            str2 = "";
        }
        sb.append(str2);
        if (protocol.length() > 0) {
            str3 = "_" + protocol + ".";
        } else {
            str3 = "";
        }
        sb.append(str3);
        sb.append(domain);
        sb.append(".");
        return sb.toString();
    }

    public String getServer() {
        return this._server != null ? this._server : "";
    }

    /* access modifiers changed from: package-private */
    public void setServer(String server) {
        this._server = server;
    }

    @Deprecated
    public String getHostAddress() {
        String[] names = getHostAddresses();
        return names.length > 0 ? names[0] : "";
    }

    public String[] getHostAddresses() {
        Inet4Address[] ip4Aaddresses = getInet4Addresses();
        Inet6Address[] ip6Aaddresses = getInet6Addresses();
        String[] names = new String[(ip4Aaddresses.length + ip6Aaddresses.length)];
        for (int i = 0; i < ip4Aaddresses.length; i++) {
            names[i] = ip4Aaddresses[i].getHostAddress();
        }
        for (int i2 = 0; i2 < ip6Aaddresses.length; i2++) {
            names[ip4Aaddresses.length + i2] = "[" + ip6Aaddresses[i2].getHostAddress() + "]";
        }
        return names;
    }

    /* access modifiers changed from: package-private */
    public void addAddress(Inet4Address addr) {
        this._ipv4Addresses.add(addr);
    }

    /* access modifiers changed from: package-private */
    public void addAddress(Inet6Address addr) {
        this._ipv6Addresses.add(addr);
    }

    @Deprecated
    public InetAddress getAddress() {
        return getInetAddress();
    }

    @Deprecated
    public InetAddress getInetAddress() {
        InetAddress[] addresses = getInetAddresses();
        if (addresses.length > 0) {
            return addresses[0];
        }
        return null;
    }

    @Deprecated
    public Inet4Address getInet4Address() {
        Inet4Address[] addresses = getInet4Addresses();
        if (addresses.length > 0) {
            return addresses[0];
        }
        return null;
    }

    @Deprecated
    public Inet6Address getInet6Address() {
        Inet6Address[] addresses = getInet6Addresses();
        if (addresses.length > 0) {
            return addresses[0];
        }
        return null;
    }

    public InetAddress[] getInetAddresses() {
        List<InetAddress> aList = new ArrayList<>(this._ipv4Addresses.size() + this._ipv6Addresses.size());
        aList.addAll(this._ipv4Addresses);
        aList.addAll(this._ipv6Addresses);
        return (InetAddress[]) aList.toArray(new InetAddress[aList.size()]);
    }

    public Inet4Address[] getInet4Addresses() {
        return (Inet4Address[]) this._ipv4Addresses.toArray(new Inet4Address[this._ipv4Addresses.size()]);
    }

    public Inet6Address[] getInet6Addresses() {
        return (Inet6Address[]) this._ipv6Addresses.toArray(new Inet6Address[this._ipv6Addresses.size()]);
    }

    public int getPort() {
        return this._port;
    }

    public int getPriority() {
        return this._priority;
    }

    public int getWeight() {
        return this._weight;
    }

    public byte[] getTextBytes() {
        return (this._text == null || this._text.length <= 0) ? DNSRecord.EMPTY_TXT : this._text;
    }

    @Deprecated
    public String getTextString() {
        Map<String, byte[]> properties = getProperties();
        Iterator<String> it = properties.keySet().iterator();
        if (!it.hasNext()) {
            return "";
        }
        String key = it.next();
        byte[] value = properties.get(key);
        if (value == null || value.length <= 0) {
            return key;
        }
        return key + "=" + new String(value);
    }

    @Deprecated
    public String getURL() {
        return getURL("http");
    }

    public String[] getURLs() {
        return getURLs("http");
    }

    @Deprecated
    public String getURL(String protocol) {
        String[] urls = getURLs(protocol);
        if (urls.length > 0) {
            return urls[0];
        }
        return protocol + "://null:" + getPort();
    }

    public String[] getURLs(String protocol) {
        String str;
        InetAddress[] addresses = getInetAddresses();
        String[] urls = new String[addresses.length];
        for (int i = 0; i < addresses.length; i++) {
            String url = protocol + "://" + addresses[i].getHostAddress() + ":" + getPort();
            String path = getPropertyString(Cookie2.PATH);
            if (path != null) {
                if (path.indexOf("://") >= 0) {
                    url = path;
                } else {
                    StringBuilder sb = new StringBuilder();
                    sb.append(url);
                    if (path.startsWith(CookieSpec.PATH_DELIM)) {
                        str = path;
                    } else {
                        str = CookieSpec.PATH_DELIM + path;
                    }
                    sb.append(str);
                    url = sb.toString();
                }
            }
            urls[i] = url;
        }
        return urls;
    }

    public synchronized byte[] getPropertyBytes(String name) {
        return getProperties().get(name);
    }

    public synchronized String getPropertyString(String name) {
        byte[] data = getProperties().get(name);
        if (data == null) {
            return null;
        }
        if (data == NO_VALUE) {
            return "true";
        }
        return readUTF(data, 0, data.length);
    }

    public Enumeration<String> getPropertyNames() {
        Map<String, byte[]> properties = getProperties();
        return new Vector(properties != null ? properties.keySet() : Collections.emptySet()).elements();
    }

    public String getApplication() {
        return this._application != null ? this._application : "";
    }

    public String getDomain() {
        return this._domain != null ? this._domain : "local";
    }

    public String getProtocol() {
        return this._protocol != null ? this._protocol : "tcp";
    }

    public String getSubtype() {
        return this._subtype != null ? this._subtype : "";
    }

    public Map<ServiceInfo.Fields, String> getQualifiedNameMap() {
        Map<ServiceInfo.Fields, String> map = new HashMap<>(5);
        map.put(ServiceInfo.Fields.Domain, getDomain());
        map.put(ServiceInfo.Fields.Protocol, getProtocol());
        map.put(ServiceInfo.Fields.Application, getApplication());
        map.put(ServiceInfo.Fields.Instance, getName());
        map.put(ServiceInfo.Fields.Subtype, getSubtype());
        return map;
    }

    static void writeUTF(OutputStream out, String str) throws IOException {
        int len = str.length();
        for (int i = 0; i < len; i++) {
            int c = str.charAt(i);
            if (c >= 1 && c <= 127) {
                out.write(c);
            } else if (c > 2047) {
                out.write(((c >> 12) & 15) | 224);
                out.write(((c >> 6) & 63) | 128);
                out.write(((c >> 0) & 63) | 128);
            } else {
                out.write(((c >> 6) & 31) | 192);
                out.write(((c >> 0) & 63) | 128);
            }
        }
    }

    /* access modifiers changed from: package-private */
    public String readUTF(byte[] data, int off, int len) {
        int offset;
        int offset2 = off;
        StringBuffer buf = new StringBuffer();
        int end = offset2 + len;
        while (offset2 < end) {
            int offset3 = offset2 + 1;
            int ch = data[offset2] & 255;
            int i = ch >> 4;
            switch (i) {
                case 0:
                case 1:
                case 2:
                case 3:
                case 4:
                case 5:
                case 6:
                case 7:
                    offset = offset3;
                    break;
                default:
                    switch (i) {
                        case 12:
                        case 13:
                            if (offset3 < len) {
                                offset = offset3 + 1;
                                ch = ((ch & 31) << 6) | (data[offset3] & 63);
                                break;
                            } else {
                                return null;
                            }
                        case 14:
                            if (offset3 + 2 < len) {
                                int offset4 = offset3 + 1;
                                ch = ((data[offset3] & 63) << 6) | ((ch & 15) << 12) | (data[offset4] & 63);
                                offset = offset4 + 1;
                                break;
                            } else {
                                return null;
                            }
                        default:
                            if (offset3 + 1 < len) {
                                offset = offset3 + 1;
                                ch = ((ch & 63) << 4) | (data[offset3] & 15);
                                break;
                            } else {
                                return null;
                            }
                    }
            }
            buf.append((char) ch);
            offset2 = offset;
        }
        return buf.toString();
    }

    /* access modifiers changed from: package-private */
    /* JADX WARNING: Code restructure failed: missing block: B:29:0x006f, code lost:
        r0.clear();
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public synchronized java.util.Map<java.lang.String, byte[]> getProperties() {
        /*
            r10 = this;
            monitor-enter(r10)
            java.util.Map<java.lang.String, byte[]> r0 = r10._props     // Catch:{ all -> 0x008d }
            if (r0 != 0) goto L_0x0080
            byte[] r0 = r10.getTextBytes()     // Catch:{ all -> 0x008d }
            if (r0 == 0) goto L_0x0080
            java.util.Hashtable r0 = new java.util.Hashtable     // Catch:{ all -> 0x008d }
            r0.<init>()     // Catch:{ all -> 0x008d }
            r1 = 0
            r2 = 0
        L_0x0012:
            byte[] r3 = r10.getTextBytes()     // Catch:{ Exception -> 0x0074 }
            int r3 = r3.length     // Catch:{ Exception -> 0x0074 }
            if (r2 >= r3) goto L_0x0073
            byte[] r3 = r10.getTextBytes()     // Catch:{ Exception -> 0x0074 }
            int r4 = r2 + 1
            byte r2 = r3[r2]     // Catch:{ Exception -> 0x0074 }
            r2 = r2 & 255(0xff, float:3.57E-43)
            if (r2 == 0) goto L_0x006f
            int r3 = r4 + r2
            byte[] r5 = r10.getTextBytes()     // Catch:{ Exception -> 0x0074 }
            int r5 = r5.length     // Catch:{ Exception -> 0x0074 }
            if (r3 <= r5) goto L_0x002f
            goto L_0x006f
        L_0x002f:
            r3 = 0
        L_0x0030:
            if (r3 >= r2) goto L_0x0041
            byte[] r5 = r10.getTextBytes()     // Catch:{ Exception -> 0x0074 }
            int r6 = r4 + r3
            byte r5 = r5[r6]     // Catch:{ Exception -> 0x0074 }
            r6 = 61
            if (r5 == r6) goto L_0x0041
            int r3 = r3 + 1
            goto L_0x0030
        L_0x0041:
            byte[] r5 = r10.getTextBytes()     // Catch:{ Exception -> 0x0074 }
            java.lang.String r5 = r10.readUTF(r5, r4, r3)     // Catch:{ Exception -> 0x0074 }
            if (r5 != 0) goto L_0x004f
            r0.clear()     // Catch:{ Exception -> 0x0074 }
            goto L_0x0073
        L_0x004f:
            if (r3 != r2) goto L_0x0058
            byte[] r6 = NO_VALUE     // Catch:{ Exception -> 0x0074 }
            r0.put(r5, r6)     // Catch:{ Exception -> 0x0074 }
        L_0x0056:
            r2 = r4
            goto L_0x006e
        L_0x0058:
            int r3 = r3 + 1
            int r6 = r2 - r3
            byte[] r6 = new byte[r6]     // Catch:{ Exception -> 0x0074 }
            byte[] r7 = r10.getTextBytes()     // Catch:{ Exception -> 0x0074 }
            int r8 = r4 + r3
            int r9 = r2 - r3
            java.lang.System.arraycopy(r7, r8, r6, r1, r9)     // Catch:{ Exception -> 0x0074 }
            r0.put(r5, r6)     // Catch:{ Exception -> 0x0074 }
            int r4 = r4 + r2
            goto L_0x0056
        L_0x006e:
            goto L_0x0012
        L_0x006f:
            r0.clear()     // Catch:{ Exception -> 0x0074 }
        L_0x0073:
            goto L_0x007e
        L_0x0074:
            r1 = move-exception
            java.util.logging.Logger r2 = logger     // Catch:{ all -> 0x008d }
            java.util.logging.Level r3 = java.util.logging.Level.WARNING     // Catch:{ all -> 0x008d }
            java.lang.String r4 = "Malformed TXT Field "
            r2.log(r3, r4, r1)     // Catch:{ all -> 0x008d }
        L_0x007e:
            r10._props = r0     // Catch:{ all -> 0x008d }
        L_0x0080:
            java.util.Map<java.lang.String, byte[]> r0 = r10._props     // Catch:{ all -> 0x008d }
            if (r0 == 0) goto L_0x0087
            java.util.Map<java.lang.String, byte[]> r0 = r10._props     // Catch:{ all -> 0x008d }
            goto L_0x008b
        L_0x0087:
            java.util.Map r0 = java.util.Collections.emptyMap()     // Catch:{ all -> 0x008d }
        L_0x008b:
            monitor-exit(r10)
            return r0
        L_0x008d:
            r0 = move-exception
            monitor-exit(r10)
            throw r0
        */
        throw new UnsupportedOperationException("Method not decompiled: javax.jmdns.impl.ServiceInfoImpl.getProperties():java.util.Map");
    }

    public void updateRecord(DNSCache dnsCache, long now, DNSEntry rec) {
        JmDNSImpl dns;
        if ((rec instanceof DNSRecord) && !rec.isExpired(now)) {
            boolean serviceUpdated = false;
            switch (rec.getRecordType()) {
                case TYPE_A:
                    if (rec.getName().equalsIgnoreCase(getServer())) {
                        this._ipv4Addresses.add((Inet4Address) ((DNSRecord.Address) rec).getAddress());
                        serviceUpdated = true;
                        break;
                    }
                    break;
                case TYPE_AAAA:
                    if (rec.getName().equalsIgnoreCase(getServer())) {
                        this._ipv6Addresses.add((Inet6Address) ((DNSRecord.Address) rec).getAddress());
                        serviceUpdated = true;
                        break;
                    }
                    break;
                case TYPE_SRV:
                    if (rec.getName().equalsIgnoreCase(getQualifiedName())) {
                        DNSRecord.Service srv = (DNSRecord.Service) rec;
                        boolean serverChanged = this._server == null || !this._server.equalsIgnoreCase(srv.getServer());
                        this._server = srv.getServer();
                        this._port = srv.getPort();
                        this._weight = srv.getWeight();
                        this._priority = srv.getPriority();
                        if (!serverChanged) {
                            serviceUpdated = true;
                            break;
                        } else {
                            this._ipv4Addresses.clear();
                            this._ipv6Addresses.clear();
                            for (DNSEntry entry : dnsCache.getDNSEntryList(this._server, DNSRecordType.TYPE_A, DNSRecordClass.CLASS_IN)) {
                                updateRecord(dnsCache, now, entry);
                            }
                            for (DNSEntry entry2 : dnsCache.getDNSEntryList(this._server, DNSRecordType.TYPE_AAAA, DNSRecordClass.CLASS_IN)) {
                                updateRecord(dnsCache, now, entry2);
                            }
                            break;
                        }
                    }
                    break;
                case TYPE_TXT:
                    if (rec.getName().equalsIgnoreCase(getQualifiedName())) {
                        this._text = ((DNSRecord.Text) rec).getText();
                        this._props = null;
                        serviceUpdated = true;
                        break;
                    }
                    break;
                case TYPE_PTR:
                    if (getSubtype().length() == 0 && rec.getSubtype().length() != 0) {
                        this._subtype = rec.getSubtype();
                        serviceUpdated = true;
                        break;
                    }
            }
            if (serviceUpdated && hasData() && (dns = getDns()) != null) {
                ServiceEvent event = ((DNSRecord) rec).getServiceEvent(dns);
                dns.handleServiceResolved(new ServiceEventImpl(dns, event.getType(), event.getName(), this));
            }
            synchronized (this) {
                notifyAll();
            }
        }
    }

    public synchronized boolean hasData() {
        return getServer() != null && hasInetAddress() && getTextBytes() != null && getTextBytes().length > 0;
    }

    private final boolean hasInetAddress() {
        return this._ipv4Addresses.size() > 0 || this._ipv6Addresses.size() > 0;
    }

    public boolean advanceState(DNSTask task) {
        return this._state.advanceState(task);
    }

    public boolean revertState() {
        return this._state.revertState();
    }

    public boolean cancelState() {
        return this._state.cancelState();
    }

    public boolean closeState() {
        return this._state.closeState();
    }

    public boolean recoverState() {
        return this._state.recoverState();
    }

    public void removeAssociationWithTask(DNSTask task) {
        this._state.removeAssociationWithTask(task);
    }

    public void associateWithTask(DNSTask task, DNSState state) {
        this._state.associateWithTask(task, state);
    }

    public boolean isAssociatedWithTask(DNSTask task, DNSState state) {
        return this._state.isAssociatedWithTask(task, state);
    }

    public boolean isProbing() {
        return this._state.isProbing();
    }

    public boolean isAnnouncing() {
        return this._state.isAnnouncing();
    }

    public boolean isAnnounced() {
        return this._state.isAnnounced();
    }

    public boolean isCanceling() {
        return this._state.isCanceling();
    }

    public boolean isCanceled() {
        return this._state.isCanceled();
    }

    public boolean isClosing() {
        return this._state.isClosing();
    }

    public boolean isClosed() {
        return this._state.isClosed();
    }

    public boolean waitForAnnounced(long timeout) {
        return this._state.waitForAnnounced(timeout);
    }

    public boolean waitForCanceled(long timeout) {
        return this._state.waitForCanceled(timeout);
    }

    public int hashCode() {
        return getQualifiedName().hashCode();
    }

    public boolean equals(Object obj) {
        return (obj instanceof ServiceInfoImpl) && getQualifiedName().equals(((ServiceInfoImpl) obj).getQualifiedName());
    }

    public String getNiceTextString() {
        StringBuffer buf = new StringBuffer();
        int i = 0;
        int len = getTextBytes().length;
        while (true) {
            if (i >= len) {
                break;
            } else if (i >= 200) {
                buf.append("...");
                break;
            } else {
                int ch = getTextBytes()[i] & 255;
                if (ch < 32 || ch > 127) {
                    buf.append("\\0");
                    buf.append(Integer.toString(ch, 8));
                } else {
                    buf.append((char) ch);
                }
                i++;
            }
        }
        return buf.toString();
    }

    public ServiceInfoImpl clone() {
        ServiceInfoImpl serviceInfo = new ServiceInfoImpl(getQualifiedNameMap(), this._port, this._weight, this._priority, this._persistent, this._text);
        for (Inet6Address address : getInet6Addresses()) {
            serviceInfo._ipv6Addresses.add(address);
        }
        for (Inet4Address address2 : getInet4Addresses()) {
            serviceInfo._ipv4Addresses.add(address2);
        }
        return serviceInfo;
    }

    public String toString() {
        String str;
        StringBuilder buf = new StringBuilder();
        buf.append("[" + getClass().getSimpleName() + "@" + System.identityHashCode(this) + " ");
        buf.append("name: '");
        StringBuilder sb = new StringBuilder();
        if (getName().length() > 0) {
            str = getName() + ".";
        } else {
            str = "";
        }
        sb.append(str);
        sb.append(getTypeWithSubtype());
        buf.append(sb.toString());
        buf.append("' address: '");
        InetAddress[] addresses = getInetAddresses();
        if (addresses.length > 0) {
            for (InetAddress address : addresses) {
                buf.append(address);
                buf.append(':');
                buf.append(getPort());
                buf.append(' ');
            }
        } else {
            buf.append("(null):");
            buf.append(getPort());
        }
        buf.append("' status: '");
        buf.append(this._state.toString());
        buf.append(isPersistent() ? "' is persistent," : "',");
        buf.append(" has ");
        buf.append(hasData() ? "" : "NO ");
        buf.append(ObjectArraySerializer.DATA_TAG);
        if (getTextBytes().length > 0) {
            Map<String, byte[]> properties = getProperties();
            if (!properties.isEmpty()) {
                buf.append("\n");
                for (String key : properties.keySet()) {
                    buf.append("\t" + key + ": " + new String(properties.get(key)) + "\n");
                }
            } else {
                buf.append(" empty");
            }
        }
        buf.append(']');
        return buf.toString();
    }

    public Collection<DNSRecord> answers(boolean unique, int ttl, HostInfo localHost) {
        List<DNSRecord> list = new ArrayList<>();
        if (getSubtype().length() > 0) {
            list.add(new DNSRecord.Pointer(getTypeWithSubtype(), DNSRecordClass.CLASS_IN, false, ttl, getQualifiedName()));
        }
        list.add(new DNSRecord.Pointer(getType(), DNSRecordClass.CLASS_IN, false, ttl, getQualifiedName()));
        String qualifiedName = getQualifiedName();
        DNSRecordClass dNSRecordClass = DNSRecordClass.CLASS_IN;
        int i = this._priority;
        int i2 = this._weight;
        list.add(new DNSRecord.Service(qualifiedName, dNSRecordClass, unique, ttl, i, i2, this._port, localHost.getName()));
        list.add(new DNSRecord.Text(getQualifiedName(), DNSRecordClass.CLASS_IN, unique, ttl, getTextBytes()));
        return list;
    }

    public void setText(byte[] text) throws IllegalStateException {
        synchronized (this) {
            this._text = text;
            this._props = null;
            setNeedTextAnnouncing(true);
        }
    }

    public void setText(Map<String, ?> props) throws IllegalStateException {
        setText(textFromProperties(props));
    }

    /* access modifiers changed from: package-private */
    public void _setText(byte[] text) {
        this._text = text;
        this._props = null;
    }

    private static byte[] textFromProperties(Map<String, ?> props) {
        String str;
        byte[] text = null;
        if (props != null) {
            try {
                ByteArrayOutputStream out = new ByteArrayOutputStream(256);
                for (String key : props.keySet()) {
                    Object val = props.get(key);
                    ByteArrayOutputStream out2 = new ByteArrayOutputStream(100);
                    writeUTF(out2, key);
                    if (val != null) {
                        if (val instanceof String) {
                            out2.write(61);
                            writeUTF(out2, (String) val);
                        } else if (val instanceof byte[]) {
                            byte[] bval = (byte[]) val;
                            if (bval.length > 0) {
                                out2.write(61);
                                out2.write(bval, 0, bval.length);
                            } else {
                                val = null;
                            }
                        } else {
                            throw new IllegalArgumentException("invalid property value: " + val);
                        }
                    }
                    byte[] data = out2.toByteArray();
                    if (data.length > 255) {
                        StringBuilder sb = new StringBuilder();
                        sb.append("Cannot have individual values larger that 255 chars. Offending value: ");
                        sb.append(key);
                        if (val != null) {
                            str = "";
                        } else {
                            str = "=" + val;
                        }
                        sb.append(str);
                        throw new IOException(sb.toString());
                    }
                    out.write((byte) data.length);
                    out.write(data, 0, data.length);
                }
                text = out.toByteArray();
            } catch (IOException e) {
                throw new RuntimeException("unexpected exception: " + e);
            }
        }
        return (text == null || text.length <= 0) ? DNSRecord.EMPTY_TXT : text;
    }

    public void setDns(JmDNSImpl dns) {
        this._state.setDns(dns);
    }

    public JmDNSImpl getDns() {
        return this._state.getDns();
    }

    public boolean isPersistent() {
        return this._persistent;
    }

    public void setNeedTextAnnouncing(boolean needTextAnnouncing) {
        this._needTextAnnouncing = needTextAnnouncing;
        if (this._needTextAnnouncing) {
            this._state.setTask((DNSTask) null);
        }
    }

    public boolean needTextAnnouncing() {
        return this._needTextAnnouncing;
    }

    /* access modifiers changed from: package-private */
    public Delegate getDelegate() {
        return this._delegate;
    }

    /* access modifiers changed from: package-private */
    public void setDelegate(Delegate delegate) {
        this._delegate = delegate;
    }
}
