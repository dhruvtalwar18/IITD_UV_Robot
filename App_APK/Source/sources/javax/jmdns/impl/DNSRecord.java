package javax.jmdns.impl;

import java.io.DataOutputStream;
import java.io.IOException;
import java.io.UnsupportedEncodingException;
import java.net.Inet4Address;
import java.net.Inet6Address;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.jmdns.ServiceEvent;
import javax.jmdns.ServiceInfo;
import javax.jmdns.impl.DNSOutgoing;
import javax.jmdns.impl.constants.DNSConstants;
import javax.jmdns.impl.constants.DNSRecordClass;
import javax.jmdns.impl.constants.DNSRecordType;
import org.apache.commons.httpclient.cookie.CookieSpec;

public abstract class DNSRecord extends DNSEntry {
    public static final byte[] EMPTY_TXT = {0};
    private static Logger logger = Logger.getLogger(DNSRecord.class.getName());
    private long _created = System.currentTimeMillis();
    private InetAddress _source;
    private int _ttl;

    /* access modifiers changed from: package-private */
    public abstract DNSOutgoing addAnswer(JmDNSImpl jmDNSImpl, DNSIncoming dNSIncoming, InetAddress inetAddress, int i, DNSOutgoing dNSOutgoing) throws IOException;

    public abstract ServiceEvent getServiceEvent(JmDNSImpl jmDNSImpl);

    public abstract ServiceInfo getServiceInfo(boolean z);

    /* access modifiers changed from: package-private */
    public abstract boolean handleQuery(JmDNSImpl jmDNSImpl, long j);

    /* access modifiers changed from: package-private */
    public abstract boolean handleResponse(JmDNSImpl jmDNSImpl);

    public abstract boolean isSingleValued();

    /* access modifiers changed from: package-private */
    public abstract boolean sameValue(DNSRecord dNSRecord);

    /* access modifiers changed from: package-private */
    public abstract void write(DNSOutgoing.MessageOutputStream messageOutputStream);

    DNSRecord(String name, DNSRecordType type, DNSRecordClass recordClass, boolean unique, int ttl) {
        super(name, type, recordClass, unique);
        this._ttl = ttl;
    }

    public boolean equals(Object other) {
        return (other instanceof DNSRecord) && super.equals(other) && sameValue((DNSRecord) other);
    }

    /* access modifiers changed from: package-private */
    public boolean sameType(DNSRecord other) {
        return getRecordType() == other.getRecordType();
    }

    /* access modifiers changed from: package-private */
    public boolean suppressedBy(DNSIncoming msg) {
        try {
            for (DNSRecord answer : msg.getAllAnswers()) {
                if (suppressedBy(answer)) {
                    return true;
                }
            }
            return false;
        } catch (ArrayIndexOutOfBoundsException e) {
            Logger logger2 = logger;
            Level level = Level.WARNING;
            logger2.log(level, "suppressedBy() message " + msg + " exception ", e);
            return false;
        }
    }

    /* access modifiers changed from: package-private */
    public boolean suppressedBy(DNSRecord other) {
        if (!equals(other) || other._ttl <= this._ttl / 2) {
            return false;
        }
        return true;
    }

    /* access modifiers changed from: package-private */
    public long getExpirationTime(int percent) {
        return this._created + (((long) (this._ttl * percent)) * 10);
    }

    /* access modifiers changed from: package-private */
    public int getRemainingTTL(long now) {
        return (int) Math.max(0, (getExpirationTime(100) - now) / 1000);
    }

    public boolean isExpired(long now) {
        return getExpirationTime(100) <= now;
    }

    public boolean isStale(long now) {
        return getExpirationTime(50) <= now;
    }

    /* access modifiers changed from: package-private */
    public void resetTTL(DNSRecord other) {
        this._created = other._created;
        this._ttl = other._ttl;
    }

    /* access modifiers changed from: package-private */
    public void setWillExpireSoon(long now) {
        this._created = now;
        this._ttl = 1;
    }

    public static class IPv4Address extends Address {
        IPv4Address(String name, DNSRecordClass recordClass, boolean unique, int ttl, InetAddress addr) {
            super(name, DNSRecordType.TYPE_A, recordClass, unique, ttl, addr);
        }

        IPv4Address(String name, DNSRecordClass recordClass, boolean unique, int ttl, byte[] rawAddress) {
            super(name, DNSRecordType.TYPE_A, recordClass, unique, ttl, rawAddress);
        }

        /* access modifiers changed from: package-private */
        public void write(DNSOutgoing.MessageOutputStream out) {
            if (this._addr != null) {
                byte[] buffer = this._addr.getAddress();
                if (!(this._addr instanceof Inet4Address)) {
                    byte[] tempbuffer = buffer;
                    buffer = new byte[4];
                    System.arraycopy(tempbuffer, 12, buffer, 0, 4);
                }
                out.writeBytes(buffer, 0, buffer.length);
            }
        }

        public ServiceInfo getServiceInfo(boolean persistent) {
            ServiceInfoImpl info = (ServiceInfoImpl) super.getServiceInfo(persistent);
            info.addAddress((Inet4Address) this._addr);
            return info;
        }
    }

    public static class IPv6Address extends Address {
        IPv6Address(String name, DNSRecordClass recordClass, boolean unique, int ttl, InetAddress addr) {
            super(name, DNSRecordType.TYPE_AAAA, recordClass, unique, ttl, addr);
        }

        IPv6Address(String name, DNSRecordClass recordClass, boolean unique, int ttl, byte[] rawAddress) {
            super(name, DNSRecordType.TYPE_AAAA, recordClass, unique, ttl, rawAddress);
        }

        /* access modifiers changed from: package-private */
        public void write(DNSOutgoing.MessageOutputStream out) {
            if (this._addr != null) {
                byte[] buffer = this._addr.getAddress();
                if (this._addr instanceof Inet4Address) {
                    byte[] tempbuffer = buffer;
                    buffer = new byte[16];
                    for (int i = 0; i < 16; i++) {
                        if (i < 11) {
                            buffer[i] = tempbuffer[i - 12];
                        } else {
                            buffer[i] = 0;
                        }
                    }
                }
                out.writeBytes(buffer, 0, buffer.length);
            }
        }

        public ServiceInfo getServiceInfo(boolean persistent) {
            ServiceInfoImpl info = (ServiceInfoImpl) super.getServiceInfo(persistent);
            info.addAddress((Inet6Address) this._addr);
            return info;
        }
    }

    public static abstract class Address extends DNSRecord {
        private static Logger logger1 = Logger.getLogger(Address.class.getName());
        InetAddress _addr;

        protected Address(String name, DNSRecordType type, DNSRecordClass recordClass, boolean unique, int ttl, InetAddress addr) {
            super(name, type, recordClass, unique, ttl);
            this._addr = addr;
        }

        protected Address(String name, DNSRecordType type, DNSRecordClass recordClass, boolean unique, int ttl, byte[] rawAddress) {
            super(name, type, recordClass, unique, ttl);
            try {
                this._addr = InetAddress.getByAddress(rawAddress);
            } catch (UnknownHostException exception) {
                logger1.log(Level.WARNING, "Address() exception ", exception);
            }
        }

        /* access modifiers changed from: package-private */
        public boolean same(DNSRecord other) {
            if ((other instanceof Address) && sameName(other) && sameValue(other)) {
                return true;
            }
            return false;
        }

        /* access modifiers changed from: package-private */
        public boolean sameName(DNSRecord other) {
            return getName().equalsIgnoreCase(other.getName());
        }

        /* access modifiers changed from: package-private */
        public boolean sameValue(DNSRecord other) {
            if (!(other instanceof Address)) {
                return false;
            }
            Address address = (Address) other;
            if (getAddress() != null || address.getAddress() == null) {
                return getAddress().equals(address.getAddress());
            }
            return false;
        }

        public boolean isSingleValued() {
            return false;
        }

        /* access modifiers changed from: package-private */
        public InetAddress getAddress() {
            return this._addr;
        }

        /* access modifiers changed from: protected */
        public void toByteArray(DataOutputStream dout) throws IOException {
            DNSRecord.super.toByteArray(dout);
            byte[] buffer = getAddress().getAddress();
            for (byte writeByte : buffer) {
                dout.writeByte(writeByte);
            }
        }

        /* access modifiers changed from: package-private */
        public boolean handleQuery(JmDNSImpl dns, long expirationTime) {
            if (!dns.getLocalHost().conflictWithRecord(this)) {
                return false;
            }
            int comparison = compareTo(dns.getLocalHost().getDNSAddressRecord(getRecordType(), isUnique(), DNSConstants.DNS_TTL));
            if (comparison == 0) {
                logger1.finer("handleQuery() Ignoring an identical address query");
                return false;
            }
            logger1.finer("handleQuery() Conflicting query detected.");
            if (dns.isProbing() && comparison > 0) {
                dns.getLocalHost().incrementHostName();
                dns.getCache().clear();
                Iterator<ServiceInfo> it = dns.getServices().values().iterator();
                while (it.hasNext()) {
                    ((ServiceInfoImpl) it.next()).revertState();
                }
            }
            dns.revertState();
            return true;
        }

        /* access modifiers changed from: package-private */
        public boolean handleResponse(JmDNSImpl dns) {
            if (!dns.getLocalHost().conflictWithRecord(this)) {
                return false;
            }
            logger1.finer("handleResponse() Denial detected");
            if (dns.isProbing()) {
                dns.getLocalHost().incrementHostName();
                dns.getCache().clear();
                Iterator<ServiceInfo> it = dns.getServices().values().iterator();
                while (it.hasNext()) {
                    ((ServiceInfoImpl) it.next()).revertState();
                }
            }
            dns.revertState();
            return true;
        }

        /* access modifiers changed from: package-private */
        public DNSOutgoing addAnswer(JmDNSImpl dns, DNSIncoming in, InetAddress addr, int port, DNSOutgoing out) throws IOException {
            return out;
        }

        public ServiceInfo getServiceInfo(boolean persistent) {
            return new ServiceInfoImpl(getQualifiedNameMap(), 0, 0, 0, persistent, (byte[]) null);
        }

        public ServiceEvent getServiceEvent(JmDNSImpl dns) {
            ServiceInfo info = getServiceInfo(false);
            ((ServiceInfoImpl) info).setDns(dns);
            return new ServiceEventImpl(dns, info.getType(), info.getName(), info);
        }

        /* access modifiers changed from: protected */
        public void toString(StringBuilder aLog) {
            DNSRecord.super.toString(aLog);
            StringBuilder sb = new StringBuilder();
            sb.append(" address: '");
            sb.append(getAddress() != null ? getAddress().getHostAddress() : "null");
            sb.append("'");
            aLog.append(sb.toString());
        }
    }

    public static class Pointer extends DNSRecord {
        private final String _alias;

        public Pointer(String name, DNSRecordClass recordClass, boolean unique, int ttl, String alias) {
            super(name, DNSRecordType.TYPE_PTR, recordClass, unique, ttl);
            this._alias = alias;
        }

        public boolean isSameEntry(DNSEntry entry) {
            return DNSRecord.super.isSameEntry(entry) && (entry instanceof Pointer) && sameValue((Pointer) entry);
        }

        /* access modifiers changed from: package-private */
        public void write(DNSOutgoing.MessageOutputStream out) {
            out.writeName(this._alias);
        }

        /* access modifiers changed from: package-private */
        public boolean sameValue(DNSRecord other) {
            if (!(other instanceof Pointer)) {
                return false;
            }
            Pointer pointer = (Pointer) other;
            if (this._alias != null || pointer._alias == null) {
                return this._alias.equals(pointer._alias);
            }
            return false;
        }

        public boolean isSingleValued() {
            return false;
        }

        /* access modifiers changed from: package-private */
        public boolean handleQuery(JmDNSImpl dns, long expirationTime) {
            return false;
        }

        /* access modifiers changed from: package-private */
        public boolean handleResponse(JmDNSImpl dns) {
            return false;
        }

        /* access modifiers changed from: package-private */
        public String getAlias() {
            return this._alias;
        }

        /* access modifiers changed from: package-private */
        public DNSOutgoing addAnswer(JmDNSImpl dns, DNSIncoming in, InetAddress addr, int port, DNSOutgoing out) throws IOException {
            return out;
        }

        public ServiceInfo getServiceInfo(boolean persistent) {
            if (isServicesDiscoveryMetaQuery()) {
                return new ServiceInfoImpl(ServiceInfoImpl.decodeQualifiedNameMapForType(getAlias()), 0, 0, 0, persistent, (byte[]) null);
            } else if (isReverseLookup()) {
                return new ServiceInfoImpl(getQualifiedNameMap(), 0, 0, 0, persistent, (byte[]) null);
            } else if (isDomainDiscoveryQuery()) {
                return new ServiceInfoImpl(getQualifiedNameMap(), 0, 0, 0, persistent, (byte[]) null);
            } else {
                Map<ServiceInfo.Fields, String> map = ServiceInfoImpl.decodeQualifiedNameMapForType(getAlias());
                map.put(ServiceInfo.Fields.Subtype, getQualifiedNameMap().get(ServiceInfo.Fields.Subtype));
                return new ServiceInfoImpl(map, 0, 0, 0, persistent, getAlias());
            }
        }

        public ServiceEvent getServiceEvent(JmDNSImpl dns) {
            ServiceInfo info = getServiceInfo(false);
            ((ServiceInfoImpl) info).setDns(dns);
            String domainName = info.getType();
            return new ServiceEventImpl(dns, domainName, JmDNSImpl.toUnqualifiedName(domainName, getAlias()), info);
        }

        /* access modifiers changed from: protected */
        public void toString(StringBuilder aLog) {
            DNSRecord.super.toString(aLog);
            StringBuilder sb = new StringBuilder();
            sb.append(" alias: '");
            sb.append(this._alias != null ? this._alias.toString() : "null");
            sb.append("'");
            aLog.append(sb.toString());
        }
    }

    public static class Text extends DNSRecord {
        private final byte[] _text;

        public Text(String name, DNSRecordClass recordClass, boolean unique, int ttl, byte[] text) {
            super(name, DNSRecordType.TYPE_TXT, recordClass, unique, ttl);
            this._text = (text == null || text.length <= 0) ? EMPTY_TXT : text;
        }

        /* access modifiers changed from: package-private */
        public byte[] getText() {
            return this._text;
        }

        /* access modifiers changed from: package-private */
        public void write(DNSOutgoing.MessageOutputStream out) {
            out.writeBytes(this._text, 0, this._text.length);
        }

        /* access modifiers changed from: package-private */
        public boolean sameValue(DNSRecord other) {
            if (!(other instanceof Text)) {
                return false;
            }
            Text txt = (Text) other;
            if ((this._text == null && txt._text != null) || txt._text.length != this._text.length) {
                return false;
            }
            int i = this._text.length;
            while (true) {
                int i2 = i - 1;
                if (i <= 0) {
                    return true;
                }
                if (txt._text[i2] != this._text[i2]) {
                    return false;
                }
                i = i2;
            }
        }

        public boolean isSingleValued() {
            return true;
        }

        /* access modifiers changed from: package-private */
        public boolean handleQuery(JmDNSImpl dns, long expirationTime) {
            return false;
        }

        /* access modifiers changed from: package-private */
        public boolean handleResponse(JmDNSImpl dns) {
            return false;
        }

        /* access modifiers changed from: package-private */
        public DNSOutgoing addAnswer(JmDNSImpl dns, DNSIncoming in, InetAddress addr, int port, DNSOutgoing out) throws IOException {
            return out;
        }

        public ServiceInfo getServiceInfo(boolean persistent) {
            return new ServiceInfoImpl(getQualifiedNameMap(), 0, 0, 0, persistent, this._text);
        }

        public ServiceEvent getServiceEvent(JmDNSImpl dns) {
            ServiceInfo info = getServiceInfo(false);
            ((ServiceInfoImpl) info).setDns(dns);
            return new ServiceEventImpl(dns, info.getType(), info.getName(), info);
        }

        /* access modifiers changed from: protected */
        public void toString(StringBuilder aLog) {
            String str;
            DNSRecord.super.toString(aLog);
            StringBuilder sb = new StringBuilder();
            sb.append(" text: '");
            if (this._text.length > 20) {
                str = new String(this._text, 0, 17) + "...";
            } else {
                str = new String(this._text);
            }
            sb.append(str);
            sb.append("'");
            aLog.append(sb.toString());
        }
    }

    public static class Service extends DNSRecord {
        private static Logger logger1 = Logger.getLogger(Service.class.getName());
        private final int _port;
        private final int _priority;
        private final String _server;
        private final int _weight;

        public Service(String name, DNSRecordClass recordClass, boolean unique, int ttl, int priority, int weight, int port, String server) {
            super(name, DNSRecordType.TYPE_SRV, recordClass, unique, ttl);
            this._priority = priority;
            this._weight = weight;
            this._port = port;
            this._server = server;
        }

        /* access modifiers changed from: package-private */
        public void write(DNSOutgoing.MessageOutputStream out) {
            out.writeShort(this._priority);
            out.writeShort(this._weight);
            out.writeShort(this._port);
            if (DNSIncoming.USE_DOMAIN_NAME_FORMAT_FOR_SRV_TARGET) {
                out.writeName(this._server);
                return;
            }
            out.writeUTF(this._server, 0, this._server.length());
            out.writeByte(0);
        }

        /* access modifiers changed from: protected */
        public void toByteArray(DataOutputStream dout) throws IOException {
            DNSRecord.super.toByteArray(dout);
            dout.writeShort(this._priority);
            dout.writeShort(this._weight);
            dout.writeShort(this._port);
            try {
                dout.write(this._server.getBytes("UTF-8"));
            } catch (UnsupportedEncodingException e) {
            }
        }

        /* access modifiers changed from: package-private */
        public String getServer() {
            return this._server;
        }

        public int getPriority() {
            return this._priority;
        }

        public int getWeight() {
            return this._weight;
        }

        public int getPort() {
            return this._port;
        }

        /* access modifiers changed from: package-private */
        public boolean sameValue(DNSRecord other) {
            if (!(other instanceof Service)) {
                return false;
            }
            Service s = (Service) other;
            if (this._priority == s._priority && this._weight == s._weight && this._port == s._port && this._server.equals(s._server)) {
                return true;
            }
            return false;
        }

        public boolean isSingleValued() {
            return true;
        }

        /* access modifiers changed from: package-private */
        public boolean handleQuery(JmDNSImpl dns, long expirationTime) {
            ServiceInfoImpl info = (ServiceInfoImpl) dns.getServices().get(getKey());
            if (info == null || ((!info.isAnnouncing() && !info.isAnnounced()) || (this._port == info.getPort() && this._server.equalsIgnoreCase(dns.getLocalHost().getName())))) {
                JmDNSImpl jmDNSImpl = dns;
                return false;
            }
            Logger logger = logger1;
            logger.finer("handleQuery() Conflicting probe detected from: " + getRecordSource());
            Service localService = new Service(info.getQualifiedName(), DNSRecordClass.CLASS_IN, true, DNSConstants.DNS_TTL, info.getPriority(), info.getWeight(), info.getPort(), dns.getLocalHost().getName());
            try {
                if (dns.getInetAddress().equals(getRecordSource())) {
                    Logger logger2 = logger1;
                    logger2.warning("Got conflicting probe from ourselves\nincoming: " + toString() + "\nlocal   : " + localService.toString());
                }
            } catch (IOException e) {
                logger1.log(Level.WARNING, "IOException", e);
            }
            int comparison = compareTo(localService);
            if (comparison == 0) {
                logger1.finer("handleQuery() Ignoring a identical service query");
                return false;
            } else if (!info.isProbing() || comparison <= 0) {
                JmDNSImpl jmDNSImpl2 = dns;
                return false;
            } else {
                String oldName = info.getQualifiedName().toLowerCase();
                JmDNSImpl jmDNSImpl3 = dns;
                info.setName(dns.incrementName(info.getName()));
                dns.getServices().remove(oldName);
                dns.getServices().put(info.getQualifiedName().toLowerCase(), info);
                Logger logger3 = logger1;
                logger3.finer("handleQuery() Lost tie break: new unique name chosen:" + info.getName());
                info.revertState();
                return true;
            }
        }

        /* access modifiers changed from: package-private */
        public boolean handleResponse(JmDNSImpl dns) {
            ServiceInfoImpl info = (ServiceInfoImpl) dns.getServices().get(getKey());
            if (info == null) {
                return false;
            }
            if (this._port == info.getPort() && this._server.equalsIgnoreCase(dns.getLocalHost().getName())) {
                return false;
            }
            logger1.finer("handleResponse() Denial detected");
            if (info.isProbing()) {
                String oldName = info.getQualifiedName().toLowerCase();
                info.setName(dns.incrementName(info.getName()));
                dns.getServices().remove(oldName);
                dns.getServices().put(info.getQualifiedName().toLowerCase(), info);
                Logger logger = logger1;
                logger.finer("handleResponse() New unique name chose:" + info.getName());
            }
            info.revertState();
            return true;
        }

        /* access modifiers changed from: package-private */
        public DNSOutgoing addAnswer(JmDNSImpl dns, DNSIncoming in, InetAddress addr, int port, DNSOutgoing out) throws IOException {
            ServiceInfoImpl info = (ServiceInfoImpl) dns.getServices().get(getKey());
            if (info != null) {
                if ((this._port == info.getPort()) != this._server.equals(dns.getLocalHost().getName())) {
                    return dns.addAnswer(in, addr, port, out, new Service(info.getQualifiedName(), DNSRecordClass.CLASS_IN, true, DNSConstants.DNS_TTL, info.getPriority(), info.getWeight(), info.getPort(), dns.getLocalHost().getName()));
                }
            }
            return out;
        }

        public ServiceInfo getServiceInfo(boolean persistent) {
            return new ServiceInfoImpl(getQualifiedNameMap(), this._port, this._weight, this._priority, persistent, this._server);
        }

        public ServiceEvent getServiceEvent(JmDNSImpl dns) {
            ServiceInfo info = getServiceInfo(false);
            ((ServiceInfoImpl) info).setDns(dns);
            return new ServiceEventImpl(dns, info.getType(), info.getName(), info);
        }

        /* access modifiers changed from: protected */
        public void toString(StringBuilder aLog) {
            DNSRecord.super.toString(aLog);
            aLog.append(" server: '" + this._server + ":" + this._port + "'");
        }
    }

    public static class HostInformation extends DNSRecord {
        String _cpu;
        String _os;

        public HostInformation(String name, DNSRecordClass recordClass, boolean unique, int ttl, String cpu, String os) {
            super(name, DNSRecordType.TYPE_HINFO, recordClass, unique, ttl);
            this._cpu = cpu;
            this._os = os;
        }

        /* access modifiers changed from: package-private */
        public DNSOutgoing addAnswer(JmDNSImpl dns, DNSIncoming in, InetAddress addr, int port, DNSOutgoing out) throws IOException {
            return out;
        }

        /* access modifiers changed from: package-private */
        public boolean handleQuery(JmDNSImpl dns, long expirationTime) {
            return false;
        }

        /* access modifiers changed from: package-private */
        public boolean handleResponse(JmDNSImpl dns) {
            return false;
        }

        /* access modifiers changed from: package-private */
        public boolean sameValue(DNSRecord other) {
            if (!(other instanceof HostInformation)) {
                return false;
            }
            HostInformation hinfo = (HostInformation) other;
            if (this._cpu == null && hinfo._cpu != null) {
                return false;
            }
            if ((this._os != null || hinfo._os == null) && this._cpu.equals(hinfo._cpu) && this._os.equals(hinfo._os)) {
                return true;
            }
            return false;
        }

        public boolean isSingleValued() {
            return true;
        }

        /* access modifiers changed from: package-private */
        public void write(DNSOutgoing.MessageOutputStream out) {
            String hostInfo = this._cpu + " " + this._os;
            out.writeUTF(hostInfo, 0, hostInfo.length());
        }

        public ServiceInfo getServiceInfo(boolean persistent) {
            Map<String, String> hinfo = new HashMap<>(2);
            hinfo.put("cpu", this._cpu);
            hinfo.put("os", this._os);
            return new ServiceInfoImpl(getQualifiedNameMap(), 0, 0, 0, persistent, (Map<String, ?>) hinfo);
        }

        public ServiceEvent getServiceEvent(JmDNSImpl dns) {
            ServiceInfo info = getServiceInfo(false);
            ((ServiceInfoImpl) info).setDns(dns);
            return new ServiceEventImpl(dns, info.getType(), info.getName(), info);
        }

        /* access modifiers changed from: protected */
        public void toString(StringBuilder aLog) {
            DNSRecord.super.toString(aLog);
            aLog.append(" cpu: '" + this._cpu + "' os: '" + this._os + "'");
        }
    }

    public ServiceInfo getServiceInfo() {
        return getServiceInfo(false);
    }

    public void setRecordSource(InetAddress source) {
        this._source = source;
    }

    public InetAddress getRecordSource() {
        return this._source;
    }

    /* access modifiers changed from: protected */
    public void toString(StringBuilder aLog) {
        super.toString(aLog);
        aLog.append(" ttl: '" + getRemainingTTL(System.currentTimeMillis()) + CookieSpec.PATH_DELIM + this._ttl + "'");
    }

    public void setTTL(int ttl) {
        this._ttl = ttl;
    }

    public int getTTL() {
        return this._ttl;
    }
}
