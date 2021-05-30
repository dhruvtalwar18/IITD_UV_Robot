package javax.jmdns.impl;

import java.io.ByteArrayOutputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.util.Collections;
import java.util.Map;
import javax.jmdns.ServiceInfo;
import javax.jmdns.impl.constants.DNSRecordClass;
import javax.jmdns.impl.constants.DNSRecordType;

public abstract class DNSEntry {
    private final DNSRecordClass _dnsClass;
    private final String _key;
    private final String _name;
    final Map<ServiceInfo.Fields, String> _qualifiedNameMap = ServiceInfoImpl.decodeQualifiedNameMapForType(getName());
    private final DNSRecordType _recordType;
    private final String _type;
    private final boolean _unique;

    public abstract boolean isExpired(long j);

    public abstract boolean isStale(long j);

    DNSEntry(String name, DNSRecordType type, DNSRecordClass recordClass, boolean unique) {
        String str;
        String str2;
        String str3;
        this._name = name;
        this._recordType = type;
        this._dnsClass = recordClass;
        this._unique = unique;
        String domain = this._qualifiedNameMap.get(ServiceInfo.Fields.Domain);
        String protocol = this._qualifiedNameMap.get(ServiceInfo.Fields.Protocol);
        String application = this._qualifiedNameMap.get(ServiceInfo.Fields.Application);
        String instance = this._qualifiedNameMap.get(ServiceInfo.Fields.Instance).toLowerCase();
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
        this._type = sb.toString();
        StringBuilder sb2 = new StringBuilder();
        if (instance.length() > 0) {
            str3 = instance + ".";
        } else {
            str3 = "";
        }
        sb2.append(str3);
        sb2.append(this._type);
        this._key = sb2.toString().toLowerCase();
    }

    public boolean equals(Object obj) {
        if (!(obj instanceof DNSEntry)) {
            return false;
        }
        DNSEntry other = (DNSEntry) obj;
        return getKey().equals(other.getKey()) && getRecordType().equals(other.getRecordType()) && getRecordClass() == other.getRecordClass();
    }

    public boolean isSameEntry(DNSEntry entry) {
        return getKey().equals(entry.getKey()) && getRecordType().equals(entry.getRecordType()) && (DNSRecordClass.CLASS_ANY == entry.getRecordClass() || getRecordClass().equals(entry.getRecordClass()));
    }

    public boolean sameSubtype(DNSEntry other) {
        return getSubtype().equals(other.getSubtype());
    }

    public String getSubtype() {
        String subtype = getQualifiedNameMap().get(ServiceInfo.Fields.Subtype);
        return subtype != null ? subtype : "";
    }

    public String getName() {
        return this._name != null ? this._name : "";
    }

    public String getType() {
        return this._type != null ? this._type : "";
    }

    public String getKey() {
        return this._key != null ? this._key : "";
    }

    public DNSRecordType getRecordType() {
        return this._recordType != null ? this._recordType : DNSRecordType.TYPE_IGNORE;
    }

    public DNSRecordClass getRecordClass() {
        return this._dnsClass != null ? this._dnsClass : DNSRecordClass.CLASS_UNKNOWN;
    }

    public boolean isUnique() {
        return this._unique;
    }

    public Map<ServiceInfo.Fields, String> getQualifiedNameMap() {
        return Collections.unmodifiableMap(this._qualifiedNameMap);
    }

    public boolean isServicesDiscoveryMetaQuery() {
        return this._qualifiedNameMap.get(ServiceInfo.Fields.Application).equals("dns-sd") && this._qualifiedNameMap.get(ServiceInfo.Fields.Instance).equals("_services");
    }

    public boolean isDomainDiscoveryQuery() {
        if (!this._qualifiedNameMap.get(ServiceInfo.Fields.Application).equals("dns-sd")) {
            return false;
        }
        String name = this._qualifiedNameMap.get(ServiceInfo.Fields.Instance);
        if ("b".equals(name) || "db".equals(name) || "r".equals(name) || "dr".equals(name) || "lb".equals(name)) {
            return true;
        }
        return false;
    }

    public boolean isReverseLookup() {
        return isV4ReverseLookup() || isV6ReverseLookup();
    }

    public boolean isV4ReverseLookup() {
        return this._qualifiedNameMap.get(ServiceInfo.Fields.Domain).endsWith("in-addr.arpa");
    }

    public boolean isV6ReverseLookup() {
        return this._qualifiedNameMap.get(ServiceInfo.Fields.Domain).endsWith("ip6.arpa");
    }

    public boolean isSameRecordClass(DNSEntry entry) {
        return entry != null && entry.getRecordClass() == getRecordClass();
    }

    public boolean isSameType(DNSEntry entry) {
        return entry != null && entry.getRecordType() == getRecordType();
    }

    /* access modifiers changed from: protected */
    public void toByteArray(DataOutputStream dout) throws IOException {
        dout.write(getName().getBytes("UTF8"));
        dout.writeShort(getRecordType().indexValue());
        dout.writeShort(getRecordClass().indexValue());
    }

    /* access modifiers changed from: protected */
    public byte[] toByteArray() {
        try {
            ByteArrayOutputStream bout = new ByteArrayOutputStream();
            DataOutputStream dout = new DataOutputStream(bout);
            toByteArray(dout);
            dout.close();
            return bout.toByteArray();
        } catch (IOException e) {
            throw new InternalError();
        }
    }

    public int compareTo(DNSEntry that) {
        byte[] thisBytes = toByteArray();
        byte[] thatBytes = that.toByteArray();
        int n = Math.min(thisBytes.length, thatBytes.length);
        for (int i = 0; i < n; i++) {
            if (thisBytes[i] > thatBytes[i]) {
                return 1;
            }
            if (thisBytes[i] < thatBytes[i]) {
                return -1;
            }
        }
        return thisBytes.length - thatBytes.length;
    }

    public int hashCode() {
        return getKey().hashCode() + getRecordType().indexValue() + getRecordClass().indexValue();
    }

    public String toString() {
        StringBuilder aLog = new StringBuilder(200);
        aLog.append("[" + getClass().getSimpleName() + "@" + System.identityHashCode(this));
        StringBuilder sb = new StringBuilder();
        sb.append(" type: ");
        sb.append(getRecordType());
        aLog.append(sb.toString());
        aLog.append(", class: " + getRecordClass());
        aLog.append(this._unique ? "-unique," : ",");
        aLog.append(" name: " + this._name);
        toString(aLog);
        aLog.append("]");
        return aLog.toString();
    }

    /* access modifiers changed from: protected */
    public void toString(StringBuilder aLog) {
    }
}
