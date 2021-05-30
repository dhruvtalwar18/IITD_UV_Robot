package javax.jmdns.impl;

import java.net.InetAddress;
import java.util.Iterator;
import java.util.Set;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.jmdns.ServiceInfo;
import javax.jmdns.impl.DNSRecord;
import javax.jmdns.impl.constants.DNSConstants;
import javax.jmdns.impl.constants.DNSRecordClass;
import javax.jmdns.impl.constants.DNSRecordType;

public class DNSQuestion extends DNSEntry {
    private static Logger logger = Logger.getLogger(DNSQuestion.class.getName());

    private static class DNS4Address extends DNSQuestion {
        DNS4Address(String name, DNSRecordType type, DNSRecordClass recordClass, boolean unique) {
            super(name, type, recordClass, unique);
        }

        public void addAnswers(JmDNSImpl jmDNSImpl, Set<DNSRecord> answers) {
            DNSRecord answer = jmDNSImpl.getLocalHost().getDNSAddressRecord(getRecordType(), true, DNSConstants.DNS_TTL);
            if (answer != null) {
                answers.add(answer);
            }
        }

        public boolean iAmTheOnlyOne(JmDNSImpl jmDNSImpl) {
            String name = getName().toLowerCase();
            return jmDNSImpl.getLocalHost().getName().equals(name) || jmDNSImpl.getServices().keySet().contains(name);
        }
    }

    private static class DNS6Address extends DNSQuestion {
        DNS6Address(String name, DNSRecordType type, DNSRecordClass recordClass, boolean unique) {
            super(name, type, recordClass, unique);
        }

        public void addAnswers(JmDNSImpl jmDNSImpl, Set<DNSRecord> answers) {
            DNSRecord answer = jmDNSImpl.getLocalHost().getDNSAddressRecord(getRecordType(), true, DNSConstants.DNS_TTL);
            if (answer != null) {
                answers.add(answer);
            }
        }

        public boolean iAmTheOnlyOne(JmDNSImpl jmDNSImpl) {
            String name = getName().toLowerCase();
            return jmDNSImpl.getLocalHost().getName().equals(name) || jmDNSImpl.getServices().keySet().contains(name);
        }
    }

    private static class HostInformation extends DNSQuestion {
        HostInformation(String name, DNSRecordType type, DNSRecordClass recordClass, boolean unique) {
            super(name, type, recordClass, unique);
        }
    }

    private static class Pointer extends DNSQuestion {
        Pointer(String name, DNSRecordType type, DNSRecordClass recordClass, boolean unique) {
            super(name, type, recordClass, unique);
        }

        public void addAnswers(JmDNSImpl jmDNSImpl, Set<DNSRecord> answers) {
            Iterator<ServiceInfo> it = jmDNSImpl.getServices().values().iterator();
            while (it.hasNext()) {
                addAnswersForServiceInfo(jmDNSImpl, answers, (ServiceInfoImpl) it.next());
            }
            if (isServicesDiscoveryMetaQuery()) {
                for (String serviceType : jmDNSImpl.getServiceTypes().keySet()) {
                    answers.add(new DNSRecord.Pointer("_services._dns-sd._udp.local.", DNSRecordClass.CLASS_IN, false, DNSConstants.DNS_TTL, jmDNSImpl.getServiceTypes().get(serviceType).getType()));
                }
            } else if (isReverseLookup()) {
                String ipValue = getQualifiedNameMap().get(ServiceInfo.Fields.Instance);
                if (ipValue != null && ipValue.length() > 0) {
                    InetAddress address = jmDNSImpl.getLocalHost().getInetAddress();
                    if (ipValue.equalsIgnoreCase(address != null ? address.getHostAddress() : "")) {
                        if (isV4ReverseLookup()) {
                            answers.add(jmDNSImpl.getLocalHost().getDNSReverseAddressRecord(DNSRecordType.TYPE_A, false, DNSConstants.DNS_TTL));
                        }
                        if (isV6ReverseLookup()) {
                            answers.add(jmDNSImpl.getLocalHost().getDNSReverseAddressRecord(DNSRecordType.TYPE_AAAA, false, DNSConstants.DNS_TTL));
                        }
                    }
                }
            } else {
                isDomainDiscoveryQuery();
            }
        }
    }

    private static class Service extends DNSQuestion {
        Service(String name, DNSRecordType type, DNSRecordClass recordClass, boolean unique) {
            super(name, type, recordClass, unique);
        }

        public void addAnswers(JmDNSImpl jmDNSImpl, Set<DNSRecord> answers) {
            String loname = getName().toLowerCase();
            if (jmDNSImpl.getLocalHost().getName().equalsIgnoreCase(loname)) {
                answers.addAll(jmDNSImpl.getLocalHost().answers(isUnique(), DNSConstants.DNS_TTL));
            } else if (jmDNSImpl.getServiceTypes().containsKey(loname)) {
                new Pointer(getName(), DNSRecordType.TYPE_PTR, getRecordClass(), isUnique()).addAnswers(jmDNSImpl, answers);
            } else {
                addAnswersForServiceInfo(jmDNSImpl, answers, (ServiceInfoImpl) jmDNSImpl.getServices().get(loname));
            }
        }

        public boolean iAmTheOnlyOne(JmDNSImpl jmDNSImpl) {
            String name = getName().toLowerCase();
            return jmDNSImpl.getLocalHost().getName().equals(name) || jmDNSImpl.getServices().keySet().contains(name);
        }
    }

    private static class Text extends DNSQuestion {
        Text(String name, DNSRecordType type, DNSRecordClass recordClass, boolean unique) {
            super(name, type, recordClass, unique);
        }

        public void addAnswers(JmDNSImpl jmDNSImpl, Set<DNSRecord> answers) {
            addAnswersForServiceInfo(jmDNSImpl, answers, (ServiceInfoImpl) jmDNSImpl.getServices().get(getName().toLowerCase()));
        }

        public boolean iAmTheOnlyOne(JmDNSImpl jmDNSImpl) {
            String name = getName().toLowerCase();
            return jmDNSImpl.getLocalHost().getName().equals(name) || jmDNSImpl.getServices().keySet().contains(name);
        }
    }

    private static class AllRecords extends DNSQuestion {
        AllRecords(String name, DNSRecordType type, DNSRecordClass recordClass, boolean unique) {
            super(name, type, recordClass, unique);
        }

        public boolean isSameType(DNSEntry entry) {
            return entry != null;
        }

        public void addAnswers(JmDNSImpl jmDNSImpl, Set<DNSRecord> answers) {
            String loname = getName().toLowerCase();
            if (jmDNSImpl.getLocalHost().getName().equalsIgnoreCase(loname)) {
                answers.addAll(jmDNSImpl.getLocalHost().answers(isUnique(), DNSConstants.DNS_TTL));
            } else if (jmDNSImpl.getServiceTypes().containsKey(loname)) {
                new Pointer(getName(), DNSRecordType.TYPE_PTR, getRecordClass(), isUnique()).addAnswers(jmDNSImpl, answers);
            } else {
                addAnswersForServiceInfo(jmDNSImpl, answers, (ServiceInfoImpl) jmDNSImpl.getServices().get(loname));
            }
        }

        public boolean iAmTheOnlyOne(JmDNSImpl jmDNSImpl) {
            String name = getName().toLowerCase();
            return jmDNSImpl.getLocalHost().getName().equals(name) || jmDNSImpl.getServices().keySet().contains(name);
        }
    }

    DNSQuestion(String name, DNSRecordType type, DNSRecordClass recordClass, boolean unique) {
        super(name, type, recordClass, unique);
    }

    public static DNSQuestion newQuestion(String name, DNSRecordType type, DNSRecordClass recordClass, boolean unique) {
        switch (type) {
            case TYPE_A:
                return new DNS4Address(name, type, recordClass, unique);
            case TYPE_A6:
                return new DNS6Address(name, type, recordClass, unique);
            case TYPE_AAAA:
                return new DNS6Address(name, type, recordClass, unique);
            case TYPE_ANY:
                return new AllRecords(name, type, recordClass, unique);
            case TYPE_HINFO:
                return new HostInformation(name, type, recordClass, unique);
            case TYPE_PTR:
                return new Pointer(name, type, recordClass, unique);
            case TYPE_SRV:
                return new Service(name, type, recordClass, unique);
            case TYPE_TXT:
                return new Text(name, type, recordClass, unique);
            default:
                return new DNSQuestion(name, type, recordClass, unique);
        }
    }

    /* access modifiers changed from: package-private */
    public boolean answeredBy(DNSEntry rec) {
        return isSameRecordClass(rec) && isSameType(rec) && getName().equals(rec.getName());
    }

    public void addAnswers(JmDNSImpl jmDNSImpl, Set<DNSRecord> set) {
    }

    /* access modifiers changed from: protected */
    public void addAnswersForServiceInfo(JmDNSImpl jmDNSImpl, Set<DNSRecord> answers, ServiceInfoImpl info) {
        if (info != null && info.isAnnounced()) {
            if (getName().equalsIgnoreCase(info.getQualifiedName()) || getName().equalsIgnoreCase(info.getType())) {
                answers.addAll(jmDNSImpl.getLocalHost().answers(true, DNSConstants.DNS_TTL));
                answers.addAll(info.answers(true, DNSConstants.DNS_TTL, jmDNSImpl.getLocalHost()));
            }
            if (logger.isLoggable(Level.FINER)) {
                Logger logger2 = logger;
                logger2.finer(jmDNSImpl.getName() + " DNSQuestion(" + getName() + ").addAnswersForServiceInfo(): info: " + info + "\n" + answers);
            }
        }
    }

    public boolean isStale(long now) {
        return false;
    }

    public boolean isExpired(long now) {
        return false;
    }

    public boolean iAmTheOnlyOne(JmDNSImpl jmDNSImpl) {
        return false;
    }

    public void toString(StringBuilder aLog) {
    }
}
