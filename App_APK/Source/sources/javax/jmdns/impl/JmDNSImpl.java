package javax.jmdns.impl;

import java.io.IOException;
import java.io.PrintStream;
import java.io.Serializable;
import java.net.DatagramPacket;
import java.net.Inet4Address;
import java.net.Inet6Address;
import java.net.InetAddress;
import java.net.MulticastSocket;
import java.net.SocketException;
import java.util.AbstractMap;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Properties;
import java.util.Random;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentMap;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.locks.ReentrantLock;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.jmdns.JmDNS;
import javax.jmdns.ServiceEvent;
import javax.jmdns.ServiceInfo;
import javax.jmdns.ServiceListener;
import javax.jmdns.ServiceTypeListener;
import javax.jmdns.impl.DNSRecord;
import javax.jmdns.impl.DNSTaskStarter;
import javax.jmdns.impl.ListenerStatus;
import javax.jmdns.impl.constants.DNSConstants;
import javax.jmdns.impl.constants.DNSRecordClass;
import javax.jmdns.impl.constants.DNSRecordType;
import javax.jmdns.impl.constants.DNSState;
import javax.jmdns.impl.tasks.DNSTask;

public class JmDNSImpl extends JmDNS implements DNSStatefulObject, DNSTaskStarter {
    private static final Random _random = new Random();
    private static Logger logger = Logger.getLogger(JmDNSImpl.class.getName());
    private final DNSCache _cache;
    private volatile JmDNS.Delegate _delegate;
    private final ExecutorService _executor = Executors.newSingleThreadExecutor();
    private volatile InetAddress _group;
    private Thread _incomingListener;
    private final ReentrantLock _ioLock = new ReentrantLock();
    private long _lastThrottleIncrement;
    private final List<DNSListener> _listeners;
    private HostInfo _localHost;
    private final String _name;
    private DNSIncoming _plannedAnswer;
    private final Object _recoverLock = new Object();
    private final ConcurrentMap<String, ServiceCollector> _serviceCollectors;
    private final ConcurrentMap<String, List<ListenerStatus.ServiceListenerStatus>> _serviceListeners;
    private final ConcurrentMap<String, ServiceTypeEntry> _serviceTypes;
    private final ConcurrentMap<String, ServiceInfo> _services;
    protected Thread _shutdown;
    private volatile MulticastSocket _socket;
    private int _throttle;
    private final Set<ListenerStatus.ServiceTypeListenerStatus> _typeListeners;

    public enum Operation {
        Remove,
        Update,
        Add,
        RegisterServiceType,
        Noop
    }

    public static class ServiceTypeEntry extends AbstractMap<String, String> implements Cloneable {
        private final Set<Map.Entry<String, String>> _entrySet = new HashSet();
        private final String _type;

        private static class SubTypeEntry implements Map.Entry<String, String>, Serializable, Cloneable {
            private static final long serialVersionUID = 9188503522395855322L;
            private final String _key;
            private final String _value;

            public SubTypeEntry(String subtype) {
                this._value = subtype != null ? subtype : "";
                this._key = this._value.toLowerCase();
            }

            public String getKey() {
                return this._key;
            }

            public String getValue() {
                return this._value;
            }

            public String setValue(String value) {
                throw new UnsupportedOperationException();
            }

            public boolean equals(Object entry) {
                if ((entry instanceof Map.Entry) && getKey().equals(((Map.Entry) entry).getKey()) && getValue().equals(((Map.Entry) entry).getValue())) {
                    return true;
                }
                return false;
            }

            public int hashCode() {
                int i = 0;
                int hashCode = this._key == null ? 0 : this._key.hashCode();
                if (this._value != null) {
                    i = this._value.hashCode();
                }
                return hashCode ^ i;
            }

            public SubTypeEntry clone() {
                return this;
            }

            public String toString() {
                return this._key + "=" + this._value;
            }
        }

        public ServiceTypeEntry(String type) {
            this._type = type;
        }

        public String getType() {
            return this._type;
        }

        public Set<Map.Entry<String, String>> entrySet() {
            return this._entrySet;
        }

        public boolean contains(String subtype) {
            return subtype != null && containsKey(subtype.toLowerCase());
        }

        public boolean add(String subtype) {
            if (subtype == null || contains(subtype)) {
                return false;
            }
            this._entrySet.add(new SubTypeEntry(subtype));
            return true;
        }

        public Iterator<String> iterator() {
            return keySet().iterator();
        }

        public ServiceTypeEntry clone() {
            ServiceTypeEntry entry = new ServiceTypeEntry(getType());
            for (Map.Entry<String, String> subTypeEntry : entrySet()) {
                entry.add(subTypeEntry.getValue());
            }
            return entry;
        }

        public String toString() {
            StringBuilder aLog = new StringBuilder(200);
            if (isEmpty()) {
                aLog.append("empty");
            } else {
                for (String value : values()) {
                    aLog.append(value);
                    aLog.append(", ");
                }
                aLog.setLength(aLog.length() - 2);
            }
            return aLog.toString();
        }
    }

    public static void main(String[] argv) {
        String version;
        try {
            Properties pomProperties = new Properties();
            pomProperties.load(JmDNSImpl.class.getResourceAsStream("/META-INF/maven/javax.jmdns/jmdns/pom.properties"));
            version = pomProperties.getProperty("version");
        } catch (Exception e) {
            version = "RUNNING.IN.IDE.FULL";
        }
        PrintStream printStream = System.out;
        printStream.println("JmDNS version \"" + version + "\"");
        System.out.println(" ");
        PrintStream printStream2 = System.out;
        printStream2.println("Running on java version \"" + System.getProperty("java.version") + "\" (build " + System.getProperty("java.runtime.version") + ") from " + System.getProperty("java.vendor"));
        PrintStream printStream3 = System.out;
        printStream3.println("Operating environment \"" + System.getProperty("os.name") + "\" version " + System.getProperty("os.version") + " on " + System.getProperty("os.arch"));
        System.out.println("For more information on JmDNS please visit https://sourceforge.net/projects/jmdns/");
    }

    public JmDNSImpl(InetAddress address, String name) throws IOException {
        if (logger.isLoggable(Level.FINER)) {
            logger.finer("JmDNS instance created");
        }
        this._cache = new DNSCache(100);
        this._listeners = Collections.synchronizedList(new ArrayList());
        this._serviceListeners = new ConcurrentHashMap();
        this._typeListeners = Collections.synchronizedSet(new HashSet());
        this._serviceCollectors = new ConcurrentHashMap();
        this._services = new ConcurrentHashMap(20);
        this._serviceTypes = new ConcurrentHashMap(20);
        this._localHost = HostInfo.newHostInfo(address, this, name);
        this._name = name != null ? name : this._localHost.getName();
        openMulticastSocket(getLocalHost());
        start(getServices().values());
        startReaper();
    }

    private void start(Collection<? extends ServiceInfo> serviceInfos) {
        if (this._incomingListener == null) {
            this._incomingListener = new SocketListener(this);
            this._incomingListener.start();
        }
        startProber();
        for (ServiceInfo info : serviceInfos) {
            try {
                registerService(new ServiceInfoImpl(info));
            } catch (Exception exception) {
                logger.log(Level.WARNING, "start() Registration exception ", exception);
            }
        }
    }

    private void openMulticastSocket(HostInfo hostInfo) throws IOException {
        if (this._group == null) {
            if (hostInfo.getInetAddress() instanceof Inet6Address) {
                this._group = InetAddress.getByName(DNSConstants.MDNS_GROUP_IPV6);
            } else {
                this._group = InetAddress.getByName(DNSConstants.MDNS_GROUP);
            }
        }
        if (this._socket != null) {
            closeMulticastSocket();
        }
        this._socket = new MulticastSocket(DNSConstants.MDNS_PORT);
        if (!(hostInfo == null || hostInfo.getInterface() == null)) {
            try {
                this._socket.setNetworkInterface(hostInfo.getInterface());
            } catch (SocketException e) {
                if (logger.isLoggable(Level.FINE)) {
                    Logger logger2 = logger;
                    logger2.fine("openMulticastSocket() Set network interface exception: " + e.getMessage());
                }
            }
        }
        this._socket.setTimeToLive(255);
        this._socket.joinGroup(this._group);
    }

    private void closeMulticastSocket() {
        if (logger.isLoggable(Level.FINER)) {
            logger.finer("closeMulticastSocket()");
        }
        if (this._socket != null) {
            try {
                this._socket.leaveGroup(this._group);
            } catch (SocketException e) {
            }
            try {
                this._socket.close();
                while (this._incomingListener != null && this._incomingListener.isAlive()) {
                    synchronized (this) {
                        try {
                            if (this._incomingListener != null && this._incomingListener.isAlive()) {
                                if (logger.isLoggable(Level.FINER)) {
                                    logger.finer("closeMulticastSocket(): waiting for jmDNS monitor");
                                }
                                wait(1000);
                            }
                        } catch (InterruptedException e2) {
                        }
                    }
                }
                this._incomingListener = null;
            } catch (Exception exception) {
                logger.log(Level.WARNING, "closeMulticastSocket() Close socket exception ", exception);
            }
            this._socket = null;
        }
    }

    public boolean advanceState(DNSTask task) {
        return this._localHost.advanceState(task);
    }

    public boolean revertState() {
        return this._localHost.revertState();
    }

    public boolean cancelState() {
        return this._localHost.cancelState();
    }

    public boolean closeState() {
        return this._localHost.closeState();
    }

    public boolean recoverState() {
        return this._localHost.recoverState();
    }

    public JmDNSImpl getDns() {
        return this;
    }

    public void associateWithTask(DNSTask task, DNSState state) {
        this._localHost.associateWithTask(task, state);
    }

    public void removeAssociationWithTask(DNSTask task) {
        this._localHost.removeAssociationWithTask(task);
    }

    public boolean isAssociatedWithTask(DNSTask task, DNSState state) {
        return this._localHost.isAssociatedWithTask(task, state);
    }

    public boolean isProbing() {
        return this._localHost.isProbing();
    }

    public boolean isAnnouncing() {
        return this._localHost.isAnnouncing();
    }

    public boolean isAnnounced() {
        return this._localHost.isAnnounced();
    }

    public boolean isCanceling() {
        return this._localHost.isCanceling();
    }

    public boolean isCanceled() {
        return this._localHost.isCanceled();
    }

    public boolean isClosing() {
        return this._localHost.isClosing();
    }

    public boolean isClosed() {
        return this._localHost.isClosed();
    }

    public boolean waitForAnnounced(long timeout) {
        return this._localHost.waitForAnnounced(timeout);
    }

    public boolean waitForCanceled(long timeout) {
        return this._localHost.waitForCanceled(timeout);
    }

    public DNSCache getCache() {
        return this._cache;
    }

    public String getName() {
        return this._name;
    }

    public String getHostName() {
        return this._localHost.getName();
    }

    public HostInfo getLocalHost() {
        return this._localHost;
    }

    public InetAddress getInetAddress() throws IOException {
        return this._localHost.getInetAddress();
    }

    @Deprecated
    public InetAddress getInterface() throws IOException {
        return this._socket.getInterface();
    }

    public ServiceInfo getServiceInfo(String type, String name) {
        return getServiceInfo(type, name, false, DNSConstants.SERVICE_INFO_TIMEOUT);
    }

    public ServiceInfo getServiceInfo(String type, String name, long timeout) {
        return getServiceInfo(type, name, false, timeout);
    }

    public ServiceInfo getServiceInfo(String type, String name, boolean persistent) {
        return getServiceInfo(type, name, persistent, DNSConstants.SERVICE_INFO_TIMEOUT);
    }

    public ServiceInfo getServiceInfo(String type, String name, boolean persistent, long timeout) {
        ServiceInfoImpl info = resolveServiceInfo(type, name, "", persistent);
        waitForInfoData(info, timeout);
        if (info.hasData()) {
            return info;
        }
        return null;
    }

    /* access modifiers changed from: package-private */
    public ServiceInfoImpl resolveServiceInfo(String type, String name, String subtype, boolean persistent) {
        cleanCache();
        String loType = type.toLowerCase();
        registerServiceType(type);
        if (this._serviceCollectors.putIfAbsent(loType, new ServiceCollector(type)) == null) {
            addServiceListener(loType, (ServiceListener) this._serviceCollectors.get(loType), true);
        }
        ServiceInfoImpl info = getServiceInfoFromCache(type, name, subtype, persistent);
        startServiceInfoResolver(info);
        return info;
    }

    /* access modifiers changed from: package-private */
    public ServiceInfoImpl getServiceInfoFromCache(String type, String name, String subtype, boolean persistent) {
        ServiceInfoImpl cachedInfo;
        ServiceInfo cachedTextInfo;
        ServiceInfo cachedAddressInfo;
        ServiceInfo cachedAddressInfo2;
        ServiceInfo cachedServiceEntryInfo;
        boolean z = persistent;
        byte[] bArr = null;
        ServiceInfoImpl info = new ServiceInfoImpl(type, name, subtype, 0, 0, 0, persistent, bArr);
        DNSEntry pointerEntry = getCache().getDNSEntry(new DNSRecord.Pointer(type, DNSRecordClass.CLASS_ANY, false, 0, info.getQualifiedName()));
        if (!(pointerEntry instanceof DNSRecord) || (cachedInfo = (ServiceInfoImpl) ((DNSRecord) pointerEntry).getServiceInfo(z)) == null) {
            return info;
        }
        Map<ServiceInfo.Fields, String> map = cachedInfo.getQualifiedNameMap();
        byte[] srvBytes = null;
        String server = "";
        DNSEntry serviceEntry = getCache().getDNSEntry(info.getQualifiedName(), DNSRecordType.TYPE_SRV, DNSRecordClass.CLASS_ANY);
        if ((serviceEntry instanceof DNSRecord) && (cachedServiceEntryInfo = ((DNSRecord) serviceEntry).getServiceInfo(z)) != null) {
            cachedInfo = new ServiceInfoImpl(map, cachedServiceEntryInfo.getPort(), cachedServiceEntryInfo.getWeight(), cachedServiceEntryInfo.getPriority(), persistent, bArr);
            srvBytes = cachedServiceEntryInfo.getTextBytes();
            server = cachedServiceEntryInfo.getServer();
        }
        DNSEntry addressEntry = getCache().getDNSEntry(server, DNSRecordType.TYPE_A, DNSRecordClass.CLASS_ANY);
        if ((addressEntry instanceof DNSRecord) && (cachedAddressInfo2 = ((DNSRecord) addressEntry).getServiceInfo(z)) != null) {
            for (Inet4Address address : cachedAddressInfo2.getInet4Addresses()) {
                cachedInfo.addAddress(address);
            }
            cachedInfo._setText(cachedAddressInfo2.getTextBytes());
        }
        DNSEntry addressEntry2 = getCache().getDNSEntry(server, DNSRecordType.TYPE_AAAA, DNSRecordClass.CLASS_ANY);
        if ((addressEntry2 instanceof DNSRecord) && (cachedAddressInfo = ((DNSRecord) addressEntry2).getServiceInfo(z)) != null) {
            for (Inet6Address address2 : cachedAddressInfo.getInet6Addresses()) {
                cachedInfo.addAddress(address2);
            }
            cachedInfo._setText(cachedAddressInfo.getTextBytes());
        }
        DNSEntry textEntry = getCache().getDNSEntry(cachedInfo.getQualifiedName(), DNSRecordType.TYPE_TXT, DNSRecordClass.CLASS_ANY);
        if ((textEntry instanceof DNSRecord) && (cachedTextInfo = ((DNSRecord) textEntry).getServiceInfo(z)) != null) {
            cachedInfo._setText(cachedTextInfo.getTextBytes());
        }
        if (cachedInfo.getTextBytes().length == 0) {
            cachedInfo._setText(srvBytes);
        }
        if (cachedInfo.hasData()) {
            return cachedInfo;
        }
        return info;
    }

    private void waitForInfoData(ServiceInfo info, long timeout) {
        synchronized (info) {
            long loops = timeout / 200;
            if (loops < 1) {
                loops = 1;
            }
            for (int i = 0; ((long) i) < loops && !info.hasData(); i++) {
                try {
                    info.wait(200);
                } catch (InterruptedException e) {
                }
            }
        }
    }

    public void requestServiceInfo(String type, String name) {
        requestServiceInfo(type, name, false, DNSConstants.SERVICE_INFO_TIMEOUT);
    }

    public void requestServiceInfo(String type, String name, boolean persistent) {
        requestServiceInfo(type, name, persistent, DNSConstants.SERVICE_INFO_TIMEOUT);
    }

    public void requestServiceInfo(String type, String name, long timeout) {
        requestServiceInfo(type, name, false, DNSConstants.SERVICE_INFO_TIMEOUT);
    }

    public void requestServiceInfo(String type, String name, boolean persistent, long timeout) {
        waitForInfoData(resolveServiceInfo(type, name, "", persistent), timeout);
    }

    /* access modifiers changed from: package-private */
    public void handleServiceResolved(ServiceEvent event) {
        List<ListenerStatus.ServiceListenerStatus> listCopy;
        List<ListenerStatus.ServiceListenerStatus> list = (List) this._serviceListeners.get(event.getType().toLowerCase());
        if (list != null && !list.isEmpty() && event.getInfo() != null && event.getInfo().hasData()) {
            final ServiceEvent localEvent = event;
            synchronized (list) {
                listCopy = new ArrayList<>(list);
            }
            for (final ListenerStatus.ServiceListenerStatus listener : listCopy) {
                this._executor.submit(new Runnable() {
                    public void run() {
                        listener.serviceResolved(localEvent);
                    }
                });
            }
        }
    }

    public void addServiceTypeListener(ServiceTypeListener listener) throws IOException {
        ListenerStatus.ServiceTypeListenerStatus status = new ListenerStatus.ServiceTypeListenerStatus(listener, false);
        this._typeListeners.add(status);
        for (String type : this._serviceTypes.keySet()) {
            status.serviceTypeAdded(new ServiceEventImpl(this, type, "", (ServiceInfo) null));
        }
        startTypeResolver();
    }

    public void removeServiceTypeListener(ServiceTypeListener listener) {
        this._typeListeners.remove(new ListenerStatus.ServiceTypeListenerStatus(listener, false));
    }

    public void addServiceListener(String type, ServiceListener listener) {
        addServiceListener(type, listener, false);
    }

    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r3v6, resolved type: java.lang.Object} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r2v4, resolved type: java.util.List} */
    /* JADX WARNING: Multi-variable type inference failed */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    private void addServiceListener(java.lang.String r13, javax.jmdns.ServiceListener r14, boolean r15) {
        /*
            r12 = this;
            javax.jmdns.impl.ListenerStatus$ServiceListenerStatus r0 = new javax.jmdns.impl.ListenerStatus$ServiceListenerStatus
            r0.<init>(r14, r15)
            java.lang.String r1 = r13.toLowerCase()
            java.util.concurrent.ConcurrentMap<java.lang.String, java.util.List<javax.jmdns.impl.ListenerStatus$ServiceListenerStatus>> r2 = r12._serviceListeners
            java.lang.Object r2 = r2.get(r1)
            java.util.List r2 = (java.util.List) r2
            if (r2 != 0) goto L_0x0042
            java.util.concurrent.ConcurrentMap<java.lang.String, java.util.List<javax.jmdns.impl.ListenerStatus$ServiceListenerStatus>> r3 = r12._serviceListeners
            java.util.LinkedList r4 = new java.util.LinkedList
            r4.<init>()
            java.lang.Object r3 = r3.putIfAbsent(r1, r4)
            if (r3 != 0) goto L_0x0039
            java.util.concurrent.ConcurrentMap<java.lang.String, javax.jmdns.impl.JmDNSImpl$ServiceCollector> r3 = r12._serviceCollectors
            javax.jmdns.impl.JmDNSImpl$ServiceCollector r4 = new javax.jmdns.impl.JmDNSImpl$ServiceCollector
            r4.<init>(r13)
            java.lang.Object r3 = r3.putIfAbsent(r1, r4)
            if (r3 != 0) goto L_0x0039
            java.util.concurrent.ConcurrentMap<java.lang.String, javax.jmdns.impl.JmDNSImpl$ServiceCollector> r3 = r12._serviceCollectors
            java.lang.Object r3 = r3.get(r1)
            javax.jmdns.ServiceListener r3 = (javax.jmdns.ServiceListener) r3
            r4 = 1
            r12.addServiceListener(r1, r3, r4)
        L_0x0039:
            java.util.concurrent.ConcurrentMap<java.lang.String, java.util.List<javax.jmdns.impl.ListenerStatus$ServiceListenerStatus>> r3 = r12._serviceListeners
            java.lang.Object r3 = r3.get(r1)
            r2 = r3
            java.util.List r2 = (java.util.List) r2
        L_0x0042:
            if (r2 == 0) goto L_0x0053
            monitor-enter(r2)
            boolean r3 = r2.contains(r14)     // Catch:{ all -> 0x0050 }
            if (r3 != 0) goto L_0x004e
            r2.add(r0)     // Catch:{ all -> 0x0050 }
        L_0x004e:
            monitor-exit(r2)     // Catch:{ all -> 0x0050 }
            goto L_0x0053
        L_0x0050:
            r3 = move-exception
            monitor-exit(r2)     // Catch:{ all -> 0x0050 }
            throw r3
        L_0x0053:
            java.util.ArrayList r3 = new java.util.ArrayList
            r3.<init>()
            javax.jmdns.impl.DNSCache r4 = r12.getCache()
            java.util.Collection r4 = r4.allValues()
            java.util.Iterator r5 = r4.iterator()
        L_0x0064:
            boolean r6 = r5.hasNext()
            if (r6 == 0) goto L_0x00a2
            java.lang.Object r6 = r5.next()
            javax.jmdns.impl.DNSEntry r6 = (javax.jmdns.impl.DNSEntry) r6
            r7 = r6
            javax.jmdns.impl.DNSRecord r7 = (javax.jmdns.impl.DNSRecord) r7
            javax.jmdns.impl.constants.DNSRecordType r8 = r7.getRecordType()
            javax.jmdns.impl.constants.DNSRecordType r9 = javax.jmdns.impl.constants.DNSRecordType.TYPE_SRV
            if (r8 != r9) goto L_0x00a1
            java.lang.String r8 = r7.getKey()
            boolean r8 = r8.endsWith(r1)
            if (r8 == 0) goto L_0x00a1
            javax.jmdns.impl.ServiceEventImpl r8 = new javax.jmdns.impl.ServiceEventImpl
            java.lang.String r9 = r7.getType()
            java.lang.String r10 = r7.getType()
            java.lang.String r11 = r7.getName()
            java.lang.String r10 = toUnqualifiedName(r10, r11)
            javax.jmdns.ServiceInfo r11 = r7.getServiceInfo()
            r8.<init>(r12, r9, r10, r11)
            r3.add(r8)
        L_0x00a1:
            goto L_0x0064
        L_0x00a2:
            java.util.Iterator r5 = r3.iterator()
        L_0x00a6:
            boolean r6 = r5.hasNext()
            if (r6 == 0) goto L_0x00b6
            java.lang.Object r6 = r5.next()
            javax.jmdns.ServiceEvent r6 = (javax.jmdns.ServiceEvent) r6
            r0.serviceAdded(r6)
            goto L_0x00a6
        L_0x00b6:
            r12.startServiceResolver(r13)
            return
        */
        throw new UnsupportedOperationException("Method not decompiled: javax.jmdns.impl.JmDNSImpl.addServiceListener(java.lang.String, javax.jmdns.ServiceListener, boolean):void");
    }

    public void removeServiceListener(String type, ServiceListener listener) {
        List<ListenerStatus.ServiceListenerStatus> listCopy;
        String loType = type.toLowerCase();
        List<ListenerStatus.ServiceListenerStatus> list = (List) this._serviceListeners.get(loType);
        for (ServiceInfo info : Arrays.asList(((ServiceCollector) this._serviceCollectors.get(type)).list(-1))) {
            final ServiceEvent event = new ServiceEventImpl(this, info.getType(), info.getName(), info);
            if (list != null && !list.isEmpty()) {
                synchronized (list) {
                    listCopy = new ArrayList<>(list);
                }
                for (final ListenerStatus.ServiceListenerStatus listener_status : listCopy) {
                    this._executor.submit(new Runnable() {
                        public void run() {
                            listener_status.serviceRemoved(event);
                        }
                    });
                }
            }
        }
        if (list != null) {
            synchronized (list) {
                list.remove(new ListenerStatus.ServiceListenerStatus(listener, false));
                if (list.isEmpty()) {
                    this._serviceListeners.remove(loType, list);
                }
            }
        }
    }

    public void registerService(ServiceInfo infoAbstract) throws IOException {
        if (isClosing() || isClosed()) {
            throw new IllegalStateException("This DNS is closed.");
        }
        ServiceInfoImpl info = (ServiceInfoImpl) infoAbstract;
        if (info.getDns() != null) {
            if (info.getDns() != this) {
                throw new IllegalStateException("A service information can only be registered with a single instamce of JmDNS.");
            } else if (this._services.get(info.getKey()) != null) {
                throw new IllegalStateException("A service information can only be registered once.");
            }
        }
        info.setDns(this);
        registerServiceType(info.getTypeWithSubtype());
        info.recoverState();
        info.setServer(this._localHost.getName());
        info.addAddress(this._localHost.getInet4Address());
        info.addAddress(this._localHost.getInet6Address());
        waitForAnnounced(DNSConstants.SERVICE_INFO_TIMEOUT);
        makeServiceNameUnique(info);
        while (this._services.putIfAbsent(info.getKey(), info) != null) {
            makeServiceNameUnique(info);
        }
        startProber();
        info.waitForAnnounced(DNSConstants.SERVICE_INFO_TIMEOUT);
        if (logger.isLoggable(Level.FINE)) {
            Logger logger2 = logger;
            logger2.fine("registerService() JmDNS registered service as " + info);
        }
    }

    public void unregisterService(ServiceInfo infoAbstract) {
        ServiceInfoImpl info = (ServiceInfoImpl) this._services.get(infoAbstract.getKey());
        if (info != null) {
            info.cancelState();
            startCanceler();
            info.waitForCanceled(DNSConstants.CLOSE_TIMEOUT);
            this._services.remove(info.getKey(), info);
            if (logger.isLoggable(Level.FINE)) {
                Logger logger2 = logger;
                logger2.fine("unregisterService() JmDNS unregistered service as " + info);
                return;
            }
            return;
        }
        Logger logger3 = logger;
        logger3.warning("Removing unregistered service info: " + infoAbstract.getKey());
    }

    public void unregisterAllServices() {
        if (logger.isLoggable(Level.FINER)) {
            logger.finer("unregisterAllServices()");
        }
        for (String name : this._services.keySet()) {
            ServiceInfoImpl info = (ServiceInfoImpl) this._services.get(name);
            if (info != null) {
                if (logger.isLoggable(Level.FINER)) {
                    Logger logger2 = logger;
                    logger2.finer("Cancelling service info: " + info);
                }
                info.cancelState();
            }
        }
        startCanceler();
        for (String name2 : this._services.keySet()) {
            ServiceInfoImpl info2 = (ServiceInfoImpl) this._services.get(name2);
            if (info2 != null) {
                if (logger.isLoggable(Level.FINER)) {
                    Logger logger3 = logger;
                    logger3.finer("Wait for service info cancel: " + info2);
                }
                info2.waitForCanceled(DNSConstants.CLOSE_TIMEOUT);
                this._services.remove(name2, info2);
            }
        }
    }

    public boolean registerServiceType(String type) {
        String str;
        String str2;
        boolean typeAdded;
        ServiceTypeEntry subtypes;
        String str3;
        Map<ServiceInfo.Fields, String> map = ServiceInfoImpl.decodeQualifiedNameMapForType(type);
        String domain = map.get(ServiceInfo.Fields.Domain);
        String protocol = map.get(ServiceInfo.Fields.Protocol);
        String application = map.get(ServiceInfo.Fields.Application);
        String subtype = map.get(ServiceInfo.Fields.Subtype);
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
        String name = sb.toString();
        String loname = name.toLowerCase();
        if (logger.isLoggable(Level.FINE)) {
            Logger logger2 = logger;
            StringBuilder sb2 = new StringBuilder();
            sb2.append(getName());
            sb2.append(".registering service type: ");
            sb2.append(type);
            sb2.append(" as: ");
            sb2.append(name);
            if (subtype.length() > 0) {
                str3 = " subtype: " + subtype;
            } else {
                str3 = "";
            }
            sb2.append(str3);
            logger2.fine(sb2.toString());
        } else {
            String str4 = type;
        }
        if (this._serviceTypes.containsKey(loname) || application.toLowerCase().equals("dns-sd") || domain.toLowerCase().endsWith("in-addr.arpa") || domain.toLowerCase().endsWith("ip6.arpa")) {
            typeAdded = false;
        } else {
            boolean typeAdded2 = this._serviceTypes.putIfAbsent(loname, new ServiceTypeEntry(name)) == null;
            if (typeAdded2) {
                ListenerStatus.ServiceTypeListenerStatus[] list = (ListenerStatus.ServiceTypeListenerStatus[]) this._typeListeners.toArray(new ListenerStatus.ServiceTypeListenerStatus[this._typeListeners.size()]);
                final ServiceEvent event = new ServiceEventImpl(this, name, "", (ServiceInfo) null);
                int length = list.length;
                int i = 0;
                while (i < length) {
                    final ListenerStatus.ServiceTypeListenerStatus status = list[i];
                    this._executor.submit(new Runnable() {
                        public void run() {
                            status.serviceTypeAdded(event);
                        }
                    });
                    i++;
                    typeAdded2 = typeAdded2;
                }
            }
            typeAdded = typeAdded2;
        }
        if (subtype.length() > 0 && (subtypes = (ServiceTypeEntry) this._serviceTypes.get(loname)) != null && !subtypes.contains(subtype)) {
            synchronized (subtypes) {
                if (!subtypes.contains(subtype)) {
                    typeAdded = true;
                    subtypes.add(subtype);
                    ListenerStatus.ServiceTypeListenerStatus[] list2 = (ListenerStatus.ServiceTypeListenerStatus[]) this._typeListeners.toArray(new ListenerStatus.ServiceTypeListenerStatus[this._typeListeners.size()]);
                    final ServiceEvent event2 = new ServiceEventImpl(this, "_" + subtype + "._sub." + name, "", (ServiceInfo) null);
                    int length2 = list2.length;
                    int i2 = 0;
                    while (i2 < length2) {
                        final ListenerStatus.ServiceTypeListenerStatus status2 = list2[i2];
                        this._executor.submit(new Runnable() {
                            public void run() {
                                status2.subTypeForServiceTypeAdded(event2);
                            }
                        });
                        i2++;
                        list2 = list2;
                    }
                }
            }
        }
        return typeAdded;
    }

    /* JADX WARNING: Code restructure failed: missing block: B:13:0x005c, code lost:
        if (logger.isLoggable(java.util.logging.Level.FINER) == false) goto L_0x00a4;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:14:0x005e, code lost:
        r4 = logger;
        r4.finer("makeServiceNameUnique() JmDNS.makeServiceNameUnique srv collision:" + r5 + " s.server=" + r6.getServer() + " " + r10._localHost.getName() + " equals:" + r6.getServer().equals(r10._localHost.getName()));
     */
    /* JADX WARNING: Code restructure failed: missing block: B:15:0x00a4, code lost:
        r11.setName(incrementName(r11.getName()));
        r3 = true;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    private boolean makeServiceNameUnique(javax.jmdns.impl.ServiceInfoImpl r11) {
        /*
            r10 = this;
            java.lang.String r0 = r11.getKey()
            long r1 = java.lang.System.currentTimeMillis()
        L_0x0008:
            r3 = 0
            javax.jmdns.impl.DNSCache r4 = r10.getCache()
            java.lang.String r5 = r11.getKey()
            java.util.Collection r4 = r4.getDNSEntryList(r5)
            java.util.Iterator r4 = r4.iterator()
        L_0x0019:
            boolean r5 = r4.hasNext()
            if (r5 == 0) goto L_0x00b3
            java.lang.Object r5 = r4.next()
            javax.jmdns.impl.DNSEntry r5 = (javax.jmdns.impl.DNSEntry) r5
            javax.jmdns.impl.constants.DNSRecordType r6 = javax.jmdns.impl.constants.DNSRecordType.TYPE_SRV
            javax.jmdns.impl.constants.DNSRecordType r7 = r5.getRecordType()
            boolean r6 = r6.equals(r7)
            if (r6 == 0) goto L_0x00b1
            boolean r6 = r5.isExpired(r1)
            if (r6 != 0) goto L_0x00b1
            r6 = r5
            javax.jmdns.impl.DNSRecord$Service r6 = (javax.jmdns.impl.DNSRecord.Service) r6
            int r7 = r6.getPort()
            int r8 = r11.getPort()
            if (r7 != r8) goto L_0x0054
            java.lang.String r7 = r6.getServer()
            javax.jmdns.impl.HostInfo r8 = r10._localHost
            java.lang.String r8 = r8.getName()
            boolean r7 = r7.equals(r8)
            if (r7 != 0) goto L_0x00b1
        L_0x0054:
            java.util.logging.Logger r4 = logger
            java.util.logging.Level r7 = java.util.logging.Level.FINER
            boolean r4 = r4.isLoggable(r7)
            if (r4 == 0) goto L_0x00a4
            java.util.logging.Logger r4 = logger
            java.lang.StringBuilder r7 = new java.lang.StringBuilder
            r7.<init>()
            java.lang.String r8 = "makeServiceNameUnique() JmDNS.makeServiceNameUnique srv collision:"
            r7.append(r8)
            r7.append(r5)
            java.lang.String r8 = " s.server="
            r7.append(r8)
            java.lang.String r8 = r6.getServer()
            r7.append(r8)
            java.lang.String r8 = " "
            r7.append(r8)
            javax.jmdns.impl.HostInfo r8 = r10._localHost
            java.lang.String r8 = r8.getName()
            r7.append(r8)
            java.lang.String r8 = " equals:"
            r7.append(r8)
            java.lang.String r8 = r6.getServer()
            javax.jmdns.impl.HostInfo r9 = r10._localHost
            java.lang.String r9 = r9.getName()
            boolean r8 = r8.equals(r9)
            r7.append(r8)
            java.lang.String r7 = r7.toString()
            r4.finer(r7)
        L_0x00a4:
            java.lang.String r4 = r11.getName()
            java.lang.String r4 = r10.incrementName(r4)
            r11.setName(r4)
            r3 = 1
            goto L_0x00b3
        L_0x00b1:
            goto L_0x0019
        L_0x00b3:
            java.util.concurrent.ConcurrentMap<java.lang.String, javax.jmdns.ServiceInfo> r4 = r10._services
            java.lang.String r5 = r11.getKey()
            java.lang.Object r4 = r4.get(r5)
            javax.jmdns.ServiceInfo r4 = (javax.jmdns.ServiceInfo) r4
            if (r4 == 0) goto L_0x00cf
            if (r4 == r11) goto L_0x00cf
            java.lang.String r5 = r11.getName()
            java.lang.String r5 = r10.incrementName(r5)
            r11.setName(r5)
            r3 = 1
        L_0x00cf:
            if (r3 != 0) goto L_0x0008
            java.lang.String r4 = r11.getKey()
            boolean r4 = r0.equals(r4)
            r4 = r4 ^ 1
            return r4
        */
        throw new UnsupportedOperationException("Method not decompiled: javax.jmdns.impl.JmDNSImpl.makeServiceNameUnique(javax.jmdns.impl.ServiceInfoImpl):boolean");
    }

    /* access modifiers changed from: package-private */
    public String incrementName(String name) {
        String aName = name;
        try {
            int l = aName.lastIndexOf(40);
            int r = aName.lastIndexOf(41);
            if (l < 0 || l >= r) {
                return aName + " (2)";
            }
            return aName.substring(0, l) + "(" + (Integer.parseInt(aName.substring(l + 1, r)) + 1) + ")";
        } catch (NumberFormatException e) {
            return aName + " (2)";
        }
    }

    public void addListener(DNSListener listener, DNSQuestion question) {
        long now = System.currentTimeMillis();
        this._listeners.add(listener);
        if (question != null) {
            for (DNSEntry dnsEntry : getCache().getDNSEntryList(question.getName().toLowerCase())) {
                if (question.answeredBy(dnsEntry) && !dnsEntry.isExpired(now)) {
                    listener.updateRecord(getCache(), now, dnsEntry);
                }
            }
        }
    }

    public void removeListener(DNSListener listener) {
        this._listeners.remove(listener);
    }

    public void renewServiceCollector(DNSRecord record) {
        ServiceInfo info = record.getServiceInfo();
        if (this._serviceCollectors.containsKey(info.getType().toLowerCase())) {
            startServiceResolver(info.getType());
        }
    }

    public void updateRecord(long now, DNSRecord rec, Operation operation) {
        List<DNSListener> listenerList;
        List<ListenerStatus.ServiceListenerStatus> serviceListenerList;
        synchronized (this._listeners) {
            listenerList = new ArrayList<>(this._listeners);
        }
        for (DNSListener listener : listenerList) {
            listener.updateRecord(getCache(), now, rec);
        }
        if (DNSRecordType.TYPE_PTR.equals(rec.getRecordType())) {
            ServiceEvent event = rec.getServiceEvent(this);
            if (event.getInfo() == null || !event.getInfo().hasData()) {
                ServiceInfo info = getServiceInfoFromCache(event.getType(), event.getName(), "", false);
                if (info.hasData()) {
                    event = new ServiceEventImpl(this, event.getType(), event.getName(), info);
                }
            }
            List<ListenerStatus.ServiceListenerStatus> list = (List) this._serviceListeners.get(event.getType().toLowerCase());
            if (list != null) {
                synchronized (list) {
                    serviceListenerList = new ArrayList<>(list);
                }
            } else {
                serviceListenerList = Collections.emptyList();
            }
            if (logger.isLoggable(Level.FINEST)) {
                Logger logger2 = logger;
                logger2.finest(getName() + ".updating record for event: " + event + " list " + serviceListenerList + " operation: " + operation);
            }
            if (!serviceListenerList.isEmpty()) {
                final ServiceEvent localEvent = event;
                switch (operation) {
                    case Add:
                        for (final ListenerStatus.ServiceListenerStatus listener2 : serviceListenerList) {
                            if (listener2.isSynchronous()) {
                                listener2.serviceAdded(localEvent);
                            } else {
                                this._executor.submit(new Runnable() {
                                    public void run() {
                                        listener2.serviceAdded(localEvent);
                                    }
                                });
                            }
                        }
                        return;
                    case Remove:
                        for (final ListenerStatus.ServiceListenerStatus listener3 : serviceListenerList) {
                            if (listener3.isSynchronous()) {
                                listener3.serviceRemoved(localEvent);
                            } else {
                                this._executor.submit(new Runnable() {
                                    public void run() {
                                        listener3.serviceRemoved(localEvent);
                                    }
                                });
                            }
                        }
                        return;
                    default:
                        return;
                }
            }
        }
    }

    /* access modifiers changed from: package-private */
    public void handleRecord(DNSRecord record, long now) {
        DNSRecord newRecord = record;
        Operation cacheOperation = Operation.Noop;
        boolean expired = newRecord.isExpired(now);
        if (logger.isLoggable(Level.FINE)) {
            Logger logger2 = logger;
            logger2.fine(getName() + " handle response: " + newRecord);
        }
        if (!newRecord.isServicesDiscoveryMetaQuery() && !newRecord.isDomainDiscoveryQuery()) {
            boolean unique = newRecord.isUnique();
            DNSRecord cachedRecord = (DNSRecord) getCache().getDNSEntry(newRecord);
            if (logger.isLoggable(Level.FINE)) {
                Logger logger3 = logger;
                logger3.fine(getName() + " handle response cached record: " + cachedRecord);
            }
            if (unique) {
                for (DNSEntry entry : getCache().getDNSEntryList(newRecord.getKey())) {
                    if (newRecord.getRecordType().equals(entry.getRecordType()) && newRecord.getRecordClass().equals(entry.getRecordClass()) && entry != cachedRecord) {
                        ((DNSRecord) entry).setWillExpireSoon(now);
                    }
                }
            }
            if (cachedRecord != null) {
                if (expired) {
                    if (newRecord.getTTL() == 0) {
                        cacheOperation = Operation.Noop;
                        cachedRecord.setWillExpireSoon(now);
                    } else {
                        cacheOperation = Operation.Remove;
                        getCache().removeDNSEntry(cachedRecord);
                    }
                } else if (newRecord.sameValue(cachedRecord) && (newRecord.sameSubtype(cachedRecord) || newRecord.getSubtype().length() <= 0)) {
                    cachedRecord.resetTTL(newRecord);
                    newRecord = cachedRecord;
                } else if (newRecord.isSingleValued()) {
                    cacheOperation = Operation.Update;
                    getCache().replaceDNSEntry(newRecord, cachedRecord);
                } else {
                    cacheOperation = Operation.Add;
                    getCache().addDNSEntry(newRecord);
                }
            } else if (!expired) {
                cacheOperation = Operation.Add;
                getCache().addDNSEntry(newRecord);
            }
        }
        if (newRecord.getRecordType() == DNSRecordType.TYPE_PTR) {
            if (newRecord.isServicesDiscoveryMetaQuery()) {
                if (!expired) {
                    boolean typeAdded = registerServiceType(((DNSRecord.Pointer) newRecord).getAlias());
                    return;
                }
                return;
            } else if ((false || registerServiceType(newRecord.getName())) && cacheOperation == Operation.Noop) {
                cacheOperation = Operation.RegisterServiceType;
            }
        }
        if (cacheOperation != Operation.Noop) {
            updateRecord(now, newRecord, cacheOperation);
        }
    }

    /* access modifiers changed from: package-private */
    public void handleResponse(DNSIncoming msg) throws IOException {
        long now = System.currentTimeMillis();
        boolean hostConflictDetected = false;
        boolean serviceConflictDetected = false;
        String hostAddress = getLocalHost().getInetAddress().getHostAddress();
        for (DNSRecord newRecord : msg.getAllAnswers()) {
            String dNSRecord = newRecord.toString();
            handleRecord(newRecord, now);
            if (DNSRecordType.TYPE_A.equals(newRecord.getRecordType()) || DNSRecordType.TYPE_AAAA.equals(newRecord.getRecordType())) {
                hostConflictDetected |= newRecord.handleResponse(this);
            } else {
                serviceConflictDetected |= newRecord.handleResponse(this);
            }
        }
        if (hostConflictDetected || serviceConflictDetected) {
            startProber();
        }
    }

    /* JADX INFO: finally extract failed */
    /* access modifiers changed from: package-private */
    public void handleQuery(DNSIncoming in, InetAddress addr, int port) throws IOException {
        if (logger.isLoggable(Level.FINE)) {
            Logger logger2 = logger;
            logger2.fine(getName() + ".handle query: " + in);
        }
        boolean conflictDetected = false;
        long expirationTime = System.currentTimeMillis() + 120;
        for (DNSRecord answer : in.getAllAnswers()) {
            conflictDetected |= answer.handleQuery(this, expirationTime);
        }
        ioLock();
        try {
            if (this._plannedAnswer != null) {
                this._plannedAnswer.append(in);
            } else {
                DNSIncoming plannedAnswer = in.clone();
                if (in.isTruncated()) {
                    this._plannedAnswer = plannedAnswer;
                }
                startResponder(plannedAnswer, port);
            }
            ioUnlock();
            long now = System.currentTimeMillis();
            for (DNSRecord answer2 : in.getAnswers()) {
                handleRecord(answer2, now);
            }
            if (conflictDetected) {
                startProber();
            }
        } catch (Throwable th) {
            ioUnlock();
            throw th;
        }
    }

    public void respondToQuery(DNSIncoming in) {
        ioLock();
        try {
            if (this._plannedAnswer == in) {
                this._plannedAnswer = null;
            }
        } finally {
            ioUnlock();
        }
    }

    public DNSOutgoing addAnswer(DNSIncoming in, InetAddress addr, int port, DNSOutgoing out, DNSRecord rec) throws IOException {
        DNSOutgoing newOut = out;
        if (newOut == null) {
            newOut = new DNSOutgoing(33792, false, in.getSenderUDPPayload());
        }
        try {
            newOut.addAnswer(in, rec);
            return newOut;
        } catch (IOException e) {
            newOut.setFlags(newOut.getFlags() | 512);
            newOut.setId(in.getId());
            send(newOut);
            DNSOutgoing newOut2 = new DNSOutgoing(33792, false, in.getSenderUDPPayload());
            newOut2.addAnswer(in, rec);
            return newOut2;
        }
    }

    public void send(DNSOutgoing out) throws IOException {
        if (!out.isEmpty()) {
            byte[] message = out.data();
            DatagramPacket packet = new DatagramPacket(message, message.length, this._group, DNSConstants.MDNS_PORT);
            if (logger.isLoggable(Level.FINEST)) {
                try {
                    DNSIncoming msg = new DNSIncoming(packet);
                    if (logger.isLoggable(Level.FINEST)) {
                        Logger logger2 = logger;
                        logger2.finest("send(" + getName() + ") JmDNS out:" + msg.print(true));
                    }
                } catch (IOException e) {
                    Logger logger3 = logger;
                    String cls = getClass().toString();
                    logger3.throwing(cls, "send(" + getName() + ") - JmDNS can not parse what it sends!!!", e);
                }
            }
            MulticastSocket ms = this._socket;
            if (ms != null && !ms.isClosed()) {
                ms.send(packet);
            }
        }
    }

    public void purgeTimer() {
        DNSTaskStarter.Factory.getInstance().getStarter(getDns()).purgeTimer();
    }

    public void purgeStateTimer() {
        DNSTaskStarter.Factory.getInstance().getStarter(getDns()).purgeStateTimer();
    }

    public void cancelTimer() {
        DNSTaskStarter.Factory.getInstance().getStarter(getDns()).cancelTimer();
    }

    public void cancelStateTimer() {
        DNSTaskStarter.Factory.getInstance().getStarter(getDns()).cancelStateTimer();
    }

    public void startProber() {
        DNSTaskStarter.Factory.getInstance().getStarter(getDns()).startProber();
    }

    public void startAnnouncer() {
        DNSTaskStarter.Factory.getInstance().getStarter(getDns()).startAnnouncer();
    }

    public void startRenewer() {
        DNSTaskStarter.Factory.getInstance().getStarter(getDns()).startRenewer();
    }

    public void startCanceler() {
        DNSTaskStarter.Factory.getInstance().getStarter(getDns()).startCanceler();
    }

    public void startReaper() {
        DNSTaskStarter.Factory.getInstance().getStarter(getDns()).startReaper();
    }

    public void startServiceInfoResolver(ServiceInfoImpl info) {
        DNSTaskStarter.Factory.getInstance().getStarter(getDns()).startServiceInfoResolver(info);
    }

    public void startTypeResolver() {
        DNSTaskStarter.Factory.getInstance().getStarter(getDns()).startTypeResolver();
    }

    public void startServiceResolver(String type) {
        DNSTaskStarter.Factory.getInstance().getStarter(getDns()).startServiceResolver(type);
    }

    public void startResponder(DNSIncoming in, int port) {
        DNSTaskStarter.Factory.getInstance().getStarter(getDns()).startResponder(in, port);
    }

    protected class Shutdown implements Runnable {
        protected Shutdown() {
        }

        public void run() {
            try {
                JmDNSImpl.this._shutdown = null;
                JmDNSImpl.this.close();
            } catch (Throwable exception) {
                PrintStream printStream = System.err;
                printStream.println("Error while shuting down. " + exception);
            }
        }
    }

    public void recover() {
        Logger logger2 = logger;
        logger2.finer(getName() + "recover()");
        if (!isClosing() && !isClosed() && !isCanceling() && !isCanceled()) {
            synchronized (this._recoverLock) {
                if (cancelState()) {
                    Logger logger3 = logger;
                    logger3.finer(getName() + "recover() thread " + Thread.currentThread().getName());
                    StringBuilder sb = new StringBuilder();
                    sb.append(getName());
                    sb.append(".recover()");
                    new Thread(sb.toString()) {
                        public void run() {
                            JmDNSImpl.this.__recover();
                        }
                    }.start();
                }
            }
        }
    }

    /* access modifiers changed from: package-private */
    public void __recover() {
        if (logger.isLoggable(Level.FINER)) {
            Logger logger2 = logger;
            logger2.finer(getName() + "recover() Cleanning up");
        }
        logger.warning("RECOVERING");
        purgeTimer();
        Collection<ServiceInfo> oldServiceInfos = new ArrayList<>(getServices().values());
        unregisterAllServices();
        disposeServiceCollectors();
        waitForCanceled(DNSConstants.CLOSE_TIMEOUT);
        purgeStateTimer();
        closeMulticastSocket();
        getCache().clear();
        if (logger.isLoggable(Level.FINER)) {
            Logger logger3 = logger;
            logger3.finer(getName() + "recover() All is clean");
        }
        if (isCanceled()) {
            Iterator<ServiceInfo> it = oldServiceInfos.iterator();
            while (it.hasNext()) {
                ((ServiceInfoImpl) it.next()).recoverState();
            }
            recoverState();
            try {
                openMulticastSocket(getLocalHost());
                start(oldServiceInfos);
            } catch (Exception exception) {
                Logger logger4 = logger;
                Level level = Level.WARNING;
                logger4.log(level, getName() + "recover() Start services exception ", exception);
            }
            Logger logger5 = logger;
            Level level2 = Level.WARNING;
            logger5.log(level2, getName() + "recover() We are back!");
            return;
        }
        Logger logger6 = logger;
        Level level3 = Level.WARNING;
        logger6.log(level3, getName() + "recover() Could not recover we are Down!");
        if (getDelegate() != null) {
            getDelegate().cannotRecoverFromIOError(getDns(), oldServiceInfos);
        }
    }

    public void cleanCache() {
        long now = System.currentTimeMillis();
        for (DNSEntry entry : getCache().allValues()) {
            try {
                DNSRecord record = (DNSRecord) entry;
                if (record.isExpired(now)) {
                    updateRecord(now, record, Operation.Remove);
                    getCache().removeDNSEntry(record);
                } else if (record.isStale(now)) {
                    renewServiceCollector(record);
                }
            } catch (Exception exception) {
                Logger logger2 = logger;
                Level level = Level.SEVERE;
                logger2.log(level, getName() + ".Error while reaping records: " + entry, exception);
                logger.severe(toString());
            }
        }
    }

    public void close() {
        if (!isClosing()) {
            if (logger.isLoggable(Level.FINER)) {
                Logger logger2 = logger;
                logger2.finer("Cancelling JmDNS: " + this);
            }
            if (closeState()) {
                logger.finer("Canceling the timer");
                cancelTimer();
                unregisterAllServices();
                disposeServiceCollectors();
                if (logger.isLoggable(Level.FINER)) {
                    Logger logger3 = logger;
                    logger3.finer("Wait for JmDNS cancel: " + this);
                }
                waitForCanceled(DNSConstants.CLOSE_TIMEOUT);
                logger.finer("Canceling the state timer");
                cancelStateTimer();
                this._executor.shutdown();
                closeMulticastSocket();
                if (this._shutdown != null) {
                    Runtime.getRuntime().removeShutdownHook(this._shutdown);
                }
                if (logger.isLoggable(Level.FINER)) {
                    logger.finer("JmDNS closed.");
                }
            }
            advanceState((DNSTask) null);
        }
    }

    @Deprecated
    public void printServices() {
        System.err.println(toString());
    }

    public String toString() {
        StringBuilder aLog = new StringBuilder(2048);
        aLog.append("\t---- Local Host -----");
        aLog.append("\n\t");
        aLog.append(this._localHost);
        aLog.append("\n\t---- Services -----");
        for (String key : this._services.keySet()) {
            aLog.append("\n\t\tService: ");
            aLog.append(key);
            aLog.append(": ");
            aLog.append(this._services.get(key));
        }
        aLog.append("\n");
        aLog.append("\t---- Types ----");
        for (String key2 : this._serviceTypes.keySet()) {
            ServiceTypeEntry subtypes = (ServiceTypeEntry) this._serviceTypes.get(key2);
            aLog.append("\n\t\tType: ");
            aLog.append(subtypes.getType());
            aLog.append(": ");
            aLog.append(subtypes.isEmpty() ? "no subtypes" : subtypes);
        }
        aLog.append("\n");
        aLog.append(this._cache.toString());
        aLog.append("\n");
        aLog.append("\t---- Service Collectors ----");
        for (String key3 : this._serviceCollectors.keySet()) {
            aLog.append("\n\t\tService Collector: ");
            aLog.append(key3);
            aLog.append(": ");
            aLog.append(this._serviceCollectors.get(key3));
        }
        aLog.append("\n");
        aLog.append("\t---- Service Listeners ----");
        for (String key4 : this._serviceListeners.keySet()) {
            aLog.append("\n\t\tService Listener: ");
            aLog.append(key4);
            aLog.append(": ");
            aLog.append(this._serviceListeners.get(key4));
        }
        return aLog.toString();
    }

    public ServiceInfo[] list(String type) {
        return list(type, DNSConstants.SERVICE_INFO_TIMEOUT);
    }

    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r4v7, resolved type: java.lang.Object} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r2v7, resolved type: javax.jmdns.impl.JmDNSImpl$ServiceCollector} */
    /* JADX WARNING: Multi-variable type inference failed */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public javax.jmdns.ServiceInfo[] list(java.lang.String r8, long r9) {
        /*
            r7 = this;
            r7.cleanCache()
            java.lang.String r0 = r8.toLowerCase()
            r1 = 0
            boolean r2 = r7.isCanceling()
            r3 = 0
            if (r2 != 0) goto L_0x0071
            boolean r2 = r7.isCanceled()
            if (r2 == 0) goto L_0x0016
            goto L_0x0071
        L_0x0016:
            java.util.concurrent.ConcurrentMap<java.lang.String, javax.jmdns.impl.JmDNSImpl$ServiceCollector> r2 = r7._serviceCollectors
            java.lang.Object r2 = r2.get(r0)
            javax.jmdns.impl.JmDNSImpl$ServiceCollector r2 = (javax.jmdns.impl.JmDNSImpl.ServiceCollector) r2
            if (r2 != 0) goto L_0x0040
            java.util.concurrent.ConcurrentMap<java.lang.String, javax.jmdns.impl.JmDNSImpl$ServiceCollector> r4 = r7._serviceCollectors
            javax.jmdns.impl.JmDNSImpl$ServiceCollector r5 = new javax.jmdns.impl.JmDNSImpl$ServiceCollector
            r5.<init>(r8)
            java.lang.Object r4 = r4.putIfAbsent(r0, r5)
            r5 = 1
            if (r4 != 0) goto L_0x0030
            r4 = 1
            goto L_0x0031
        L_0x0030:
            r4 = 0
        L_0x0031:
            r1 = r4
            java.util.concurrent.ConcurrentMap<java.lang.String, javax.jmdns.impl.JmDNSImpl$ServiceCollector> r4 = r7._serviceCollectors
            java.lang.Object r4 = r4.get(r0)
            r2 = r4
            javax.jmdns.impl.JmDNSImpl$ServiceCollector r2 = (javax.jmdns.impl.JmDNSImpl.ServiceCollector) r2
            if (r1 == 0) goto L_0x0040
            r7.addServiceListener(r8, r2, r5)
        L_0x0040:
            java.util.logging.Logger r4 = logger
            java.util.logging.Level r5 = java.util.logging.Level.FINER
            boolean r4 = r4.isLoggable(r5)
            if (r4 == 0) goto L_0x0067
            java.util.logging.Logger r4 = logger
            java.lang.StringBuilder r5 = new java.lang.StringBuilder
            r5.<init>()
            java.lang.String r6 = r7.getName()
            r5.append(r6)
            java.lang.String r6 = ".collector: "
            r5.append(r6)
            r5.append(r2)
            java.lang.String r5 = r5.toString()
            r4.finer(r5)
        L_0x0067:
            if (r2 == 0) goto L_0x006e
            javax.jmdns.ServiceInfo[] r3 = r2.list(r9)
            goto L_0x0070
        L_0x006e:
            javax.jmdns.ServiceInfo[] r3 = new javax.jmdns.ServiceInfo[r3]
        L_0x0070:
            return r3
        L_0x0071:
            javax.jmdns.ServiceInfo[] r2 = new javax.jmdns.ServiceInfo[r3]
            return r2
        */
        throw new UnsupportedOperationException("Method not decompiled: javax.jmdns.impl.JmDNSImpl.list(java.lang.String, long):javax.jmdns.ServiceInfo[]");
    }

    public Map<String, ServiceInfo[]> listBySubtype(String type) {
        return listBySubtype(type, DNSConstants.SERVICE_INFO_TIMEOUT);
    }

    public Map<String, ServiceInfo[]> listBySubtype(String type, long timeout) {
        Map<String, List<ServiceInfo>> map = new HashMap<>(5);
        for (ServiceInfo info : list(type, timeout)) {
            String subtype = info.getSubtype().toLowerCase();
            if (!map.containsKey(subtype)) {
                map.put(subtype, new ArrayList(10));
            }
            map.get(subtype).add(info);
        }
        Map<String, ServiceInfo[]> result = new HashMap<>(map.size());
        for (String subtype2 : map.keySet()) {
            List<ServiceInfo> infoForSubType = map.get(subtype2);
            result.put(subtype2, infoForSubType.toArray(new ServiceInfo[infoForSubType.size()]));
        }
        return result;
    }

    private void disposeServiceCollectors() {
        if (logger.isLoggable(Level.FINER)) {
            logger.finer("disposeServiceCollectors()");
        }
        for (String type : this._serviceCollectors.keySet()) {
            ServiceCollector collector = (ServiceCollector) this._serviceCollectors.get(type);
            if (collector != null) {
                removeServiceListener(type, collector);
                this._serviceCollectors.remove(type, collector);
            }
        }
    }

    private static class ServiceCollector implements ServiceListener {
        private final ConcurrentMap<String, ServiceEvent> _events = new ConcurrentHashMap();
        private final ConcurrentMap<String, ServiceInfo> _infos = new ConcurrentHashMap();
        private volatile boolean _needToWaitForInfos;
        private final String _type;

        public ServiceCollector(String type) {
            this._type = type;
            this._needToWaitForInfos = true;
        }

        public void serviceAdded(ServiceEvent event) {
            synchronized (this) {
                ServiceInfo info = event.getInfo();
                if (info == null || !info.hasData()) {
                    ServiceInfo info2 = ((JmDNSImpl) event.getDNS()).resolveServiceInfo(event.getType(), event.getName(), info != null ? info.getSubtype() : "", true);
                    if (info2 != null) {
                        this._infos.put(event.getName(), info2);
                    } else {
                        this._events.put(event.getName(), event);
                    }
                } else {
                    this._infos.put(event.getName(), info);
                }
            }
        }

        public void serviceRemoved(ServiceEvent event) {
            synchronized (this) {
                this._infos.remove(event.getName());
                this._events.remove(event.getName());
            }
        }

        public void serviceResolved(ServiceEvent event) {
            synchronized (this) {
                this._infos.put(event.getName(), event.getInfo());
                this._events.remove(event.getName());
            }
        }

        public ServiceInfo[] list(long timeout) {
            if (this._infos.isEmpty() || !this._events.isEmpty() || this._needToWaitForInfos) {
                long loops = timeout / 200;
                if (loops < 1) {
                    loops = 1;
                }
                for (int i = 0; ((long) i) < loops; i++) {
                    try {
                        Thread.sleep(200);
                    } catch (InterruptedException e) {
                    }
                    if (this._events.isEmpty() && !this._infos.isEmpty() && !this._needToWaitForInfos) {
                        break;
                    }
                }
            }
            this._needToWaitForInfos = false;
            return (ServiceInfo[]) this._infos.values().toArray(new ServiceInfo[this._infos.size()]);
        }

        public String toString() {
            StringBuffer aLog = new StringBuffer();
            aLog.append("\n\tType: ");
            aLog.append(this._type);
            if (this._infos.isEmpty()) {
                aLog.append("\n\tNo services collected.");
            } else {
                aLog.append("\n\tServices");
                for (String key : this._infos.keySet()) {
                    aLog.append("\n\t\tService: ");
                    aLog.append(key);
                    aLog.append(": ");
                    aLog.append(this._infos.get(key));
                }
            }
            if (this._events.isEmpty()) {
                aLog.append("\n\tNo event queued.");
            } else {
                aLog.append("\n\tEvents");
                for (String key2 : this._events.keySet()) {
                    aLog.append("\n\t\tEvent: ");
                    aLog.append(key2);
                    aLog.append(": ");
                    aLog.append(this._events.get(key2));
                }
            }
            return aLog.toString();
        }
    }

    static String toUnqualifiedName(String type, String qualifiedName) {
        String loType = type.toLowerCase();
        String loQualifiedName = qualifiedName.toLowerCase();
        if (!loQualifiedName.endsWith(loType) || loQualifiedName.equals(loType)) {
            return qualifiedName;
        }
        return qualifiedName.substring(0, (qualifiedName.length() - type.length()) - 1);
    }

    public Map<String, ServiceInfo> getServices() {
        return this._services;
    }

    public void setLastThrottleIncrement(long lastThrottleIncrement) {
        this._lastThrottleIncrement = lastThrottleIncrement;
    }

    public long getLastThrottleIncrement() {
        return this._lastThrottleIncrement;
    }

    public void setThrottle(int throttle) {
        this._throttle = throttle;
    }

    public int getThrottle() {
        return this._throttle;
    }

    public static Random getRandom() {
        return _random;
    }

    public void ioLock() {
        this._ioLock.lock();
    }

    public void ioUnlock() {
        this._ioLock.unlock();
    }

    public void setPlannedAnswer(DNSIncoming plannedAnswer) {
        this._plannedAnswer = plannedAnswer;
    }

    public DNSIncoming getPlannedAnswer() {
        return this._plannedAnswer;
    }

    /* access modifiers changed from: package-private */
    public void setLocalHost(HostInfo localHost) {
        this._localHost = localHost;
    }

    public Map<String, ServiceTypeEntry> getServiceTypes() {
        return this._serviceTypes;
    }

    public MulticastSocket getSocket() {
        return this._socket;
    }

    public InetAddress getGroup() {
        return this._group;
    }

    public JmDNS.Delegate getDelegate() {
        return this._delegate;
    }

    public JmDNS.Delegate setDelegate(JmDNS.Delegate delegate) {
        JmDNS.Delegate previous = this._delegate;
        this._delegate = delegate;
        return previous;
    }
}
