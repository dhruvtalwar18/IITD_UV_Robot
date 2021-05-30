package javax.jmdns;

import java.io.Closeable;
import java.io.IOException;
import java.net.InetAddress;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;
import javax.jmdns.impl.JmmDNSImpl;

public interface JmmDNS extends Closeable {
    void addNetworkTopologyListener(NetworkTopologyListener networkTopologyListener);

    void addServiceListener(String str, ServiceListener serviceListener);

    void addServiceTypeListener(ServiceTypeListener serviceTypeListener) throws IOException;

    String[] getHostNames();

    InetAddress[] getInetAddresses() throws IOException;

    @Deprecated
    InetAddress[] getInterfaces() throws IOException;

    String[] getNames();

    ServiceInfo[] getServiceInfos(String str, String str2);

    ServiceInfo[] getServiceInfos(String str, String str2, long j);

    ServiceInfo[] getServiceInfos(String str, String str2, boolean z);

    ServiceInfo[] getServiceInfos(String str, String str2, boolean z, long j);

    boolean isClosed();

    ServiceInfo[] list(String str);

    ServiceInfo[] list(String str, long j);

    Map<String, ServiceInfo[]> listBySubtype(String str);

    Map<String, ServiceInfo[]> listBySubtype(String str, long j);

    NetworkTopologyListener[] networkListeners();

    void registerService(ServiceInfo serviceInfo) throws IOException;

    void registerServiceType(String str);

    void removeNetworkTopologyListener(NetworkTopologyListener networkTopologyListener);

    void removeServiceListener(String str, ServiceListener serviceListener);

    void removeServiceTypeListener(ServiceTypeListener serviceTypeListener);

    void requestServiceInfo(String str, String str2);

    void requestServiceInfo(String str, String str2, long j);

    void requestServiceInfo(String str, String str2, boolean z);

    void requestServiceInfo(String str, String str2, boolean z, long j);

    void unregisterAllServices();

    void unregisterService(ServiceInfo serviceInfo);

    public static final class Factory {
        private static final AtomicReference<ClassDelegate> _databaseClassDelegate = new AtomicReference<>();
        private static volatile JmmDNS _instance;

        public interface ClassDelegate {
            JmmDNS newJmmDNS();
        }

        private Factory() {
        }

        public static void setClassDelegate(ClassDelegate delegate) {
            _databaseClassDelegate.set(delegate);
        }

        public static ClassDelegate classDelegate() {
            return _databaseClassDelegate.get();
        }

        protected static JmmDNS newJmmDNS() {
            JmmDNS dns = null;
            ClassDelegate delegate = _databaseClassDelegate.get();
            if (delegate != null) {
                dns = delegate.newJmmDNS();
            }
            return dns != null ? dns : new JmmDNSImpl();
        }

        public static JmmDNS getInstance() {
            if (_instance == null || _instance.isClosed()) {
                synchronized (Factory.class) {
                    _instance = newJmmDNS();
                }
            }
            return _instance;
        }
    }
}
