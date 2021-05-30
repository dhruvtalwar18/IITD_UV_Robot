package com.github.rosjava.zeroconf_jmdns_suite.jmdns;

import java.io.IOException;
import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Set;
import javax.jmdns.JmmDNS;
import javax.jmdns.NetworkTopologyEvent;
import javax.jmdns.NetworkTopologyListener;
import javax.jmdns.ServiceEvent;
import javax.jmdns.ServiceInfo;
import javax.jmdns.ServiceListener;
import javax.jmdns.ServiceTypeListener;

public class Zeroconf implements ServiceListener, ServiceTypeListener, NetworkTopologyListener {
    ZeroconfDiscoveryHandler default_listener_callback;
    JmmDNS jmmdns;
    Map<String, ZeroconfDiscoveryHandler> listener_callbacks;
    Set<String> listeners;
    ZeroconfLogger logger;
    Set<ServiceInfo> services;

    private class DefaultLogger implements ZeroconfLogger {
        private DefaultLogger() {
        }

        public void println(String msg) {
        }
    }

    public Zeroconf() {
        this.jmmdns = JmmDNS.Factory.getInstance();
        this.listeners = new HashSet();
        this.services = new HashSet();
        this.logger = new DefaultLogger();
        this.listener_callbacks = new HashMap();
        this.default_listener_callback = null;
        this.jmmdns.addNetworkTopologyListener(this);
    }

    public Zeroconf(ZeroconfLogger logger2) {
        this.jmmdns = JmmDNS.Factory.getInstance();
        this.listeners = new HashSet();
        this.services = new HashSet();
        this.logger = logger2;
        this.listener_callbacks = new HashMap();
        this.default_listener_callback = null;
        this.jmmdns.addNetworkTopologyListener(this);
    }

    public void setDefaultDiscoveryCallback(ZeroconfDiscoveryHandler listener_callback) {
        this.default_listener_callback = listener_callback;
    }

    public void addListener(String service_type, String domain) {
        addListener(service_type, domain, this.default_listener_callback);
    }

    public void addListener(String service_type, String domain, ZeroconfDiscoveryHandler listener_callback) {
        String service = service_type + "." + domain + ".";
        this.logger.println("Activating listener: " + service);
        this.listeners.add(service);
        if (listener_callback != null) {
            this.listener_callbacks.put(service, listener_callback);
        }
        this.jmmdns.addServiceListener(service, this);
    }

    public void removeListener(String service_type, String domain) {
        String listener_to_remove = service_type + "." + domain + ".";
        Iterator<String> listener = this.listeners.iterator();
        while (true) {
            if (!listener.hasNext()) {
                break;
            }
            String this_listener = listener.next().toString();
            if (this_listener.equals(listener_to_remove)) {
                this.logger.println("Deactivating listener: " + this_listener);
                listener.remove();
                this.jmmdns.removeServiceListener(listener_to_remove, this);
                break;
            }
        }
        this.listener_callbacks.remove(listener_to_remove);
    }

    public void addService(String name, String type, String domain, int port, String description) {
        String full_service_type = type + "." + domain + ".";
        this.logger.println("Registering service: " + full_service_type);
        HashMap hashMap = new HashMap();
        hashMap.put("description", description.getBytes());
        ServiceInfo service_info = ServiceInfo.create(full_service_type, name, port, 0, 0, true, (Map<String, ?>) hashMap);
        if (this.services.add(service_info)) {
            try {
                this.jmmdns.registerService(service_info);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    public List<DiscoveredService> listDiscoveredServices() {
        List<ServiceInfo> service_infos = new ArrayList<>();
        for (String service : this.listeners) {
            service_infos.addAll(Arrays.asList(this.jmmdns.list(service)));
        }
        List<DiscoveredService> discovered_services = new ArrayList<>();
        for (ServiceInfo service_info : service_infos) {
            boolean z = false;
            Boolean service_found = false;
            Iterator<DiscoveredService> it = discovered_services.iterator();
            while (true) {
                if (!it.hasNext()) {
                    break;
                }
                DiscoveredService discovered_service = it.next();
                String qualifiedName = service_info.getQualifiedName();
                if (qualifiedName.equals(discovered_service.name + "." + discovered_service.type + "." + discovered_service.domain + ".")) {
                    InetAddress[] inetAddresses = service_info.getInetAddresses();
                    int length = inetAddresses.length;
                    int i = 0;
                    while (i < length) {
                        InetAddress inet_address = inetAddresses[i];
                        if (inet_address instanceof Inet4Address) {
                            Boolean address_found = Boolean.valueOf(z);
                            Iterator<String> it2 = discovered_service.ipv4_addresses.iterator();
                            while (true) {
                                if (!it2.hasNext()) {
                                    break;
                                }
                                if (inet_address.getHostAddress().equals(it2.next())) {
                                    address_found = true;
                                    break;
                                }
                            }
                            if (!address_found.booleanValue()) {
                                discovered_service.ipv4_addresses.add(inet_address.getHostAddress());
                            }
                        } else {
                            Boolean address_found2 = false;
                            Iterator<String> it3 = discovered_service.ipv6_addresses.iterator();
                            while (true) {
                                if (!it3.hasNext()) {
                                    break;
                                }
                                if (inet_address.getHostAddress().equals(it3.next())) {
                                    address_found2 = true;
                                    break;
                                }
                            }
                            if (!address_found2.booleanValue()) {
                                discovered_service.ipv6_addresses.add(inet_address.getHostAddress());
                            }
                        }
                        i++;
                        z = false;
                    }
                    service_found = true;
                } else {
                    z = false;
                }
            }
            if (!service_found.booleanValue()) {
                discovered_services.add(toDiscoveredService(service_info));
            }
        }
        return discovered_services;
    }

    public void removeAllServices() {
        this.logger.println("Removing all services");
        this.jmmdns.unregisterAllServices();
        this.services.clear();
    }

    public void shutdown() throws IOException {
        removeAllServices();
        this.logger.println("Shutdown");
        this.jmmdns.close();
    }

    public void serviceAdded(ServiceEvent event) {
        ServiceInfo service_info = event.getInfo();
        this.jmmdns.getServiceInfos(service_info.getType(), service_info.getName(), true);
        ZeroconfDiscoveryHandler callback = this.listener_callbacks.get(service_info.getType());
        if (callback != null) {
            callback.serviceAdded(toDiscoveredService(service_info));
            return;
        }
        ZeroconfLogger zeroconfLogger = this.logger;
        zeroconfLogger.println("[+] Service         : " + service_info.getQualifiedName());
    }

    public void serviceRemoved(ServiceEvent event) {
        String name = event.getName();
        ServiceInfo service_info = event.getInfo();
        ZeroconfDiscoveryHandler callback = this.listener_callbacks.get(service_info.getType());
        if (callback != null) {
            callback.serviceRemoved(toDiscoveredService(service_info));
            return;
        }
        ZeroconfLogger zeroconfLogger = this.logger;
        zeroconfLogger.println("[-] Service         : " + name);
    }

    public void serviceResolved(ServiceEvent event) {
        ServiceInfo service_info = event.getInfo();
        ZeroconfDiscoveryHandler callback = this.listener_callbacks.get(service_info.getType());
        if (callback != null) {
            callback.serviceResolved(toDiscoveredService(service_info));
            return;
        }
        ZeroconfLogger zeroconfLogger = this.logger;
        zeroconfLogger.println("[=] Resolved        : " + service_info.getQualifiedName());
        ZeroconfLogger zeroconfLogger2 = this.logger;
        zeroconfLogger2.println("      Port          : " + service_info.getPort());
        for (int i = 0; i < service_info.getInetAddresses().length; i++) {
            ZeroconfLogger zeroconfLogger3 = this.logger;
            zeroconfLogger3.println("      Address       : " + service_info.getInetAddresses()[i].getHostAddress());
        }
    }

    public void serviceTypeAdded(ServiceEvent event) {
    }

    public void subTypeForServiceTypeAdded(ServiceEvent event) {
    }

    public void inetAddressAdded(NetworkTopologyEvent event) {
        try {
            ZeroconfLogger zeroconfLogger = this.logger;
            zeroconfLogger.println("[+] NetworkInterface: " + event.getInetAddress().getHostAddress() + " [" + NetworkInterface.getByInetAddress(event.getInetAddress()).getDisplayName() + "]");
        } catch (IOException e) {
            e.printStackTrace();
        }
        try {
            event.getDNS().addServiceTypeListener(this);
            for (String listener : this.listeners) {
                ZeroconfLogger zeroconfLogger2 = this.logger;
                zeroconfLogger2.println("      Adding service listener '" + listener + "'");
                event.getDNS().addServiceListener(listener, this);
            }
            for (ServiceInfo service : this.services) {
                ZeroconfLogger zeroconfLogger3 = this.logger;
                zeroconfLogger3.println("Publishing Service on " + event.getInetAddress().getHostAddress());
                ZeroconfLogger zeroconfLogger4 = this.logger;
                zeroconfLogger4.println("  Name   : " + service.getName());
                ZeroconfLogger zeroconfLogger5 = this.logger;
                zeroconfLogger5.println("  Type   : " + service.getType());
                ZeroconfLogger zeroconfLogger6 = this.logger;
                zeroconfLogger6.println("  Port   : " + service.getPort());
                event.getDNS().registerService(service.clone());
            }
        } catch (IOException e2) {
            e2.printStackTrace();
        }
    }

    public void inetAddressRemoved(NetworkTopologyEvent event) {
        String event_address_str = event.getInetAddress().getHostAddress();
        ZeroconfLogger zeroconfLogger = this.logger;
        zeroconfLogger.println("[-] NetworkInterface: " + event_address_str);
        event.getDNS().removeServiceTypeListener(this);
        for (String listener : this.listeners) {
            ZeroconfLogger zeroconfLogger2 = this.logger;
            zeroconfLogger2.println("      Removing service listener '" + listener + "'");
            event.getDNS().removeServiceListener(listener, this);
        }
        for (ServiceInfo service : this.services) {
            this.logger.println("Unpublishing Service:");
            ZeroconfLogger zeroconfLogger3 = this.logger;
            zeroconfLogger3.println("  Name   : " + service.getName());
            ZeroconfLogger zeroconfLogger4 = this.logger;
            zeroconfLogger4.println("  Type   : " + service.getType());
            ZeroconfLogger zeroconfLogger5 = this.logger;
            zeroconfLogger5.println("  Port   : " + service.getPort());
            event.getDNS().unregisterService(service);
        }
    }

    public void display(DiscoveredService discovered_service) {
        this.logger.println("Discovered Service:");
        ZeroconfLogger zeroconfLogger = this.logger;
        zeroconfLogger.println("  Name   : " + discovered_service.name);
        ZeroconfLogger zeroconfLogger2 = this.logger;
        zeroconfLogger2.println("  Type   : " + discovered_service.type);
        ZeroconfLogger zeroconfLogger3 = this.logger;
        zeroconfLogger3.println("  Port   : " + discovered_service.port);
        Iterator<String> it = discovered_service.ipv4_addresses.iterator();
        while (it.hasNext()) {
            ZeroconfLogger zeroconfLogger4 = this.logger;
            zeroconfLogger4.println("  Address: " + it.next());
        }
        Iterator<String> it2 = discovered_service.ipv6_addresses.iterator();
        while (it2.hasNext()) {
            ZeroconfLogger zeroconfLogger5 = this.logger;
            zeroconfLogger5.println("  Address: " + it2.next());
        }
    }

    public String toString(DiscoveredService discovered_service) {
        String result = (("Service Info:\n" + "  Name   : " + discovered_service.name + "\n") + "  Type   : " + discovered_service.type + "\n") + "  Port   : " + discovered_service.port + "\n";
        Iterator<String> it = discovered_service.ipv4_addresses.iterator();
        while (it.hasNext()) {
            result = result + "  Address: " + it.next() + "\n";
        }
        Iterator<String> it2 = discovered_service.ipv6_addresses.iterator();
        while (it2.hasNext()) {
            result = result + "  Address: " + it2.next() + "\n";
        }
        return result;
    }

    private DiscoveredService toDiscoveredService(ServiceInfo service_info) {
        DiscoveredService discovered_service = new DiscoveredService();
        discovered_service.name = service_info.getName();
        String[] type_domain_str = service_info.getType().split("\\.");
        StringBuilder sb = new StringBuilder();
        sb.append(type_domain_str[0]);
        sb.append(".");
        sb.append(type_domain_str[1]);
        discovered_service.type = sb.toString();
        discovered_service.domain = service_info.getDomain();
        discovered_service.hostname = service_info.getServer();
        discovered_service.port = service_info.getPort();
        for (InetAddress inet_address : service_info.getInetAddresses()) {
            if (inet_address instanceof Inet4Address) {
                discovered_service.ipv4_addresses.add(inet_address.getHostAddress());
            } else {
                discovered_service.ipv6_addresses.add(inet_address.getHostAddress());
            }
        }
        return discovered_service;
    }
}
