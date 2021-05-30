package com.github.rosjava.zeroconf_jmdns_suite.jmdns;

public interface ZeroconfDiscoveryHandler {
    void serviceAdded(DiscoveredService discoveredService);

    void serviceRemoved(DiscoveredService discoveredService);

    void serviceResolved(DiscoveredService discoveredService);
}
