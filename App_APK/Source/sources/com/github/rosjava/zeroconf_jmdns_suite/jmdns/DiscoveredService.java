package com.github.rosjava.zeroconf_jmdns_suite.jmdns;

import java.util.ArrayList;

public class DiscoveredService {
    public String description;
    public String domain;
    public String hostname;
    public ArrayList<String> ipv4_addresses = new ArrayList<>();
    public ArrayList<String> ipv6_addresses = new ArrayList<>();
    public String name;
    public int port;
    public String type;
}
