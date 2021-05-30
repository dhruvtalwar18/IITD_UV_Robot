package com.github.rosjava.zeroconf_jmdns_suite.jmdns;

public class StandardLogger implements ZeroconfLogger {
    public void println(String msg) {
        System.out.println(msg);
    }
}
