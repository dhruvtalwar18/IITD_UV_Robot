package javax.jmdns.impl;

interface DNSListener {
    void updateRecord(DNSCache dNSCache, long j, DNSEntry dNSEntry);
}
