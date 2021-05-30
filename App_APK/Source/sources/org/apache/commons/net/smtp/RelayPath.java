package org.apache.commons.net.smtp;

import java.util.Enumeration;
import java.util.Vector;

public final class RelayPath {
    String _emailAddress;
    Vector<String> _path = new Vector<>();

    public RelayPath(String emailAddress) {
        this._emailAddress = emailAddress;
    }

    public void addRelay(String hostname) {
        this._path.addElement(hostname);
    }

    public String toString() {
        StringBuffer buffer = new StringBuffer();
        buffer.append('<');
        Enumeration<String> hosts = this._path.elements();
        if (hosts.hasMoreElements()) {
            buffer.append('@');
            buffer.append(hosts.nextElement());
            while (hosts.hasMoreElements()) {
                buffer.append(",@");
                buffer.append(hosts.nextElement());
            }
            buffer.append(':');
        }
        buffer.append(this._emailAddress);
        buffer.append('>');
        return buffer.toString();
    }
}
