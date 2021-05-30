package org.apache.commons.net.tftp;

import java.net.DatagramPacket;
import java.net.InetAddress;

public final class TFTPReadRequestPacket extends TFTPRequestPacket {
    public TFTPReadRequestPacket(InetAddress destination, int port, String filename, int mode) {
        super(destination, port, 1, filename, mode);
    }

    TFTPReadRequestPacket(DatagramPacket datagram) throws TFTPPacketException {
        super(1, datagram);
    }
}
