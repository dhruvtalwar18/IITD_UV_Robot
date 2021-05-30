package org.apache.commons.net.tftp;

import java.net.DatagramPacket;
import java.net.InetAddress;

public final class TFTPWriteRequestPacket extends TFTPRequestPacket {
    public TFTPWriteRequestPacket(InetAddress destination, int port, String filename, int mode) {
        super(destination, port, 2, filename, mode);
    }

    TFTPWriteRequestPacket(DatagramPacket datagram) throws TFTPPacketException {
        super(2, datagram);
    }
}
