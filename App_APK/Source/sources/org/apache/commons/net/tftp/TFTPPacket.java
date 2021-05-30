package org.apache.commons.net.tftp;

import java.net.DatagramPacket;
import java.net.InetAddress;

public abstract class TFTPPacket {
    public static final int ACKNOWLEDGEMENT = 4;
    public static final int DATA = 3;
    public static final int ERROR = 5;
    static final int MIN_PACKET_SIZE = 4;
    public static final int READ_REQUEST = 1;
    public static final int SEGMENT_SIZE = 512;
    public static final int WRITE_REQUEST = 2;
    InetAddress _address;
    int _port;
    int _type;

    /* access modifiers changed from: package-private */
    public abstract DatagramPacket _newDatagram(DatagramPacket datagramPacket, byte[] bArr);

    public abstract DatagramPacket newDatagram();

    public static final TFTPPacket newTFTPPacket(DatagramPacket datagram) throws TFTPPacketException {
        if (datagram.getLength() >= 4) {
            switch (datagram.getData()[1]) {
                case 1:
                    return new TFTPReadRequestPacket(datagram);
                case 2:
                    return new TFTPWriteRequestPacket(datagram);
                case 3:
                    return new TFTPDataPacket(datagram);
                case 4:
                    return new TFTPAckPacket(datagram);
                case 5:
                    return new TFTPErrorPacket(datagram);
                default:
                    throw new TFTPPacketException("Bad packet.  Invalid TFTP operator code.");
            }
        } else {
            throw new TFTPPacketException("Bad packet. Datagram data length is too short.");
        }
    }

    TFTPPacket(int type, InetAddress address, int port) {
        this._type = type;
        this._address = address;
        this._port = port;
    }

    public final int getType() {
        return this._type;
    }

    public final InetAddress getAddress() {
        return this._address;
    }

    public final int getPort() {
        return this._port;
    }

    public final void setPort(int port) {
        this._port = port;
    }

    public final void setAddress(InetAddress address) {
        this._address = address;
    }
}
