package org.apache.commons.net.discard;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.InetAddress;
import org.apache.commons.net.DatagramSocketClient;

public class DiscardUDPClient extends DatagramSocketClient {
    public static final int DEFAULT_PORT = 9;
    DatagramPacket _sendPacket = new DatagramPacket(new byte[0], 0);

    public void send(byte[] data, int length, InetAddress host, int port) throws IOException {
        this._sendPacket.setData(data);
        this._sendPacket.setLength(length);
        this._sendPacket.setAddress(host);
        this._sendPacket.setPort(port);
        this._socket_.send(this._sendPacket);
    }

    public void send(byte[] data, int length, InetAddress host) throws IOException {
        send(data, length, host, 9);
    }

    public void send(byte[] data, InetAddress host) throws IOException {
        send(data, data.length, host, 9);
    }
}
