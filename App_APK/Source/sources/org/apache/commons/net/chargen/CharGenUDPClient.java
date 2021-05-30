package org.apache.commons.net.chargen;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.InetAddress;
import org.apache.commons.net.DatagramSocketClient;

public final class CharGenUDPClient extends DatagramSocketClient {
    public static final int CHARGEN_PORT = 19;
    public static final int DEFAULT_PORT = 19;
    public static final int NETSTAT_PORT = 15;
    public static final int QUOTE_OF_DAY_PORT = 17;
    public static final int SYSTAT_PORT = 11;
    private byte[] __receiveData = new byte[512];
    private DatagramPacket __receivePacket = new DatagramPacket(this.__receiveData, 512);
    private DatagramPacket __sendPacket = new DatagramPacket(new byte[0], 0);

    public void send(InetAddress host, int port) throws IOException {
        this.__sendPacket.setAddress(host);
        this.__sendPacket.setPort(port);
        this._socket_.send(this.__sendPacket);
    }

    public void send(InetAddress host) throws IOException {
        send(host, 19);
    }

    public byte[] receive() throws IOException {
        this._socket_.receive(this.__receivePacket);
        int length = this.__receivePacket.getLength();
        int length2 = length;
        byte[] result = new byte[length];
        System.arraycopy(this.__receiveData, 0, result, 0, length2);
        return result;
    }
}
