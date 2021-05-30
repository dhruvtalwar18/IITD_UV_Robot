package org.apache.commons.net.time;

import com.google.common.base.Ascii;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.InetAddress;
import java.util.Date;
import org.apache.commons.net.DatagramSocketClient;
import sensor_msgs.NavSatStatus;

public final class TimeUDPClient extends DatagramSocketClient {
    public static final int DEFAULT_PORT = 37;
    public static final long SECONDS_1900_TO_1970 = 2208988800L;
    private byte[] __dummyData = new byte[1];
    private byte[] __timeData = new byte[4];

    public long getTime(InetAddress host, int port) throws IOException {
        DatagramPacket sendPacket = new DatagramPacket(this.__dummyData, this.__dummyData.length, host, port);
        DatagramPacket receivePacket = new DatagramPacket(this.__timeData, this.__timeData.length);
        this._socket_.send(sendPacket);
        this._socket_.receive(receivePacket);
        return 0 | (((long) ((this.__timeData[0] & NavSatStatus.STATUS_NO_FIX) << Ascii.CAN)) & 4294967295L) | (((long) ((this.__timeData[1] & NavSatStatus.STATUS_NO_FIX) << 16)) & 4294967295L) | (((long) ((this.__timeData[2] & NavSatStatus.STATUS_NO_FIX) << 8)) & 4294967295L) | (((long) (this.__timeData[3] & NavSatStatus.STATUS_NO_FIX)) & 4294967295L);
    }

    public long getTime(InetAddress host) throws IOException {
        return getTime(host, 37);
    }

    public Date getDate(InetAddress host, int port) throws IOException {
        return new Date((getTime(host, port) - 2208988800L) * 1000);
    }

    public Date getDate(InetAddress host) throws IOException {
        return new Date((getTime(host, 37) - 2208988800L) * 1000);
    }
}
