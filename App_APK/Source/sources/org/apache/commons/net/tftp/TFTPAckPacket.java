package org.apache.commons.net.tftp;

import java.net.DatagramPacket;
import java.net.InetAddress;
import sensor_msgs.NavSatStatus;

public final class TFTPAckPacket extends TFTPPacket {
    int _blockNumber;

    public TFTPAckPacket(InetAddress destination, int port, int blockNumber) {
        super(4, destination, port);
        this._blockNumber = blockNumber;
    }

    TFTPAckPacket(DatagramPacket datagram) throws TFTPPacketException {
        super(4, datagram.getAddress(), datagram.getPort());
        byte[] data = datagram.getData();
        if (getType() == data[1]) {
            this._blockNumber = ((data[2] & NavSatStatus.STATUS_NO_FIX) << 8) | (data[3] & NavSatStatus.STATUS_NO_FIX);
            return;
        }
        throw new TFTPPacketException("TFTP operator code does not match type.");
    }

    /* access modifiers changed from: package-private */
    public DatagramPacket _newDatagram(DatagramPacket datagram, byte[] data) {
        data[0] = 0;
        data[1] = (byte) this._type;
        data[2] = (byte) ((this._blockNumber & 65535) >> 8);
        data[3] = (byte) (this._blockNumber & 255);
        datagram.setAddress(this._address);
        datagram.setPort(this._port);
        datagram.setData(data);
        datagram.setLength(4);
        return datagram;
    }

    public DatagramPacket newDatagram() {
        byte[] data = {0, (byte) this._type, (byte) ((this._blockNumber & 65535) >> 8), (byte) (this._blockNumber & 255)};
        return new DatagramPacket(data, data.length, this._address, this._port);
    }

    public int getBlockNumber() {
        return this._blockNumber;
    }

    public void setBlockNumber(int blockNumber) {
        this._blockNumber = blockNumber;
    }
}
