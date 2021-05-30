package org.apache.commons.net.tftp;

import java.io.IOException;
import java.io.InputStream;
import java.io.InterruptedIOException;
import java.io.OutputStream;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;
import org.apache.commons.net.io.FromNetASCIIOutputStream;

public class TFTPClient extends TFTP {
    public static final int DEFAULT_MAX_TIMEOUTS = 5;
    private int __maxTimeouts = 5;

    public void setMaxTimeouts(int numTimeouts) {
        if (numTimeouts < 1) {
            this.__maxTimeouts = 1;
        } else {
            this.__maxTimeouts = numTimeouts;
        }
    }

    public int getMaxTimeouts() {
        return this.__maxTimeouts;
    }

    public int receiveFile(String filename, int mode, OutputStream output, InetAddress host, int port) throws IOException {
        OutputStream output2;
        int timeouts;
        int hostPort;
        TFTPAckPacket ack;
        int dataLength;
        TFTPDataPacket data;
        int timeouts2;
        int i = mode;
        InetAddress host2 = host;
        int i2 = port;
        TFTPPacket received = null;
        TFTPAckPacket ack2 = new TFTPAckPacket(host2, i2, 0);
        beginBufferedOps();
        int lastBlock = 0;
        int block = 1;
        if (i == 0) {
            output2 = new FromNetASCIIOutputStream(output);
        } else {
            output2 = output;
        }
        TFTPPacket sent = new TFTPReadRequestPacket(host2, i2, filename, i);
        int dataLength2 = 0;
        int dataLength3 = 0;
        int bytesRead = 0;
        TFTPDataPacket data2 = null;
        while (true) {
            bufferedSend(sent);
            int dataLength4 = dataLength2;
            int dataLength5 = dataLength3;
            int lastBlock2 = lastBlock;
            TFTPDataPacket data3 = data2;
            TFTPPacket received2 = received;
            InetAddress host3 = host2;
            while (true) {
                int timeouts3 = 0;
                while (true) {
                    timeouts = timeouts3;
                    if (timeouts >= this.__maxTimeouts) {
                        break;
                    }
                    try {
                        received2 = bufferedReceive();
                        break;
                    } catch (SocketException e) {
                        SocketException socketException = e;
                        timeouts2 = timeouts + 1;
                        if (timeouts2 >= this.__maxTimeouts) {
                            endBufferedOps();
                            throw new IOException("Connection timed out.");
                        }
                    } catch (InterruptedIOException e2) {
                        InterruptedIOException interruptedIOException = e2;
                        timeouts2 = timeouts + 1;
                        if (timeouts2 >= this.__maxTimeouts) {
                            endBufferedOps();
                            throw new IOException("Connection timed out.");
                        }
                    } catch (TFTPPacketException e3) {
                        endBufferedOps();
                        throw new IOException("Bad packet: " + e3.getMessage());
                    }
                    timeouts3 = timeouts2;
                    int i3 = mode;
                }
                if (lastBlock2 == 0) {
                    int hostPort2 = received2.getPort();
                    ack2.setPort(hostPort2);
                    if (!host3.equals(received2.getAddress())) {
                        InetAddress host4 = received2.getAddress();
                        ack2.setAddress(host4);
                        sent.setAddress(host4);
                        host3 = host4;
                    }
                    hostPort = hostPort2;
                } else {
                    hostPort = dataLength5;
                }
                if (!host3.equals(received2.getAddress()) || received2.getPort() != hostPort) {
                    ack = ack2;
                    int i4 = timeouts;
                    TFTPErrorPacket error = new TFTPErrorPacket(received2.getAddress(), received2.getPort(), 5, "Unexpected host or port.");
                    bufferedSend(error);
                    TFTPErrorPacket tFTPErrorPacket = error;
                    data = data3;
                    lastBlock = lastBlock2;
                    dataLength = dataLength4;
                } else {
                    int type = received2.getType();
                    int i5 = timeouts;
                    if (type == 3) {
                        data3 = (TFTPDataPacket) received2;
                        dataLength = data3.getDataLength();
                        lastBlock2 = data3.getBlockNumber();
                        if (lastBlock2 == block) {
                            try {
                                output2.write(data3.getData(), data3.getDataOffset(), dataLength);
                                block++;
                                if (block > 65535) {
                                    block = 0;
                                }
                                ack2.setBlockNumber(lastBlock2);
                                bytesRead += dataLength;
                                sent = ack2;
                                ack = ack2;
                                data = data3;
                                lastBlock = lastBlock2;
                            } catch (IOException e4) {
                                int i6 = dataLength;
                                TFTPAckPacket tFTPAckPacket = ack2;
                                bufferedSend(new TFTPErrorPacket(host3, hostPort, 3, "File write failed."));
                                endBufferedOps();
                                throw e4;
                            }
                        } else {
                            int dataLength6 = dataLength;
                            ack = ack2;
                            int i7 = 65535;
                            discardPackets();
                            if (block != 0) {
                                i7 = block - 1;
                            }
                            if (lastBlock2 == i7) {
                                data = data3;
                                lastBlock = lastBlock2;
                                dataLength = dataLength6;
                                break;
                            }
                            dataLength5 = hostPort;
                            dataLength4 = dataLength6;
                            ack2 = ack;
                            int hostPort3 = mode;
                            int i8 = port;
                        }
                    } else if (type != 5) {
                        endBufferedOps();
                        throw new IOException("Received unexpected packet type.");
                    } else {
                        TFTPErrorPacket error2 = (TFTPErrorPacket) received2;
                        endBufferedOps();
                        throw new IOException("Error code " + error2.getError() + " received: " + error2.getMessage());
                    }
                }
            }
            if (dataLength != 512) {
                bufferedSend(sent);
                endBufferedOps();
                return bytesRead;
            }
            dataLength3 = hostPort;
            dataLength2 = dataLength;
            ack2 = ack;
            int hostPort4 = mode;
            int dataLength7 = port;
            TFTPPacket tFTPPacket = received2;
            data2 = data;
            host2 = host3;
            received = tFTPPacket;
        }
    }

    public int receiveFile(String filename, int mode, OutputStream output, String hostname, int port) throws UnknownHostException, IOException {
        return receiveFile(filename, mode, output, InetAddress.getByName(hostname), port);
    }

    public int receiveFile(String filename, int mode, OutputStream output, InetAddress host) throws IOException {
        return receiveFile(filename, mode, output, host, 69);
    }

    public int receiveFile(String filename, int mode, OutputStream output, String hostname) throws UnknownHostException, IOException {
        return receiveFile(filename, mode, output, InetAddress.getByName(hostname), 69);
    }

    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r7v7, resolved type: org.apache.commons.net.tftp.TFTPErrorPacket} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r0v13, resolved type: org.apache.commons.net.tftp.TFTPErrorPacket} */
    /* JADX WARNING: type inference failed for: r0v26, types: [org.apache.commons.net.tftp.TFTPPacket] */
    /* JADX WARNING: Code restructure failed: missing block: B:64:0x018e, code lost:
        r26 = r11;
        r11 = r0;
        r0 = r26;
     */
    /* JADX WARNING: Multi-variable type inference failed */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public void sendFile(java.lang.String r28, int r29, java.io.InputStream r30, java.net.InetAddress r31, int r32) throws java.io.IOException {
        /*
            r27 = this;
            r1 = r27
            r2 = r29
            r0 = 0
            org.apache.commons.net.tftp.TFTPDataPacket r10 = new org.apache.commons.net.tftp.TFTPDataPacket
            byte[] r7 = r1._sendBuffer
            r6 = 0
            r8 = 4
            r9 = 0
            r3 = r10
            r4 = r31
            r5 = r32
            r3.<init>(r4, r5, r6, r7, r8, r9)
            r4 = 1
            r27.beginBufferedOps()
            r5 = 0
            r6 = r5
            r7 = r5
            r8 = r5
            r9 = r5
            r10 = r5
            r11 = 0
            r12 = 0
            if (r2 != 0) goto L_0x002a
            org.apache.commons.net.io.ToNetASCIIInputStream r13 = new org.apache.commons.net.io.ToNetASCIIInputStream
            r14 = r30
            r13.<init>(r14)
            goto L_0x002d
        L_0x002a:
            r14 = r30
            r13 = r14
        L_0x002d:
            org.apache.commons.net.tftp.TFTPWriteRequestPacket r14 = new org.apache.commons.net.tftp.TFTPWriteRequestPacket
            r15 = r28
            r5 = r31
            r16 = r4
            r4 = r32
            r14.<init>(r5, r4, r15, r2)
            r17 = 0
            r2 = r11
            r11 = r9
            r9 = r7
            r7 = r17
        L_0x0041:
            r1.bufferedSend(r14)
            r18 = r11
            r11 = r7
            r7 = r0
        L_0x0048:
            r0 = 0
        L_0x0049:
            r19 = r0
            int r0 = r1.__maxTimeouts
            r4 = r19
            if (r4 >= r0) goto L_0x00ba
            org.apache.commons.net.tftp.TFTPPacket r0 = r27.bufferedReceive()     // Catch:{ SocketException -> 0x0099, InterruptedIOException -> 0x0080, TFTPPacketException -> 0x005b }
            r7 = r0
            r20 = r6
            goto L_0x00be
        L_0x005b:
            r0 = move-exception
            r11 = r0
            r0 = r11
            r27.endBufferedOps()
            java.io.IOException r11 = new java.io.IOException
            r20 = r6
            java.lang.StringBuilder r6 = new java.lang.StringBuilder
            r6.<init>()
            r21 = r7
            java.lang.String r7 = "Bad packet: "
            r6.append(r7)
            java.lang.String r7 = r0.getMessage()
            r6.append(r7)
            java.lang.String r6 = r6.toString()
            r11.<init>(r6)
            throw r11
        L_0x0080:
            r0 = move-exception
            r20 = r6
            r21 = r7
            r6 = r0
            r0 = r6
            int r4 = r4 + 1
            int r6 = r1.__maxTimeouts
            if (r4 >= r6) goto L_0x008e
            goto L_0x00a7
        L_0x008e:
            r27.endBufferedOps()
            java.io.IOException r6 = new java.io.IOException
            java.lang.String r7 = "Connection timed out."
            r6.<init>(r7)
            throw r6
        L_0x0099:
            r0 = move-exception
            r20 = r6
            r21 = r7
            r6 = r0
            r0 = r6
            int r4 = r4 + 1
            int r6 = r1.__maxTimeouts
            if (r4 >= r6) goto L_0x00af
        L_0x00a7:
            r0 = r4
            r6 = r20
            r7 = r21
            r4 = r32
            goto L_0x0049
        L_0x00af:
            r27.endBufferedOps()
            java.io.IOException r6 = new java.io.IOException
            java.lang.String r7 = "Connection timed out."
            r6.<init>(r7)
            throw r6
        L_0x00ba:
            r20 = r6
            r21 = r7
        L_0x00be:
            if (r16 == 0) goto L_0x00df
            r0 = 0
            int r6 = r7.getPort()
            r3.setPort(r6)
            java.net.InetAddress r8 = r7.getAddress()
            boolean r8 = r5.equals(r8)
            if (r8 != 0) goto L_0x00dc
            java.net.InetAddress r5 = r7.getAddress()
            r3.setAddress(r5)
            r14.setAddress(r5)
        L_0x00dc:
            r16 = r0
            r8 = r6
        L_0x00df:
            java.net.InetAddress r0 = r7.getAddress()
            boolean r0 = r5.equals(r0)
            if (r0 == 0) goto L_0x019e
            int r0 = r7.getPort()
            if (r0 != r8) goto L_0x019e
            int r0 = r7.getType()
            switch(r0) {
                case 4: goto L_0x0134;
                case 5: goto L_0x0105;
                default: goto L_0x00f6;
            }
        L_0x00f6:
            r22 = r4
            r23 = r5
            r27.endBufferedOps()
            java.io.IOException r0 = new java.io.IOException
            java.lang.String r4 = "Received unexpected packet type."
            r0.<init>(r4)
            throw r0
        L_0x0105:
            r0 = r7
            org.apache.commons.net.tftp.TFTPErrorPacket r0 = (org.apache.commons.net.tftp.TFTPErrorPacket) r0
            r27.endBufferedOps()
            java.io.IOException r6 = new java.io.IOException
            java.lang.StringBuilder r11 = new java.lang.StringBuilder
            r11.<init>()
            r22 = r4
            java.lang.String r4 = "Error code "
            r11.append(r4)
            int r4 = r0.getError()
            r11.append(r4)
            java.lang.String r4 = " received: "
            r11.append(r4)
            java.lang.String r4 = r0.getMessage()
            r11.append(r4)
            java.lang.String r4 = r11.toString()
            r6.<init>(r4)
            throw r6
        L_0x0134:
            r22 = r4
            r11 = r7
            org.apache.commons.net.tftp.TFTPAckPacket r11 = (org.apache.commons.net.tftp.TFTPAckPacket) r11
            int r0 = r11.getBlockNumber()
            r4 = 65535(0xffff, float:9.1834E-41)
            if (r0 != r2) goto L_0x017f
            int r2 = r2 + 1
            if (r2 <= r4) goto L_0x0147
            r2 = 0
        L_0x0147:
            if (r12 == 0) goto L_0x014f
            r11 = r0
            r23 = r5
            goto L_0x01c7
        L_0x014f:
            r4 = 512(0x200, float:7.175E-43)
            r6 = 4
            r10 = 0
        L_0x0153:
            if (r4 <= 0) goto L_0x0166
            r23 = r5
            byte[] r5 = r1._sendBuffer
            int r5 = r13.read(r5, r6, r4)
            r9 = r5
            if (r5 <= 0) goto L_0x0168
            int r6 = r6 + r9
            int r4 = r4 - r9
            int r10 = r10 + r9
            r5 = r23
            goto L_0x0153
        L_0x0166:
            r23 = r5
        L_0x0168:
            r5 = 512(0x200, float:7.175E-43)
            if (r10 >= r5) goto L_0x016d
            r12 = 1
        L_0x016d:
            r3.setBlockNumber(r2)
            byte[] r5 = r1._sendBuffer
            r24 = r2
            r2 = 4
            r3.setData(r5, r2, r10)
            r2 = r3
            r14 = r2
            r6 = r10
            r2 = r24
            r10 = r4
            goto L_0x018e
        L_0x017f:
            r23 = r5
            r27.discardPackets()
            if (r2 != 0) goto L_0x0187
            goto L_0x0189
        L_0x0187:
            int r4 = r2 + -1
        L_0x0189:
            if (r0 != r4) goto L_0x0194
            r6 = r20
        L_0x018e:
            r26 = r11
            r11 = r0
            r0 = r26
            goto L_0x01c1
        L_0x0194:
            r18 = r0
            r6 = r20
            r5 = r23
            r4 = r32
            goto L_0x0048
        L_0x019e:
            r22 = r4
            r23 = r5
            org.apache.commons.net.tftp.TFTPErrorPacket r0 = new org.apache.commons.net.tftp.TFTPErrorPacket
            java.net.InetAddress r4 = r7.getAddress()
            int r5 = r7.getPort()
            r6 = 5
            r25 = r2
            java.lang.String r2 = "Unexpected host or port."
            r0.<init>(r4, r5, r6, r2)
            r1.bufferedSend(r0)
            r17 = r0
            r0 = r11
            r11 = r18
            r6 = r20
            r2 = r25
        L_0x01c1:
            if (r6 > 0) goto L_0x01cb
            if (r12 != 0) goto L_0x01cb
            r20 = r6
        L_0x01c7:
            r27.endBufferedOps()
            return
        L_0x01cb:
            r5 = r23
            r4 = r32
            r26 = r7
            r7 = r0
            r0 = r26
            goto L_0x0041
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.net.tftp.TFTPClient.sendFile(java.lang.String, int, java.io.InputStream, java.net.InetAddress, int):void");
    }

    public void sendFile(String filename, int mode, InputStream input, String hostname, int port) throws UnknownHostException, IOException {
        sendFile(filename, mode, input, InetAddress.getByName(hostname), port);
    }

    public void sendFile(String filename, int mode, InputStream input, InetAddress host) throws IOException {
        sendFile(filename, mode, input, host, 69);
    }

    public void sendFile(String filename, int mode, InputStream input, String hostname) throws UnknownHostException, IOException {
        sendFile(filename, mode, input, InetAddress.getByName(hostname), 69);
    }
}
