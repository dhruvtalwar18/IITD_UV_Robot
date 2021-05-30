package javax.jmdns.impl;

import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.InetAddress;
import java.util.HashMap;
import java.util.Map;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.jmdns.impl.constants.DNSConstants;
import javax.jmdns.impl.constants.DNSLabel;
import javax.jmdns.impl.constants.DNSRecordClass;
import javax.jmdns.impl.constants.DNSRecordType;
import sensor_msgs.NavSatStatus;

public final class DNSIncoming extends DNSMessage {
    public static boolean USE_DOMAIN_NAME_FORMAT_FOR_SRV_TARGET = true;
    private static final char[] _nibbleToHex = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
    private static Logger logger = Logger.getLogger(DNSIncoming.class.getName());
    private final MessageInputStream _messageInputStream;
    private final DatagramPacket _packet;
    private final long _receivedTime;
    private int _senderUDPPayload;

    public static class MessageInputStream extends ByteArrayInputStream {
        private static Logger logger1 = Logger.getLogger(MessageInputStream.class.getName());
        final Map<Integer, String> _names;

        public MessageInputStream(byte[] buffer, int length) {
            this(buffer, 0, length);
        }

        public MessageInputStream(byte[] buffer, int offset, int length) {
            super(buffer, offset, length);
            this._names = new HashMap();
        }

        public int readByte() {
            return read();
        }

        public int readUnsignedShort() {
            return (read() << 8) | read();
        }

        public int readInt() {
            return (readUnsignedShort() << 16) | readUnsignedShort();
        }

        public byte[] readBytes(int len) {
            byte[] bytes = new byte[len];
            read(bytes, 0, len);
            return bytes;
        }

        public String readUTF(int len) {
            StringBuilder buffer = new StringBuilder(len);
            int index = 0;
            while (index < len) {
                int ch = read();
                int i = ch >> 4;
                switch (i) {
                    case 0:
                    case 1:
                    case 2:
                    case 3:
                    case 4:
                    case 5:
                    case 6:
                    case 7:
                        break;
                    default:
                        switch (i) {
                            case 12:
                            case 13:
                                ch = ((ch & 31) << 6) | (read() & 63);
                                index++;
                                break;
                            case 14:
                                ch = ((ch & 15) << 12) | ((read() & 63) << 6) | (read() & 63);
                                index = index + 1 + 1;
                                break;
                            default:
                                ch = ((ch & 63) << 4) | (read() & 15);
                                index++;
                                break;
                        }
                }
                buffer.append((char) ch);
                index++;
            }
            return buffer.toString();
        }

        /* access modifiers changed from: protected */
        public synchronized int peek() {
            return this.pos < this.count ? this.buf[this.pos] & NavSatStatus.STATUS_NO_FIX : -1;
        }

        public String readName() {
            Map<Integer, StringBuilder> names = new HashMap<>();
            StringBuilder buffer = new StringBuilder();
            boolean finished = false;
            while (true) {
                if (!finished) {
                    int len = read();
                    if (len != 0) {
                        switch (DNSLabel.labelForByte(len)) {
                            case Standard:
                                int offset = this.pos - 1;
                                String label = readUTF(len) + ".";
                                buffer.append(label);
                                for (StringBuilder previousLabel : names.values()) {
                                    previousLabel.append(label);
                                }
                                names.put(Integer.valueOf(offset), new StringBuilder(label));
                                break;
                            case Compressed:
                                int index = (DNSLabel.labelValue(len) << 8) | read();
                                String compressedLabel = this._names.get(Integer.valueOf(index));
                                if (compressedLabel == null) {
                                    logger1.severe("bad domain name: possible circular name detected. Bad offset: 0x" + Integer.toHexString(index) + " at 0x" + Integer.toHexString(this.pos - 2));
                                    compressedLabel = "";
                                }
                                buffer.append(compressedLabel);
                                for (StringBuilder previousLabel2 : names.values()) {
                                    previousLabel2.append(compressedLabel);
                                }
                                finished = true;
                                break;
                            case Extended:
                                logger1.severe("Extended label are not currently supported.");
                                break;
                            default:
                                logger1.severe("unsupported dns label type: '" + Integer.toHexString(len & 192) + "'");
                                break;
                        }
                    }
                }
            }
            for (Integer index2 : names.keySet()) {
                this._names.put(index2, names.get(index2).toString());
            }
            return buffer.toString();
        }

        public String readNonNameString() {
            return readUTF(read());
        }
    }

    /* JADX INFO: super call moved to the top of the method (can break code semantics) */
    public DNSIncoming(DatagramPacket packet) throws IOException {
        super(0, 0, packet.getPort() == DNSConstants.MDNS_PORT);
        this._packet = packet;
        InetAddress source = packet.getAddress();
        this._messageInputStream = new MessageInputStream(packet.getData(), packet.getLength());
        this._receivedTime = System.currentTimeMillis();
        this._senderUDPPayload = DNSConstants.MAX_MSG_TYPICAL;
        try {
            setId(this._messageInputStream.readUnsignedShort());
            setFlags(this._messageInputStream.readUnsignedShort());
            int numQuestions = this._messageInputStream.readUnsignedShort();
            int numAnswers = this._messageInputStream.readUnsignedShort();
            int numAuthorities = this._messageInputStream.readUnsignedShort();
            int numAdditionals = this._messageInputStream.readUnsignedShort();
            if (numQuestions > 0) {
                for (int i = 0; i < numQuestions; i++) {
                    this._questions.add(readQuestion());
                }
            }
            if (numAnswers > 0) {
                for (int i2 = 0; i2 < numAnswers; i2++) {
                    DNSRecord rec = readAnswer(source);
                    if (rec != null) {
                        this._answers.add(rec);
                    }
                }
            }
            if (numAuthorities > 0) {
                for (int i3 = 0; i3 < numAuthorities; i3++) {
                    DNSRecord rec2 = readAnswer(source);
                    if (rec2 != null) {
                        this._authoritativeAnswers.add(rec2);
                    }
                }
            }
            if (numAdditionals > 0) {
                for (int i4 = 0; i4 < numAdditionals; i4++) {
                    DNSRecord rec3 = readAnswer(source);
                    if (rec3 != null) {
                        this._additionals.add(rec3);
                    }
                }
            }
        } catch (Exception e) {
            Logger logger2 = logger;
            Level level = Level.WARNING;
            logger2.log(level, "DNSIncoming() dump " + print(true) + "\n exception ", e);
            IOException ioe = new IOException("DNSIncoming corrupted message");
            ioe.initCause(e);
            throw ioe;
        }
    }

    private DNSIncoming(int flags, int id, boolean multicast, DatagramPacket packet, long receivedTime) {
        super(flags, id, multicast);
        this._packet = packet;
        this._messageInputStream = new MessageInputStream(packet.getData(), packet.getLength());
        this._receivedTime = receivedTime;
    }

    public DNSIncoming clone() {
        DNSIncoming in = new DNSIncoming(getFlags(), getId(), isMulticast(), this._packet, this._receivedTime);
        in._senderUDPPayload = this._senderUDPPayload;
        in._questions.addAll(this._questions);
        in._answers.addAll(this._answers);
        in._authoritativeAnswers.addAll(this._authoritativeAnswers);
        in._additionals.addAll(this._additionals);
        return in;
    }

    private DNSQuestion readQuestion() {
        String domain = this._messageInputStream.readName();
        DNSRecordType type = DNSRecordType.typeForIndex(this._messageInputStream.readUnsignedShort());
        if (type == DNSRecordType.TYPE_IGNORE) {
            Logger logger2 = logger;
            Level level = Level.SEVERE;
            logger2.log(level, "Could not find record type: " + print(true));
        }
        int recordClassIndex = this._messageInputStream.readUnsignedShort();
        DNSRecordClass recordClass = DNSRecordClass.classForIndex(recordClassIndex);
        return DNSQuestion.newQuestion(domain, type, recordClass, recordClass.isUnique(recordClassIndex));
    }

    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r17v0, resolved type: javax.jmdns.impl.DNSRecord$Pointer} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r17v1, resolved type: javax.jmdns.impl.DNSRecord$Pointer} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r17v4, resolved type: javax.jmdns.impl.DNSRecord$Pointer} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r2v18, resolved type: javax.jmdns.impl.DNSRecord$Service} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r17v6, resolved type: javax.jmdns.impl.DNSRecord$Pointer} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r17v7, resolved type: javax.jmdns.impl.DNSRecord$Pointer} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r2v25, resolved type: javax.jmdns.impl.DNSRecord$HostInformation} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r17v8, resolved type: javax.jmdns.impl.DNSRecord$Pointer} */
    /* JADX WARNING: type inference failed for: r0v11, types: [javax.jmdns.impl.DNSRecord] */
    /* JADX WARNING: type inference failed for: r0v12 */
    /* JADX WARNING: type inference failed for: r2v90, types: [javax.jmdns.impl.DNSRecord$IPv4Address] */
    /* JADX WARNING: type inference failed for: r2v91, types: [javax.jmdns.impl.DNSRecord$IPv6Address] */
    /* JADX WARNING: type inference failed for: r2v92, types: [javax.jmdns.impl.DNSRecord$Text] */
    /* JADX WARNING: Can't fix incorrect switch cases order */
    /* JADX WARNING: Code restructure failed: missing block: B:118:0x03ec, code lost:
        r12 = r9;
        r20 = r10;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:133:0x04d9, code lost:
        r0 = r17;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:134:0x04db, code lost:
        if (r0 == 0) goto L_0x04e3;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:135:0x04dd, code lost:
        r0.setRecordSource(r44);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:136:0x04e3, code lost:
        r2 = r44;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:137:0x04e5, code lost:
        return r0;
     */
    /* JADX WARNING: Multi-variable type inference failed */
    /* JADX WARNING: Removed duplicated region for block: B:103:0x0347  */
    /* JADX WARNING: Removed duplicated region for block: B:94:0x02dc  */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    private javax.jmdns.impl.DNSRecord readAnswer(java.net.InetAddress r44) {
        /*
            r43 = this;
            r1 = r43
            javax.jmdns.impl.DNSIncoming$MessageInputStream r0 = r1._messageInputStream
            java.lang.String r11 = r0.readName()
            javax.jmdns.impl.DNSIncoming$MessageInputStream r0 = r1._messageInputStream
            int r12 = r0.readUnsignedShort()
            javax.jmdns.impl.constants.DNSRecordType r13 = javax.jmdns.impl.constants.DNSRecordType.typeForIndex(r12)
            javax.jmdns.impl.constants.DNSRecordType r0 = javax.jmdns.impl.constants.DNSRecordType.TYPE_IGNORE
            r2 = 1
            if (r13 != r0) goto L_0x003b
            java.util.logging.Logger r0 = logger
            java.util.logging.Level r3 = java.util.logging.Level.SEVERE
            java.lang.StringBuilder r4 = new java.lang.StringBuilder
            r4.<init>()
            java.lang.String r5 = "Could not find record type. domain: "
            r4.append(r5)
            r4.append(r11)
            java.lang.String r5 = "\n"
            r4.append(r5)
            java.lang.String r5 = r1.print(r2)
            r4.append(r5)
            java.lang.String r4 = r4.toString()
            r0.log(r3, r4)
        L_0x003b:
            javax.jmdns.impl.DNSIncoming$MessageInputStream r0 = r1._messageInputStream
            int r14 = r0.readUnsignedShort()
            javax.jmdns.impl.constants.DNSRecordType r0 = javax.jmdns.impl.constants.DNSRecordType.TYPE_OPT
            if (r13 != r0) goto L_0x0048
            javax.jmdns.impl.constants.DNSRecordClass r0 = javax.jmdns.impl.constants.DNSRecordClass.CLASS_UNKNOWN
            goto L_0x004c
        L_0x0048:
            javax.jmdns.impl.constants.DNSRecordClass r0 = javax.jmdns.impl.constants.DNSRecordClass.classForIndex(r14)
        L_0x004c:
            r15 = r0
            javax.jmdns.impl.constants.DNSRecordClass r0 = javax.jmdns.impl.constants.DNSRecordClass.CLASS_UNKNOWN
            if (r15 != r0) goto L_0x0081
            javax.jmdns.impl.constants.DNSRecordType r0 = javax.jmdns.impl.constants.DNSRecordType.TYPE_OPT
            if (r13 == r0) goto L_0x0081
            java.util.logging.Logger r0 = logger
            java.util.logging.Level r3 = java.util.logging.Level.SEVERE
            java.lang.StringBuilder r4 = new java.lang.StringBuilder
            r4.<init>()
            java.lang.String r5 = "Could not find record class. domain: "
            r4.append(r5)
            r4.append(r11)
            java.lang.String r5 = " type: "
            r4.append(r5)
            r4.append(r13)
            java.lang.String r5 = "\n"
            r4.append(r5)
            java.lang.String r5 = r1.print(r2)
            r4.append(r5)
            java.lang.String r4 = r4.toString()
            r0.log(r3, r4)
        L_0x0081:
            boolean r16 = r15.isUnique(r14)
            javax.jmdns.impl.DNSIncoming$MessageInputStream r0 = r1._messageInputStream
            int r10 = r0.readInt()
            javax.jmdns.impl.DNSIncoming$MessageInputStream r0 = r1._messageInputStream
            int r9 = r0.readUnsignedShort()
            r17 = 0
            int[] r0 = javax.jmdns.impl.DNSIncoming.AnonymousClass1.$SwitchMap$javax$jmdns$impl$constants$DNSRecordType
            int r3 = r13.ordinal()
            r0 = r0[r3]
            r3 = 0
            switch(r0) {
                case 1: goto L_0x04b7;
                case 2: goto L_0x049b;
                case 3: goto L_0x0454;
                case 4: goto L_0x0454;
                case 5: goto L_0x0437;
                case 6: goto L_0x03f1;
                case 7: goto L_0x03a3;
                case 8: goto L_0x00c8;
                default: goto L_0x009f;
            }
        L_0x009f:
            r20 = r10
            r24 = r12
            r40 = r14
            r12 = r9
            java.util.logging.Logger r0 = logger
            java.util.logging.Level r2 = java.util.logging.Level.FINER
            boolean r0 = r0.isLoggable(r2)
            if (r0 == 0) goto L_0x04d3
            java.util.logging.Logger r0 = logger
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            java.lang.String r3 = "DNSIncoming() unknown type:"
            r2.append(r3)
            r2.append(r13)
            java.lang.String r2 = r2.toString()
            r0.finer(r2)
            goto L_0x04d3
        L_0x00c8:
            int r0 = r43.getFlags()
            javax.jmdns.impl.constants.DNSResultCode r4 = javax.jmdns.impl.constants.DNSResultCode.resultCodeForFlags(r0, r10)
            r0 = 16711680(0xff0000, float:2.3418052E-38)
            r0 = r0 & r10
            r5 = 16
            int r6 = r0 >> 16
            if (r6 != 0) goto L_0x037e
            r1._senderUDPPayload = r14
        L_0x00db:
            javax.jmdns.impl.DNSIncoming$MessageInputStream r0 = r1._messageInputStream
            int r0 = r0.available()
            if (r0 <= 0) goto L_0x0375
            r0 = 0
            r7 = 0
            javax.jmdns.impl.DNSIncoming$MessageInputStream r8 = r1._messageInputStream
            int r8 = r8.available()
            r5 = 2
            if (r8 < r5) goto L_0x0366
            javax.jmdns.impl.DNSIncoming$MessageInputStream r8 = r1._messageInputStream
            int r8 = r8.readUnsignedShort()
            javax.jmdns.impl.constants.DNSOptionCode r7 = javax.jmdns.impl.constants.DNSOptionCode.resultCodeForFlags(r8)
            r0 = 0
            javax.jmdns.impl.DNSIncoming$MessageInputStream r2 = r1._messageInputStream
            int r2 = r2.available()
            if (r2 < r5) goto L_0x0353
            javax.jmdns.impl.DNSIncoming$MessageInputStream r2 = r1._messageInputStream
            int r2 = r2.readUnsignedShort()
            byte[] r0 = new byte[r3]
            javax.jmdns.impl.DNSIncoming$MessageInputStream r5 = r1._messageInputStream
            int r5 = r5.available()
            if (r5 < r2) goto L_0x0117
            javax.jmdns.impl.DNSIncoming$MessageInputStream r5 = r1._messageInputStream
            byte[] r0 = r5.readBytes(r2)
        L_0x0117:
            r5 = r0
            int[] r0 = javax.jmdns.impl.DNSIncoming.AnonymousClass1.$SwitchMap$javax$jmdns$impl$constants$DNSOptionCode
            int r21 = r7.ordinal()
            r0 = r0[r21]
            switch(r0) {
                case 1: goto L_0x0190;
                case 2: goto L_0x0156;
                case 3: goto L_0x0156;
                case 4: goto L_0x0156;
                case 5: goto L_0x012d;
                default: goto L_0x0123;
            }
        L_0x0123:
            r24 = r12
            r40 = r14
            r18 = 16
            r19 = 1
            goto L_0x0349
        L_0x012d:
            java.util.logging.Logger r0 = logger
            java.util.logging.Level r3 = java.util.logging.Level.WARNING
            r23 = r2
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            r24 = r12
            java.lang.String r12 = "There was an OPT answer. Not currently handled. Option code: "
            r2.append(r12)
            r2.append(r8)
            java.lang.String r12 = " data: "
            r2.append(r12)
            java.lang.String r12 = r1._hexString(r5)
            r2.append(r12)
            java.lang.String r2 = r2.toString()
            r0.log(r3, r2)
            goto L_0x0188
        L_0x0156:
            r23 = r2
            r24 = r12
            java.util.logging.Logger r0 = logger
            java.util.logging.Level r2 = java.util.logging.Level.FINE
            boolean r0 = r0.isLoggable(r2)
            if (r0 == 0) goto L_0x0188
            java.util.logging.Logger r0 = logger
            java.util.logging.Level r2 = java.util.logging.Level.FINE
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r12 = "There was an OPT answer. Option code: "
            r3.append(r12)
            r3.append(r7)
            java.lang.String r12 = " data: "
            r3.append(r12)
            java.lang.String r12 = r1._hexString(r5)
            r3.append(r12)
            java.lang.String r3 = r3.toString()
            r0.log(r2, r3)
        L_0x0188:
            r40 = r14
            r18 = 16
            r19 = 1
            goto L_0x0349
        L_0x0190:
            r23 = r2
            r24 = r12
            r2 = 0
            r3 = 0
            r12 = 0
            r21 = 0
            r0 = 0
            r25 = r0
            r22 = 0
            byte r0 = r5[r22]     // Catch:{ Exception -> 0x029f }
            r2 = r0
            r19 = 1
            byte r0 = r5[r19]     // Catch:{ Exception -> 0x0299 }
            r3 = r0
            r0 = 6
            r26 = r2
            byte[] r2 = new byte[r0]     // Catch:{ Exception -> 0x0291 }
            r20 = 2
            byte r27 = r5[r20]     // Catch:{ Exception -> 0x0291 }
            r22 = 0
            r2[r22] = r27     // Catch:{ Exception -> 0x0291 }
            r27 = 3
            byte r28 = r5[r27]     // Catch:{ Exception -> 0x0291 }
            r19 = 1
            r2[r19] = r28     // Catch:{ Exception -> 0x028d }
            r0 = 4
            byte r28 = r5[r0]     // Catch:{ Exception -> 0x0291 }
            r20 = 2
            r2[r20] = r28     // Catch:{ Exception -> 0x0291 }
            r28 = 5
            byte r30 = r5[r28]     // Catch:{ Exception -> 0x0291 }
            r2[r27] = r30     // Catch:{ Exception -> 0x0291 }
            r29 = 6
            byte r30 = r5[r29]     // Catch:{ Exception -> 0x0291 }
            r2[r0] = r30     // Catch:{ Exception -> 0x0291 }
            r30 = 7
            byte r31 = r5[r30]     // Catch:{ Exception -> 0x0291 }
            r2[r28] = r31     // Catch:{ Exception -> 0x0291 }
            r12 = r2
            r21 = r12
            int r2 = r5.length     // Catch:{ Exception -> 0x0291 }
            r0 = 8
            if (r2 <= r0) goto L_0x0212
            r2 = 6
            byte[] r0 = new byte[r2]     // Catch:{ Exception -> 0x020d }
            r2 = 8
            byte r31 = r5[r2]     // Catch:{ Exception -> 0x020d }
            r2 = 0
            r0[r2] = r31     // Catch:{ Exception -> 0x020d }
            r2 = 9
            byte r2 = r5[r2]     // Catch:{ Exception -> 0x020d }
            r19 = 1
            r0[r19] = r2     // Catch:{ Exception -> 0x020d }
            r2 = 10
            byte r2 = r5[r2]     // Catch:{ Exception -> 0x020d }
            r20 = 2
            r0[r20] = r2     // Catch:{ Exception -> 0x020d }
            r2 = 11
            byte r2 = r5[r2]     // Catch:{ Exception -> 0x020d }
            r0[r27] = r2     // Catch:{ Exception -> 0x020d }
            r2 = 12
            byte r2 = r5[r2]     // Catch:{ Exception -> 0x020d }
            r31 = 4
            r0[r31] = r2     // Catch:{ Exception -> 0x020d }
            r2 = 13
            byte r2 = r5[r2]     // Catch:{ Exception -> 0x020d }
            r0[r28] = r2     // Catch:{ Exception -> 0x020d }
            r21 = r0
            goto L_0x0212
        L_0x020d:
            r0 = move-exception
            r2 = r26
            goto L_0x02a0
        L_0x0212:
            int r0 = r5.length     // Catch:{ Exception -> 0x0291 }
            r31 = 15
            r32 = 14
            r2 = 18
            if (r0 != r2) goto L_0x023b
            r0 = 4
            byte[] r2 = new byte[r0]     // Catch:{ Exception -> 0x020d }
            byte r0 = r5[r32]     // Catch:{ Exception -> 0x020d }
            r22 = 0
            r2[r22] = r0     // Catch:{ Exception -> 0x020d }
            byte r0 = r5[r31]     // Catch:{ Exception -> 0x020d }
            r19 = 1
            r2[r19] = r0     // Catch:{ Exception -> 0x020d }
            r18 = 16
            byte r0 = r5[r18]     // Catch:{ Exception -> 0x020d }
            r20 = 2
            r2[r20] = r0     // Catch:{ Exception -> 0x020d }
            r0 = 17
            byte r33 = r5[r0]     // Catch:{ Exception -> 0x020d }
            r2[r27] = r33     // Catch:{ Exception -> 0x020d }
            r0 = r2
            r25 = r0
        L_0x023b:
            int r0 = r5.length     // Catch:{ Exception -> 0x0291 }
            r2 = 22
            if (r0 != r2) goto L_0x027f
            r0 = 8
            byte[] r0 = new byte[r0]     // Catch:{ Exception -> 0x0291 }
            byte r2 = r5[r32]     // Catch:{ Exception -> 0x0291 }
            r22 = 0
            r0[r22] = r2     // Catch:{ Exception -> 0x0291 }
            byte r2 = r5[r31]     // Catch:{ Exception -> 0x0291 }
            r19 = 1
            r0[r19] = r2     // Catch:{ Exception -> 0x028d }
            r18 = 16
            byte r2 = r5[r18]     // Catch:{ Exception -> 0x027d }
            r20 = 2
            r0[r20] = r2     // Catch:{ Exception -> 0x027d }
            r2 = 17
            byte r2 = r5[r2]     // Catch:{ Exception -> 0x027d }
            r0[r27] = r2     // Catch:{ Exception -> 0x027d }
            r2 = 18
            byte r2 = r5[r2]     // Catch:{ Exception -> 0x027d }
            r20 = 4
            r0[r20] = r2     // Catch:{ Exception -> 0x027d }
            r2 = 19
            byte r2 = r5[r2]     // Catch:{ Exception -> 0x027d }
            r0[r28] = r2     // Catch:{ Exception -> 0x027d }
            r2 = 20
            byte r2 = r5[r2]     // Catch:{ Exception -> 0x027d }
            r20 = 6
            r0[r20] = r2     // Catch:{ Exception -> 0x027d }
            r2 = 21
            byte r2 = r5[r2]     // Catch:{ Exception -> 0x027d }
            r0[r30] = r2     // Catch:{ Exception -> 0x027d }
            r25 = r0
            goto L_0x0283
        L_0x027d:
            r0 = move-exception
            goto L_0x0296
        L_0x027f:
            r18 = 16
            r19 = 1
        L_0x0283:
            r37 = r5
            r0 = r21
            r2 = r25
            r5 = r3
            r3 = r26
            goto L_0x02ce
        L_0x028d:
            r0 = move-exception
            r18 = 16
            goto L_0x0296
        L_0x0291:
            r0 = move-exception
            r18 = 16
            r19 = 1
        L_0x0296:
            r2 = r26
            goto L_0x02a4
        L_0x0299:
            r0 = move-exception
            r26 = r2
            r18 = 16
            goto L_0x02a4
        L_0x029f:
            r0 = move-exception
        L_0x02a0:
            r18 = 16
            r19 = 1
        L_0x02a4:
            r34 = r0
            java.util.logging.Logger r0 = logger
            r35 = r2
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r2.<init>()
            r36 = r3
            java.lang.String r3 = "Malformed OPT answer. Option code: Owner data: "
            r2.append(r3)
            java.lang.String r3 = r1._hexString(r5)
            r2.append(r3)
            java.lang.String r2 = r2.toString()
            r0.warning(r2)
            r37 = r5
            r0 = r21
            r2 = r25
            r3 = r35
            r5 = r36
        L_0x02ce:
            r38 = r7
            java.util.logging.Logger r7 = logger
            r39 = r8
            java.util.logging.Level r8 = java.util.logging.Level.FINE
            boolean r7 = r7.isLoggable(r8)
            if (r7 == 0) goto L_0x0347
            java.util.logging.Logger r7 = logger
            java.lang.StringBuilder r8 = new java.lang.StringBuilder
            r8.<init>()
            r40 = r14
            java.lang.String r14 = "Unhandled Owner OPT version: "
            r8.append(r14)
            r8.append(r3)
            java.lang.String r14 = " sequence: "
            r8.append(r14)
            r8.append(r5)
            java.lang.String r14 = " MAC address: "
            r8.append(r14)
            java.lang.String r14 = r1._hexString(r12)
            r8.append(r14)
            if (r0 == r12) goto L_0x031b
            java.lang.StringBuilder r14 = new java.lang.StringBuilder
            r14.<init>()
            r41 = r3
            java.lang.String r3 = " wakeup MAC address: "
            r14.append(r3)
            java.lang.String r3 = r1._hexString(r0)
            r14.append(r3)
            java.lang.String r3 = r14.toString()
            goto L_0x031f
        L_0x031b:
            r41 = r3
            java.lang.String r3 = ""
        L_0x031f:
            r8.append(r3)
            if (r2 == 0) goto L_0x033a
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r14 = " password: "
            r3.append(r14)
            java.lang.String r14 = r1._hexString(r2)
            r3.append(r14)
            java.lang.String r3 = r3.toString()
            goto L_0x033c
        L_0x033a:
            java.lang.String r3 = ""
        L_0x033c:
            r8.append(r3)
            java.lang.String r3 = r8.toString()
            r7.fine(r3)
            goto L_0x0349
        L_0x0347:
            r40 = r14
        L_0x0349:
            r12 = r24
            r14 = r40
            r2 = 1
            r3 = 0
            r5 = 16
            goto L_0x00db
        L_0x0353:
            r38 = r7
            r39 = r8
            r24 = r12
            r40 = r14
            java.util.logging.Logger r2 = logger
            java.util.logging.Level r3 = java.util.logging.Level.WARNING
            java.lang.String r5 = "There was a problem reading the OPT record. Ignoring."
            r2.log(r3, r5)
            goto L_0x03ec
        L_0x0366:
            r24 = r12
            r40 = r14
            java.util.logging.Logger r2 = logger
            java.util.logging.Level r3 = java.util.logging.Level.WARNING
            java.lang.String r5 = "There was a problem reading the OPT record. Ignoring."
            r2.log(r3, r5)
            goto L_0x03ec
        L_0x0375:
            r24 = r12
            r40 = r14
            r12 = r9
            r20 = r10
            goto L_0x04d9
        L_0x037e:
            r24 = r12
            r40 = r14
            java.util.logging.Logger r0 = logger
            java.util.logging.Level r2 = java.util.logging.Level.WARNING
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r5 = "There was an OPT answer. Wrong version number: "
            r3.append(r5)
            r3.append(r6)
            java.lang.String r5 = " result code: "
            r3.append(r5)
            r3.append(r4)
            java.lang.String r3 = r3.toString()
            r0.log(r2, r3)
            goto L_0x03ec
        L_0x03a3:
            r24 = r12
            r40 = r14
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            javax.jmdns.impl.DNSIncoming$MessageInputStream r2 = r1._messageInputStream
            java.lang.String r2 = r2.readUTF(r9)
            r0.append(r2)
            java.lang.String r2 = " "
            int r12 = r0.indexOf(r2)
            if (r12 <= 0) goto L_0x03c3
            r2 = 0
            java.lang.String r2 = r0.substring(r2, r12)
            goto L_0x03c7
        L_0x03c3:
            java.lang.String r2 = r0.toString()
        L_0x03c7:
            java.lang.String r14 = r2.trim()
            if (r12 <= 0) goto L_0x03d4
            int r2 = r12 + 1
            java.lang.String r2 = r0.substring(r2)
            goto L_0x03d6
        L_0x03d4:
            java.lang.String r2 = ""
        L_0x03d6:
            java.lang.String r18 = r2.trim()
            javax.jmdns.impl.DNSRecord$HostInformation r19 = new javax.jmdns.impl.DNSRecord$HostInformation
            r2 = r19
            r3 = r11
            r4 = r15
            r5 = r16
            r6 = r10
            r7 = r14
            r8 = r18
            r2.<init>(r3, r4, r5, r6, r7, r8)
            r17 = r19
        L_0x03ec:
            r12 = r9
            r20 = r10
            goto L_0x04d9
        L_0x03f1:
            r24 = r12
            r40 = r14
            javax.jmdns.impl.DNSIncoming$MessageInputStream r0 = r1._messageInputStream
            int r0 = r0.readUnsignedShort()
            javax.jmdns.impl.DNSIncoming$MessageInputStream r2 = r1._messageInputStream
            int r12 = r2.readUnsignedShort()
            javax.jmdns.impl.DNSIncoming$MessageInputStream r2 = r1._messageInputStream
            int r14 = r2.readUnsignedShort()
            java.lang.String r2 = ""
            boolean r3 = USE_DOMAIN_NAME_FORMAT_FOR_SRV_TARGET
            if (r3 == 0) goto L_0x0416
            javax.jmdns.impl.DNSIncoming$MessageInputStream r3 = r1._messageInputStream
            java.lang.String r2 = r3.readName()
        L_0x0413:
            r18 = r2
            goto L_0x041d
        L_0x0416:
            javax.jmdns.impl.DNSIncoming$MessageInputStream r3 = r1._messageInputStream
            java.lang.String r2 = r3.readNonNameString()
            goto L_0x0413
        L_0x041d:
            javax.jmdns.impl.DNSRecord$Service r19 = new javax.jmdns.impl.DNSRecord$Service
            r2 = r19
            r3 = r11
            r4 = r15
            r5 = r16
            r6 = r10
            r7 = r0
            r8 = r12
            r42 = r12
            r12 = r9
            r9 = r14
            r20 = r10
            r10 = r18
            r2.<init>(r3, r4, r5, r6, r7, r8, r9, r10)
            r17 = r19
            goto L_0x04d9
        L_0x0437:
            r20 = r10
            r24 = r12
            r40 = r14
            r12 = r9
            javax.jmdns.impl.DNSRecord$Text r0 = new javax.jmdns.impl.DNSRecord$Text
            javax.jmdns.impl.DNSIncoming$MessageInputStream r2 = r1._messageInputStream
            byte[] r7 = r2.readBytes(r12)
            r2 = r0
            r3 = r11
            r4 = r15
            r5 = r16
            r6 = r20
            r2.<init>(r3, r4, r5, r6, r7)
            r17 = r0
            goto L_0x04db
        L_0x0454:
            r20 = r10
            r24 = r12
            r40 = r14
            r12 = r9
            java.lang.String r0 = ""
            javax.jmdns.impl.DNSIncoming$MessageInputStream r2 = r1._messageInputStream
            java.lang.String r0 = r2.readName()
            int r2 = r0.length()
            if (r2 <= 0) goto L_0x047a
            javax.jmdns.impl.DNSRecord$Pointer r8 = new javax.jmdns.impl.DNSRecord$Pointer
            r2 = r8
            r3 = r11
            r4 = r15
            r5 = r16
            r6 = r20
            r7 = r0
            r2.<init>(r3, r4, r5, r6, r7)
            r17 = r8
            goto L_0x04d9
        L_0x047a:
            java.util.logging.Logger r2 = logger
            java.util.logging.Level r3 = java.util.logging.Level.WARNING
            java.lang.StringBuilder r4 = new java.lang.StringBuilder
            r4.<init>()
            java.lang.String r5 = "PTR record of class: "
            r4.append(r5)
            r4.append(r15)
            java.lang.String r5 = ", there was a problem reading the service name of the answer for domain:"
            r4.append(r5)
            r4.append(r11)
            java.lang.String r4 = r4.toString()
            r2.log(r3, r4)
            goto L_0x04d9
        L_0x049b:
            r20 = r10
            r24 = r12
            r40 = r14
            r12 = r9
            javax.jmdns.impl.DNSRecord$IPv6Address r0 = new javax.jmdns.impl.DNSRecord$IPv6Address
            javax.jmdns.impl.DNSIncoming$MessageInputStream r2 = r1._messageInputStream
            byte[] r7 = r2.readBytes(r12)
            r2 = r0
            r3 = r11
            r4 = r15
            r5 = r16
            r6 = r20
            r2.<init>((java.lang.String) r3, (javax.jmdns.impl.constants.DNSRecordClass) r4, (boolean) r5, (int) r6, (byte[]) r7)
            r17 = r0
            goto L_0x04db
        L_0x04b7:
            r20 = r10
            r24 = r12
            r40 = r14
            r12 = r9
            javax.jmdns.impl.DNSRecord$IPv4Address r0 = new javax.jmdns.impl.DNSRecord$IPv4Address
            javax.jmdns.impl.DNSIncoming$MessageInputStream r2 = r1._messageInputStream
            byte[] r7 = r2.readBytes(r12)
            r2 = r0
            r3 = r11
            r4 = r15
            r5 = r16
            r6 = r20
            r2.<init>((java.lang.String) r3, (javax.jmdns.impl.constants.DNSRecordClass) r4, (boolean) r5, (int) r6, (byte[]) r7)
            r17 = r0
            goto L_0x04db
        L_0x04d3:
            javax.jmdns.impl.DNSIncoming$MessageInputStream r0 = r1._messageInputStream
            long r2 = (long) r12
            r0.skip(r2)
        L_0x04d9:
            r0 = r17
        L_0x04db:
            if (r0 == 0) goto L_0x04e3
            r2 = r44
            r0.setRecordSource(r2)
            goto L_0x04e5
        L_0x04e3:
            r2 = r44
        L_0x04e5:
            return r0
        */
        throw new UnsupportedOperationException("Method not decompiled: javax.jmdns.impl.DNSIncoming.readAnswer(java.net.InetAddress):javax.jmdns.impl.DNSRecord");
    }

    /* access modifiers changed from: package-private */
    public String print(boolean dump) {
        StringBuilder buf = new StringBuilder();
        buf.append(print());
        if (dump) {
            byte[] data = new byte[this._packet.getLength()];
            System.arraycopy(this._packet.getData(), 0, data, 0, data.length);
            buf.append(print(data));
        }
        return buf.toString();
    }

    public String toString() {
        StringBuilder buf = new StringBuilder();
        buf.append(isQuery() ? "dns[query," : "dns[response,");
        if (this._packet.getAddress() != null) {
            buf.append(this._packet.getAddress().getHostAddress());
        }
        buf.append(':');
        buf.append(this._packet.getPort());
        buf.append(", length=");
        buf.append(this._packet.getLength());
        buf.append(", id=0x");
        buf.append(Integer.toHexString(getId()));
        if (getFlags() != 0) {
            buf.append(", flags=0x");
            buf.append(Integer.toHexString(getFlags()));
            if ((getFlags() & 32768) != 0) {
                buf.append(":r");
            }
            if ((getFlags() & 1024) != 0) {
                buf.append(":aa");
            }
            if ((getFlags() & 512) != 0) {
                buf.append(":tc");
            }
        }
        if (getNumberOfQuestions() > 0) {
            buf.append(", questions=");
            buf.append(getNumberOfQuestions());
        }
        if (getNumberOfAnswers() > 0) {
            buf.append(", answers=");
            buf.append(getNumberOfAnswers());
        }
        if (getNumberOfAuthorities() > 0) {
            buf.append(", authorities=");
            buf.append(getNumberOfAuthorities());
        }
        if (getNumberOfAdditionals() > 0) {
            buf.append(", additionals=");
            buf.append(getNumberOfAdditionals());
        }
        if (getNumberOfQuestions() > 0) {
            buf.append("\nquestions:");
            for (DNSQuestion question : this._questions) {
                buf.append("\n\t");
                buf.append(question);
            }
        }
        if (getNumberOfAnswers() > 0) {
            buf.append("\nanswers:");
            for (DNSRecord record : this._answers) {
                buf.append("\n\t");
                buf.append(record);
            }
        }
        if (getNumberOfAuthorities() > 0) {
            buf.append("\nauthorities:");
            for (DNSRecord record2 : this._authoritativeAnswers) {
                buf.append("\n\t");
                buf.append(record2);
            }
        }
        if (getNumberOfAdditionals() > 0) {
            buf.append("\nadditionals:");
            for (DNSRecord record3 : this._additionals) {
                buf.append("\n\t");
                buf.append(record3);
            }
        }
        buf.append("]");
        return buf.toString();
    }

    /* access modifiers changed from: package-private */
    public void append(DNSIncoming that) {
        if (!isQuery() || !isTruncated() || !that.isQuery()) {
            throw new IllegalArgumentException();
        }
        this._questions.addAll(that.getQuestions());
        this._answers.addAll(that.getAnswers());
        this._authoritativeAnswers.addAll(that.getAuthorities());
        this._additionals.addAll(that.getAdditionals());
    }

    public int elapseSinceArrival() {
        return (int) (System.currentTimeMillis() - this._receivedTime);
    }

    public int getSenderUDPPayload() {
        return this._senderUDPPayload;
    }

    private String _hexString(byte[] bytes) {
        StringBuilder result = new StringBuilder(bytes.length * 2);
        for (byte b : bytes) {
            int b2 = b & 255;
            result.append(_nibbleToHex[b2 / 16]);
            result.append(_nibbleToHex[b2 % 16]);
        }
        return result.toString();
    }
}
