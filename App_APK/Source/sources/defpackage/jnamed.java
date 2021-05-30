package defpackage;

import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.InterruptedIOException;
import java.io.PrintStream;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.StringTokenizer;
import org.xbill.DNS.Address;
import org.xbill.DNS.CNAMERecord;
import org.xbill.DNS.Cache;
import org.xbill.DNS.DNAMERecord;
import org.xbill.DNS.Header;
import org.xbill.DNS.Message;
import org.xbill.DNS.Name;
import org.xbill.DNS.NameTooLongException;
import org.xbill.DNS.RRset;
import org.xbill.DNS.Record;
import org.xbill.DNS.SetResponse;
import org.xbill.DNS.TSIG;
import org.xbill.DNS.TSIGRecord;
import org.xbill.DNS.Zone;
import org.xbill.DNS.ZoneTransferException;

/* renamed from: jnamed  reason: default package */
public class jnamed {
    static final int FLAG_DNSSECOK = 1;
    static final int FLAG_SIGONLY = 2;
    Map TSIGs;
    Map caches;
    Map znames;

    private static String addrport(InetAddress addr, int port) {
        StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append(addr.getHostAddress());
        stringBuffer.append("#");
        stringBuffer.append(port);
        return stringBuffer.toString();
    }

    public jnamed(String conffile) throws IOException, ZoneTransferException {
        List<Integer> ports = new ArrayList<>();
        List<InetAddress> addresses = new ArrayList<>();
        try {
            FileInputStream fs = new FileInputStream(conffile);
            try {
                try {
                    BufferedReader br = new BufferedReader(new InputStreamReader(fs));
                    try {
                        this.caches = new HashMap();
                        this.znames = new HashMap();
                        this.TSIGs = new HashMap();
                        while (true) {
                            String readLine = br.readLine();
                            String line = readLine;
                            if (readLine == null) {
                                break;
                            }
                            StringTokenizer st = new StringTokenizer(line);
                            if (st.hasMoreTokens()) {
                                String keyword = st.nextToken();
                                if (!st.hasMoreTokens()) {
                                    PrintStream printStream = System.out;
                                    StringBuffer stringBuffer = new StringBuffer();
                                    stringBuffer.append("Invalid line: ");
                                    stringBuffer.append(line);
                                    printStream.println(stringBuffer.toString());
                                } else if (keyword.charAt(0) != '#') {
                                    if (keyword.equals("primary")) {
                                        addPrimaryZone(st.nextToken(), st.nextToken());
                                    } else if (keyword.equals("secondary")) {
                                        addSecondaryZone(st.nextToken(), st.nextToken());
                                    } else if (keyword.equals("cache")) {
                                        this.caches.put(new Integer(1), new Cache(st.nextToken()));
                                    } else if (keyword.equals("key")) {
                                        String s1 = st.nextToken();
                                        String s2 = st.nextToken();
                                        if (st.hasMoreTokens()) {
                                            addTSIG(s1, s2, st.nextToken());
                                        } else {
                                            addTSIG("hmac-md5", s1, s2);
                                        }
                                    } else if (keyword.equals("port")) {
                                        ports.add(Integer.valueOf(st.nextToken()));
                                    } else if (keyword.equals("address")) {
                                        addresses.add(Address.getByAddress(st.nextToken()));
                                    } else {
                                        PrintStream printStream2 = System.out;
                                        StringBuffer stringBuffer2 = new StringBuffer();
                                        stringBuffer2.append("unknown keyword: ");
                                        stringBuffer2.append(keyword);
                                        printStream2.println(stringBuffer2.toString());
                                    }
                                }
                            }
                        }
                        if (ports.size() == 0) {
                            ports.add(new Integer(53));
                        }
                        if (addresses.size() == 0) {
                            addresses.add(Address.getByAddress("0.0.0.0"));
                        }
                        for (InetAddress addr : addresses) {
                            for (Integer intValue : ports) {
                                int port = intValue.intValue();
                                addUDP(addr, port);
                                addTCP(addr, port);
                                PrintStream printStream3 = System.out;
                                StringBuffer stringBuffer3 = new StringBuffer();
                                stringBuffer3.append("jnamed: listening on ");
                                stringBuffer3.append(addrport(addr, port));
                                printStream3.println(stringBuffer3.toString());
                            }
                        }
                        System.out.println("jnamed: running");
                    } finally {
                        fs.close();
                    }
                } catch (Exception e) {
                }
            } catch (Exception e2) {
                Exception exc = e2;
                PrintStream printStream4 = System.out;
                StringBuffer stringBuffer4 = new StringBuffer();
                stringBuffer4.append("Cannot open ");
                stringBuffer4.append(conffile);
                printStream4.println(stringBuffer4.toString());
            }
        } catch (Exception e3) {
            Exception exc2 = e3;
            PrintStream printStream42 = System.out;
            StringBuffer stringBuffer42 = new StringBuffer();
            stringBuffer42.append("Cannot open ");
            stringBuffer42.append(conffile);
            printStream42.println(stringBuffer42.toString());
        }
    }

    public void addPrimaryZone(String zname, String zonefile) throws IOException {
        Name origin = null;
        if (zname != null) {
            origin = Name.fromString(zname, Name.root);
        }
        Zone newzone = new Zone(origin, zonefile);
        this.znames.put(newzone.getOrigin(), newzone);
    }

    public void addSecondaryZone(String zone, String remote) throws IOException, ZoneTransferException {
        Name zname = Name.fromString(zone, Name.root);
        this.znames.put(zname, new Zone(zname, 1, remote));
    }

    public void addTSIG(String algstr, String namestr, String key) throws IOException {
        this.TSIGs.put(Name.fromString(namestr, Name.root), new TSIG(algstr, namestr, key));
    }

    public Cache getCache(int dclass) {
        Cache c = (Cache) this.caches.get(new Integer(dclass));
        if (c != null) {
            return c;
        }
        Cache c2 = new Cache(dclass);
        this.caches.put(new Integer(dclass), c2);
        return c2;
    }

    public Zone findBestZone(Name name) {
        Zone foundzone = (Zone) this.znames.get(name);
        if (foundzone != null) {
            return foundzone;
        }
        int labels = name.labels();
        for (int i = 1; i < labels; i++) {
            Zone foundzone2 = (Zone) this.znames.get(new Name(name, i));
            if (foundzone2 != null) {
                return foundzone2;
            }
        }
        return null;
    }

    public RRset findExactMatch(Name name, int type, int dclass, boolean glue) {
        RRset[] rrsets;
        Zone zone = findBestZone(name);
        if (zone != null) {
            return zone.findExactMatch(name, type);
        }
        Cache cache = getCache(dclass);
        if (glue) {
            rrsets = cache.findAnyRecords(name, type);
        } else {
            rrsets = cache.findRecords(name, type);
        }
        if (rrsets == null) {
            return null;
        }
        return rrsets[0];
    }

    /* access modifiers changed from: package-private */
    public void addRRset(Name name, Message response, RRset rrset, int section, int flags) {
        int s = 1;
        while (s <= section) {
            if (!response.findRRset(name, rrset.getType(), s)) {
                s++;
            } else {
                return;
            }
        }
        if ((flags & 2) == 0) {
            Iterator it = rrset.rrs();
            while (it.hasNext()) {
                Record r = (Record) it.next();
                if (r.getName().isWild() && !name.isWild()) {
                    r = r.withName(name);
                }
                response.addRecord(r, section);
            }
        }
        if ((flags & 3) != 0) {
            Iterator it2 = rrset.sigs();
            while (it2.hasNext()) {
                Record r2 = (Record) it2.next();
                if (r2.getName().isWild() && !name.isWild()) {
                    r2 = r2.withName(name);
                }
                response.addRecord(r2, section);
            }
        }
    }

    private final void addSOA(Message response, Zone zone) {
        response.addRecord(zone.getSOA(), 2);
    }

    private final void addNS(Message response, Zone zone, int flags) {
        RRset nsRecords = zone.getNS();
        addRRset(nsRecords.getName(), response, nsRecords, 2, flags);
    }

    private final void addCacheNS(Message response, Cache cache, Name name) {
        SetResponse sr = cache.lookupRecords(name, 2, 0);
        if (sr.isDelegation()) {
            Iterator it = sr.getNS().rrs();
            while (it.hasNext()) {
                response.addRecord((Record) it.next(), 2);
            }
        }
    }

    private void addGlue(Message response, Name name, int flags) {
        RRset a = findExactMatch(name, 1, 1, true);
        if (a != null) {
            addRRset(name, response, a, 3, flags);
        }
    }

    private void addAdditional2(Message response, int section, int flags) {
        Record[] records = response.getSectionArray(section);
        for (Record r : records) {
            Name glueName = r.getAdditionalName();
            if (glueName != null) {
                addGlue(response, glueName, flags);
            }
        }
    }

    private final void addAdditional(Message response, int flags) {
        addAdditional2(response, 1, flags);
        addAdditional2(response, 2, flags);
    }

    /* access modifiers changed from: package-private */
    public byte addAnswer(Message response, Name name, int type, int dclass, int iterations, int flags) {
        int type2;
        int flags2;
        SetResponse sr;
        Message message = response;
        Name name2 = name;
        int i = type;
        int i2 = dclass;
        int i3 = iterations;
        byte rcode = 0;
        int i4 = 0;
        if (i3 > 6) {
            return 0;
        }
        if (i == 24 || i == 46) {
            type2 = 255;
            flags2 = flags | 2;
        } else {
            flags2 = flags;
            type2 = i;
        }
        Zone zone = findBestZone(name2);
        if (zone != null) {
            sr = zone.findRecords(name2, type2);
        } else {
            sr = getCache(i2).lookupRecords(name2, type2, 3);
        }
        SetResponse sr2 = sr;
        if (sr2.isUnknown()) {
            addCacheNS(message, getCache(i2), name2);
        }
        if (sr2.isNXDOMAIN()) {
            response.getHeader().setRcode(3);
            if (zone != null) {
                addSOA(message, zone);
                if (i3 == 0) {
                    response.getHeader().setFlag(5);
                }
            }
            rcode = 3;
        } else if (sr2.isNXRRSET()) {
            if (zone != null) {
                addSOA(message, zone);
                if (i3 == 0) {
                    response.getHeader().setFlag(5);
                }
            }
        } else if (sr2.isDelegation()) {
            RRset nsRecords = sr2.getNS();
            addRRset(nsRecords.getName(), response, nsRecords, 2, flags2);
        } else if (sr2.isCNAME()) {
            CNAMERecord cname = sr2.getCNAME();
            addRRset(name, response, new RRset((Record) cname), 1, flags2);
            if (zone != null && i3 == 0) {
                response.getHeader().setFlag(5);
            }
            int flags3 = flags2;
            int i5 = type2;
            int i6 = i3;
            rcode = addAnswer(response, cname.getTarget(), type2, dclass, i3 + 1, flags3);
            int i7 = flags3;
            Zone zone2 = zone;
            Name name3 = name2;
            int i8 = i2;
            Name name4 = name3;
            return rcode;
        } else {
            Zone zone3 = zone;
            int flags4 = flags2;
            int type3 = type2;
            int i9 = i3;
            if (sr2.isDNAME()) {
                DNAMERecord dname = sr2.getDNAME();
                int flags5 = flags4;
                Zone zone4 = zone3;
                addRRset(name, response, new RRset((Record) dname), 1, flags5);
                try {
                    Name newname = name2.fromDNAME(dname);
                    int i10 = i2;
                    Name name5 = name2;
                    addRRset(name, response, new RRset((Record) new CNAMERecord(name, dclass, 0, newname)), 1, flags5);
                    if (zone4 != null && i9 == 0) {
                        response.getHeader().setFlag(5);
                    }
                    Name name6 = name5;
                    int i11 = i10;
                    Zone zone5 = zone4;
                    int flags6 = flags5;
                    DNAMERecord dNAMERecord = dname;
                    int i12 = flags6;
                    Zone zone6 = zone5;
                    return addAnswer(response, newname, type3, dclass, i9 + 1, flags6);
                } catch (NameTooLongException e) {
                    Zone zone7 = zone4;
                    int i13 = flags5;
                    DNAMERecord dNAMERecord2 = dname;
                    Name name7 = name2;
                    int i14 = i2;
                    Name name8 = name7;
                    NameTooLongException nameTooLongException = e;
                    return 6;
                }
            } else {
                Name name9 = name2;
                int i15 = i2;
                Name name10 = name9;
                if (sr2.isSuccessful()) {
                    RRset[] rrsets = sr2.answers();
                    while (true) {
                        int i16 = i4;
                        if (i16 >= rrsets.length) {
                            break;
                        }
                        int flags7 = flags4;
                        addRRset(name, response, rrsets[i16], 1, flags7);
                        i4 = i16 + 1;
                        flags4 = flags7;
                        zone3 = zone3;
                    }
                    int flags8 = flags4;
                    Zone zone8 = zone3;
                    if (zone8 != null) {
                        addNS(message, zone8, flags8);
                        if (i9 != 0) {
                            return 0;
                        }
                        response.getHeader().setFlag(5);
                        return 0;
                    }
                    addCacheNS(message, getCache(i15), name10);
                    return 0;
                }
                Zone zone9 = zone3;
                return 0;
            }
        }
        int i17 = i3;
        Name name32 = name2;
        int i82 = i2;
        Name name42 = name32;
        return rcode;
    }

    /* access modifiers changed from: package-private */
    public byte[] doAXFR(Name name, Message query, TSIG tsig, TSIGRecord qtsig, Socket s) {
        TSIG tsig2 = tsig;
        Zone zone = (Zone) this.znames.get(name);
        int i = 5;
        if (zone == null) {
            return errorMessage(query, 5);
        }
        Message message = query;
        Iterator it = zone.AXFR();
        try {
            DataOutputStream dataOut = new DataOutputStream(s.getOutputStream());
            int id = query.getHeader().getID();
            TSIGRecord qtsig2 = qtsig;
            boolean first = true;
            while (true) {
                int id2 = id;
                try {
                    if (it.hasNext()) {
                        RRset rrset = (RRset) it.next();
                        Message response = new Message(id2);
                        Header header = response.getHeader();
                        header.setFlag(0);
                        header.setFlag(i);
                        Header header2 = header;
                        Message response2 = response;
                        int id3 = id2;
                        addRRset(rrset.getName(), response, rrset, 1, 1);
                        if (tsig2 != null) {
                            tsig2.applyStream(response2, qtsig2, first);
                            qtsig2 = response2.getTSIG();
                        }
                        first = false;
                        byte[] out = response2.toWire();
                        dataOut.writeShort(out.length);
                        dataOut.write(out);
                        id = id3;
                        i = 5;
                    }
                } catch (IOException e) {
                    System.out.println("AXFR failed");
                    break;
                    s.close();
                    return null;
                }
                try {
                    break;
                } catch (IOException e2) {
                    return null;
                }
            }
        } catch (IOException e3) {
            TSIGRecord tSIGRecord = qtsig;
            System.out.println("AXFR failed");
            break;
            s.close();
            return null;
        }
        s.close();
        return null;
    }

    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r4v6, resolved type: java.lang.Object} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r1v23, resolved type: org.xbill.DNS.TSIG} */
    /* access modifiers changed from: package-private */
    /* JADX WARNING: Code restructure failed: missing block: B:16:0x004c, code lost:
        if (r1.verify(r8, r9, r24, (org.xbill.DNS.TSIGRecord) null) != 0) goto L_0x0051;
     */
    /* JADX WARNING: Multi-variable type inference failed */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public byte[] generateReply(org.xbill.DNS.Message r22, byte[] r23, int r24, java.net.Socket r25) throws java.io.IOException {
        /*
            r21 = this;
            r7 = r21
            r8 = r22
            r9 = r23
            r0 = 0
            org.xbill.DNS.Header r11 = r22.getHeader()
            r12 = 0
            boolean r1 = r11.getFlag(r12)
            r2 = 0
            if (r1 == 0) goto L_0x0014
            return r2
        L_0x0014:
            int r1 = r11.getRcode()
            r13 = 1
            if (r1 == 0) goto L_0x0020
            byte[] r1 = r7.errorMessage(r8, r13)
            return r1
        L_0x0020:
            int r1 = r11.getOpcode()
            r3 = 4
            if (r1 == 0) goto L_0x002c
            byte[] r1 = r7.errorMessage(r8, r3)
            return r1
        L_0x002c:
            org.xbill.DNS.Record r14 = r22.getQuestion()
            org.xbill.DNS.TSIGRecord r15 = r22.getTSIG()
            r1 = 0
            if (r15 == 0) goto L_0x0056
            java.util.Map r4 = r7.TSIGs
            org.xbill.DNS.Name r5 = r15.getName()
            java.lang.Object r4 = r4.get(r5)
            r1 = r4
            org.xbill.DNS.TSIG r1 = (org.xbill.DNS.TSIG) r1
            if (r1 == 0) goto L_0x004f
            r6 = r24
            byte r2 = r1.verify(r8, r9, r6, r2)
            if (r2 == 0) goto L_0x0058
            goto L_0x0051
        L_0x004f:
            r6 = r24
        L_0x0051:
            byte[] r2 = r7.formerrMessage(r9)
            return r2
        L_0x0056:
            r6 = r24
        L_0x0058:
            r5 = r1
            org.xbill.DNS.OPTRecord r16 = r22.getOPT()
            if (r16 == 0) goto L_0x0066
            int r1 = r16.getVersion()
            if (r1 <= 0) goto L_0x0066
        L_0x0066:
            r1 = 512(0x200, float:7.175E-43)
            if (r25 == 0) goto L_0x006e
            r1 = 65535(0xffff, float:9.1834E-41)
            goto L_0x007a
        L_0x006e:
            if (r16 == 0) goto L_0x0079
            int r2 = r16.getPayloadSize()
            int r1 = java.lang.Math.max(r2, r1)
            goto L_0x007a
        L_0x0079:
        L_0x007a:
            r4 = r1
            r17 = 32768(0x8000, float:4.5918E-41)
            if (r16 == 0) goto L_0x0089
            int r1 = r16.getFlags()
            r1 = r1 & r17
            if (r1 == 0) goto L_0x0089
            r0 = 1
        L_0x0089:
            r2 = r0
            org.xbill.DNS.Message r0 = new org.xbill.DNS.Message
            org.xbill.DNS.Header r1 = r22.getHeader()
            int r1 = r1.getID()
            r0.<init>((int) r1)
            r1 = r0
            org.xbill.DNS.Header r0 = r1.getHeader()
            r0.setFlag(r12)
            org.xbill.DNS.Header r0 = r22.getHeader()
            r13 = 7
            boolean r0 = r0.getFlag(r13)
            if (r0 == 0) goto L_0x00b1
            org.xbill.DNS.Header r0 = r1.getHeader()
            r0.setFlag(r13)
        L_0x00b1:
            r1.addRecord(r14, r12)
            org.xbill.DNS.Name r13 = r14.getName()
            int r0 = r14.getType()
            int r18 = r14.getDClass()
            r12 = 252(0xfc, float:3.53E-43)
            if (r0 != r12) goto L_0x00dc
            if (r25 == 0) goto L_0x00dc
            r12 = r0
            r0 = r21
            r3 = r1
            r1 = r13
            r19 = r2
            r2 = r22
            r20 = r3
            r3 = r5
            r9 = r4
            r4 = r15
            r10 = r5
            r5 = r25
            byte[] r0 = r0.doAXFR(r1, r2, r3, r4, r5)
            return r0
        L_0x00dc:
            r12 = r0
            r20 = r1
            r19 = r2
            r9 = r4
            r10 = r5
            boolean r0 = org.xbill.DNS.Type.isRR(r12)
            if (r0 != 0) goto L_0x00f2
            r0 = 255(0xff, float:3.57E-43)
            if (r12 == r0) goto L_0x00f2
            byte[] r0 = r7.errorMessage(r8, r3)
            return r0
        L_0x00f2:
            r5 = 0
            r0 = r21
            r1 = r20
            r2 = r13
            r3 = r12
            r4 = r18
            r6 = r19
            byte r0 = r0.addAnswer(r1, r2, r3, r4, r5, r6)
            r1 = 3
            if (r0 == 0) goto L_0x010b
            if (r0 == r1) goto L_0x010b
            byte[] r1 = r7.errorMessage(r8, r0)
            return r1
        L_0x010b:
            r3 = r19
            r2 = r20
            r7.addAdditional(r2, r3)
            if (r16 == 0) goto L_0x0129
            r4 = 1
            if (r3 != r4) goto L_0x0118
            goto L_0x011a
        L_0x0118:
            r17 = 0
        L_0x011a:
            r4 = r17
            org.xbill.DNS.OPTRecord r5 = new org.xbill.DNS.OPTRecord
            r6 = 4096(0x1000, float:5.74E-42)
            r1 = 0
            r5.<init>(r6, r0, r1, r4)
            r6 = 3
            r2.addRecord(r5, r6)
            goto L_0x012a
        L_0x0129:
            r1 = 0
        L_0x012a:
            r2.setTSIG(r10, r1, r15)
            byte[] r1 = r2.toWire((int) r9)
            return r1
        */
        throw new UnsupportedOperationException("Method not decompiled: defpackage.jnamed.generateReply(org.xbill.DNS.Message, byte[], int, java.net.Socket):byte[]");
    }

    /* access modifiers changed from: package-private */
    public byte[] buildErrorMessage(Header header, int rcode, Record question) {
        Message response = new Message();
        response.setHeader(header);
        for (int i = 0; i < 4; i++) {
            response.removeAllRecords(i);
        }
        if (rcode == 2) {
            response.addRecord(question, 0);
        }
        header.setRcode(rcode);
        return response.toWire();
    }

    public byte[] formerrMessage(byte[] in) {
        try {
            return buildErrorMessage(new Header(in), 1, (Record) null);
        } catch (IOException e) {
            return null;
        }
    }

    public byte[] errorMessage(Message query, int rcode) {
        return buildErrorMessage(query.getHeader(), rcode, query.getQuestion());
    }

    public void TCPclient(Socket s) {
        byte[] response;
        try {
            DataInputStream dataIn = new DataInputStream(s.getInputStream());
            byte[] in = new byte[dataIn.readUnsignedShort()];
            dataIn.readFully(in);
            try {
                response = generateReply(new Message(in), in, in.length, s);
                if (response == null) {
                    try {
                        s.close();
                        return;
                    } catch (IOException e) {
                        return;
                    }
                }
            } catch (IOException e2) {
                response = formerrMessage(in);
            }
            DataOutputStream dataOut = new DataOutputStream(s.getOutputStream());
            dataOut.writeShort(response.length);
            dataOut.write(response);
            try {
                s.close();
            } catch (IOException e3) {
            }
        } catch (IOException e4) {
            PrintStream printStream = System.out;
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("TCPclient(");
            stringBuffer.append(addrport(s.getLocalAddress(), s.getLocalPort()));
            stringBuffer.append("): ");
            stringBuffer.append(e4);
            printStream.println(stringBuffer.toString());
            s.close();
        } catch (Throwable th) {
            try {
                s.close();
            } catch (IOException e5) {
            }
            throw th;
        }
    }

    public void serveTCP(InetAddress addr, int port) {
        try {
            while (true) {
                final Socket s = new ServerSocket(port, 128, addr).accept();
                new Thread(new Runnable() {
                    public void run() {
                        jnamed.this.TCPclient(s);
                    }
                }).start();
            }
        } catch (IOException e) {
            PrintStream printStream = System.out;
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("serveTCP(");
            stringBuffer.append(addrport(addr, port));
            stringBuffer.append("): ");
            stringBuffer.append(e);
            printStream.println(stringBuffer.toString());
        }
    }

    public void serveUDP(InetAddress addr, int port) {
        byte[] response;
        try {
            DatagramSocket sock = new DatagramSocket(port, addr);
            byte[] in = new byte[512];
            DatagramPacket indp = new DatagramPacket(in, in.length);
            DatagramPacket outdp = null;
            while (true) {
                indp.setLength(in.length);
                try {
                    sock.receive(indp);
                    try {
                        response = generateReply(new Message(in), in, indp.getLength(), (Socket) null);
                        if (response == null) {
                        }
                    } catch (IOException e) {
                        response = formerrMessage(in);
                    }
                    if (outdp == null) {
                        outdp = new DatagramPacket(response, response.length, indp.getAddress(), indp.getPort());
                    } else {
                        outdp.setData(response);
                        outdp.setLength(response.length);
                        outdp.setAddress(indp.getAddress());
                        outdp.setPort(indp.getPort());
                    }
                    sock.send(outdp);
                } catch (InterruptedIOException e2) {
                }
            }
        } catch (IOException e3) {
            PrintStream printStream = System.out;
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("serveUDP(");
            stringBuffer.append(addrport(addr, port));
            stringBuffer.append("): ");
            stringBuffer.append(e3);
            printStream.println(stringBuffer.toString());
        }
    }

    public void addTCP(final InetAddress addr, final int port) {
        new Thread(new Runnable() {
            public void run() {
                jnamed.this.serveTCP(addr, port);
            }
        }).start();
    }

    public void addUDP(final InetAddress addr, final int port) {
        new Thread(new Runnable() {
            public void run() {
                jnamed.this.serveUDP(addr, port);
            }
        }).start();
    }

    public static void main(String[] args) {
        String conf;
        if (args.length > 1) {
            System.out.println("usage: jnamed [conf]");
            System.exit(0);
        }
        try {
            if (args.length == 1) {
                conf = args[0];
            } else {
                conf = "jnamed.conf";
            }
            new jnamed(conf);
        } catch (IOException e) {
            System.out.println(e);
        } catch (ZoneTransferException e2) {
            System.out.println(e2);
        }
    }
}
