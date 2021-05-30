package org.xbill.DNS;

import java.io.IOException;
import java.io.PrintStream;
import java.net.InetSocketAddress;
import java.net.SocketAddress;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.List;
import org.xbill.DNS.TSIG;

public class ZoneTransferIn {
    private static final int AXFR = 6;
    private static final int END = 7;
    private static final int FIRSTDATA = 1;
    private static final int INITIALSOA = 0;
    private static final int IXFR_ADD = 5;
    private static final int IXFR_ADDSOA = 4;
    private static final int IXFR_DEL = 3;
    private static final int IXFR_DELSOA = 2;
    private SocketAddress address;
    private List axfr;
    private TCPClient client;
    private long current_serial;
    private int dclass;
    private long end_serial;
    private Record initialsoa;
    private List ixfr;
    private long ixfr_serial;
    private SocketAddress localAddress;
    private int qtype;
    private int rtype;
    private int state;
    private long timeout = 900000;
    private TSIG tsig;
    private TSIG.StreamVerifier verifier;
    private boolean want_fallback;
    private Name zname;

    public static class Delta {
        public List adds;
        public List deletes;
        public long end;
        public long start;

        private Delta() {
            this.adds = new ArrayList();
            this.deletes = new ArrayList();
        }
    }

    private ZoneTransferIn() {
    }

    private ZoneTransferIn(Name zone, int xfrtype, long serial, boolean fallback, SocketAddress address2, TSIG key) {
        this.address = address2;
        this.tsig = key;
        if (zone.isAbsolute()) {
            this.zname = zone;
        } else {
            try {
                this.zname = Name.concatenate(zone, Name.root);
            } catch (NameTooLongException e) {
                throw new IllegalArgumentException("ZoneTransferIn: name too long");
            }
        }
        this.qtype = xfrtype;
        this.dclass = 1;
        this.ixfr_serial = serial;
        this.want_fallback = fallback;
        this.state = 0;
    }

    public static ZoneTransferIn newAXFR(Name zone, SocketAddress address2, TSIG key) {
        return new ZoneTransferIn(zone, 252, 0, false, address2, key);
    }

    public static ZoneTransferIn newAXFR(Name zone, String host, int port, TSIG key) throws UnknownHostException {
        if (port == 0) {
            port = 53;
        }
        return newAXFR(zone, (SocketAddress) new InetSocketAddress(host, port), key);
    }

    public static ZoneTransferIn newAXFR(Name zone, String host, TSIG key) throws UnknownHostException {
        return newAXFR(zone, host, 0, key);
    }

    public static ZoneTransferIn newIXFR(Name zone, long serial, boolean fallback, SocketAddress address2, TSIG key) {
        return new ZoneTransferIn(zone, 251, serial, fallback, address2, key);
    }

    public static ZoneTransferIn newIXFR(Name zone, long serial, boolean fallback, String host, int port, TSIG key) throws UnknownHostException {
        if (port == 0) {
            port = 53;
        }
        return newIXFR(zone, serial, fallback, (SocketAddress) new InetSocketAddress(host, port), key);
    }

    public static ZoneTransferIn newIXFR(Name zone, long serial, boolean fallback, String host, TSIG key) throws UnknownHostException {
        return newIXFR(zone, serial, fallback, host, 0, key);
    }

    public Name getName() {
        return this.zname;
    }

    public int getType() {
        return this.qtype;
    }

    public void setTimeout(int secs) {
        if (secs >= 0) {
            this.timeout = ((long) secs) * 1000;
            return;
        }
        throw new IllegalArgumentException("timeout cannot be negative");
    }

    public void setDClass(int dclass2) {
        DClass.check(dclass2);
        this.dclass = dclass2;
    }

    public void setLocalAddress(SocketAddress addr) {
        this.localAddress = addr;
    }

    private void openConnection() throws IOException {
        this.client = new TCPClient(System.currentTimeMillis() + this.timeout);
        if (this.localAddress != null) {
            this.client.bind(this.localAddress);
        }
        this.client.connect(this.address);
    }

    private void sendQuery() throws IOException {
        Record question = Record.newRecord(this.zname, this.qtype, this.dclass);
        Message query = new Message();
        query.getHeader().setOpcode(0);
        query.addRecord(question, 0);
        if (this.qtype == 251) {
            query.addRecord(new SOARecord(this.zname, this.dclass, 0, Name.root, Name.root, this.ixfr_serial, 0, 0, 0, 0), 2);
        }
        if (this.tsig != null) {
            this.tsig.apply(query, (TSIGRecord) null);
            this.verifier = new TSIG.StreamVerifier(this.tsig, query.getTSIG());
        }
        this.client.send(query.toWire(65535));
    }

    private long getSOASerial(Record rec) {
        return ((SOARecord) rec).getSerial();
    }

    private void logxfr(String s) {
        if (Options.check("verbose")) {
            PrintStream printStream = System.out;
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append(this.zname);
            stringBuffer.append(": ");
            stringBuffer.append(s);
            printStream.println(stringBuffer.toString());
        }
    }

    private void fail(String s) throws ZoneTransferException {
        throw new ZoneTransferException(s);
    }

    private void fallback() throws ZoneTransferException {
        if (!this.want_fallback) {
            fail("server doesn't support IXFR");
        }
        logxfr("falling back to AXFR");
        this.qtype = 252;
        this.state = 0;
    }

    private void parseRR(Record rec) throws ZoneTransferException {
        int type = rec.getType();
        switch (this.state) {
            case 0:
                if (type != 6) {
                    fail("missing initial SOA");
                }
                this.initialsoa = rec;
                this.end_serial = getSOASerial(rec);
                if (this.qtype != 251 || Serial.compare(this.end_serial, this.ixfr_serial) > 0) {
                    this.state = 1;
                    return;
                }
                logxfr("up to date");
                this.state = 7;
                return;
            case 1:
                if (this.qtype == 251 && type == 6 && getSOASerial(rec) == this.ixfr_serial) {
                    this.rtype = 251;
                    this.ixfr = new ArrayList();
                    logxfr("got incremental response");
                    this.state = 2;
                } else {
                    this.rtype = 252;
                    this.axfr = new ArrayList();
                    this.axfr.add(this.initialsoa);
                    logxfr("got nonincremental response");
                    this.state = 6;
                }
                parseRR(rec);
                return;
            case 2:
                Delta delta = new Delta();
                this.ixfr.add(delta);
                delta.start = getSOASerial(rec);
                delta.deletes.add(rec);
                this.state = 3;
                return;
            case 3:
                if (type == 6) {
                    this.current_serial = getSOASerial(rec);
                    this.state = 4;
                    parseRR(rec);
                    return;
                }
                ((Delta) this.ixfr.get(this.ixfr.size() - 1)).deletes.add(rec);
                return;
            case 4:
                Delta delta2 = (Delta) this.ixfr.get(this.ixfr.size() - 1);
                delta2.end = getSOASerial(rec);
                delta2.adds.add(rec);
                this.state = 5;
                return;
            case 5:
                if (type == 6) {
                    long soa_serial = getSOASerial(rec);
                    if (soa_serial == this.end_serial) {
                        this.state = 7;
                        return;
                    } else if (soa_serial != this.current_serial) {
                        StringBuffer stringBuffer = new StringBuffer();
                        stringBuffer.append("IXFR out of sync: expected serial ");
                        stringBuffer.append(this.current_serial);
                        stringBuffer.append(" , got ");
                        stringBuffer.append(soa_serial);
                        fail(stringBuffer.toString());
                    } else {
                        this.state = 2;
                        parseRR(rec);
                        return;
                    }
                }
                ((Delta) this.ixfr.get(this.ixfr.size() - 1)).adds.add(rec);
                return;
            case 6:
                if (type != 1 || rec.getDClass() == this.dclass) {
                    this.axfr.add(rec);
                    if (type == 6) {
                        this.state = 7;
                        return;
                    }
                    return;
                }
                return;
            case 7:
                fail("extra data");
                return;
            default:
                fail("invalid state");
                return;
        }
    }

    private void closeConnection() {
        try {
            if (this.client != null) {
                this.client.cleanup();
            }
        } catch (IOException e) {
        }
    }

    private Message parseMessage(byte[] b) throws WireParseException {
        try {
            return new Message(b);
        } catch (IOException e) {
            if (e instanceof WireParseException) {
                throw ((WireParseException) e);
            }
            throw new WireParseException("Error parsing message");
        }
    }

    private void doxfr() throws IOException, ZoneTransferException {
        sendQuery();
        while (this.state != 7) {
            byte[] in = this.client.recv();
            Message response = parseMessage(in);
            if (response.getHeader().getRcode() == 0 && this.verifier != null) {
                TSIGRecord tsig2 = response.getTSIG();
                if (this.verifier.verify(response, in) != 0) {
                    fail("TSIG failure");
                }
            }
            Record[] answers = response.getSectionArray(1);
            if (this.state == 0) {
                int rcode = response.getRcode();
                if (rcode != 0) {
                    if (this.qtype == 251 && rcode == 4) {
                        fallback();
                        doxfr();
                        return;
                    }
                    fail(Rcode.string(rcode));
                }
                Record question = response.getQuestion();
                if (!(question == null || question.getType() == this.qtype)) {
                    fail("invalid question section");
                }
                if (answers.length == 0 && this.qtype == 251) {
                    fallback();
                    doxfr();
                    return;
                }
            }
            for (Record parseRR : answers) {
                parseRR(parseRR);
            }
            if (this.state == 7 && this.verifier != null && !response.isVerified()) {
                fail("last message must be signed");
            }
        }
    }

    /* JADX INFO: finally extract failed */
    public List run() throws IOException, ZoneTransferException {
        try {
            openConnection();
            doxfr();
            closeConnection();
            if (this.axfr != null) {
                return this.axfr;
            }
            return this.ixfr;
        } catch (Throwable th) {
            closeConnection();
            throw th;
        }
    }

    public boolean isAXFR() {
        return this.rtype == 252;
    }

    public List getAXFR() {
        return this.axfr;
    }

    public boolean isIXFR() {
        return this.rtype == 251;
    }

    public List getIXFR() {
        return this.ixfr;
    }

    public boolean isCurrent() {
        return this.axfr == null && this.ixfr == null;
    }
}
