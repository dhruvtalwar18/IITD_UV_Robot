package org.xbill.DNS;

import com.github.rosjava.android_remocons.common_tools.master.MasterDescription;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import rocon_std_msgs.Connection;

public class Message implements Cloneable {
    public static final int MAXLENGTH = 65535;
    static final int TSIG_FAILED = 4;
    static final int TSIG_INTERMEDIATE = 2;
    static final int TSIG_SIGNED = 3;
    static final int TSIG_UNSIGNED = 0;
    static final int TSIG_VERIFIED = 1;
    private static RRset[] emptyRRsetArray = new RRset[0];
    private static Record[] emptyRecordArray = new Record[0];
    private Header header;
    private TSIGRecord querytsig;
    private List[] sections;
    int sig0start;
    private int size;
    int tsigState;
    private int tsigerror;
    private TSIG tsigkey;
    int tsigstart;

    private Message(Header header2) {
        this.sections = new List[4];
        this.header = header2;
    }

    public Message(int id) {
        this(new Header(id));
    }

    public Message() {
        this(new Header());
    }

    public static Message newQuery(Record r) {
        Message m = new Message();
        m.header.setOpcode(0);
        m.header.setFlag(7);
        m.addRecord(r, 0);
        return m;
    }

    public static Message newUpdate(Name zone) {
        return new Update(zone);
    }

    Message(DNSInput in) throws IOException {
        this(new Header(in));
        boolean isUpdate = this.header.getOpcode() == 5;
        boolean truncated = this.header.getFlag(6);
        int i = 0;
        while (i < 4) {
            try {
                int count = this.header.getCount(i);
                if (count > 0) {
                    this.sections[i] = new ArrayList(count);
                }
                for (int j = 0; j < count; j++) {
                    int pos = in.current();
                    Record rec = Record.fromWire(in, i, isUpdate);
                    this.sections[i].add(rec);
                    if (rec.getType() == 250) {
                        this.tsigstart = pos;
                    }
                    if (rec.getType() == 24 && ((SIGRecord) rec).getTypeCovered() == 0) {
                        this.sig0start = pos;
                    }
                }
                i++;
            } catch (WireParseException e) {
                if (!truncated) {
                    throw e;
                }
            }
        }
        this.size = in.current();
    }

    public Message(byte[] b) throws IOException {
        this(new DNSInput(b));
    }

    public void setHeader(Header h) {
        this.header = h;
    }

    public Header getHeader() {
        return this.header;
    }

    public void addRecord(Record r, int section) {
        if (this.sections[section] == null) {
            this.sections[section] = new LinkedList();
        }
        this.header.incCount(section);
        this.sections[section].add(r);
    }

    public boolean removeRecord(Record r, int section) {
        if (this.sections[section] == null || !this.sections[section].remove(r)) {
            return false;
        }
        this.header.decCount(section);
        return true;
    }

    public void removeAllRecords(int section) {
        this.sections[section] = null;
        this.header.setCount(section, 0);
    }

    public boolean findRecord(Record r, int section) {
        return this.sections[section] != null && this.sections[section].contains(r);
    }

    public boolean findRecord(Record r) {
        for (int i = 1; i <= 3; i++) {
            if (this.sections[i] != null && this.sections[i].contains(r)) {
                return true;
            }
        }
        return false;
    }

    public boolean findRRset(Name name, int type, int section) {
        if (this.sections[section] == null) {
            return false;
        }
        for (int i = 0; i < this.sections[section].size(); i++) {
            Record r = (Record) this.sections[section].get(i);
            if (r.getType() == type && name.equals(r.getName())) {
                return true;
            }
        }
        return false;
    }

    public boolean findRRset(Name name, int type) {
        return findRRset(name, type, 1) || findRRset(name, type, 2) || findRRset(name, type, 3);
    }

    public Record getQuestion() {
        List l = this.sections[0];
        if (l == null || l.size() == 0) {
            return null;
        }
        return (Record) l.get(0);
    }

    public TSIGRecord getTSIG() {
        int count = this.header.getCount(3);
        if (count == 0) {
            return null;
        }
        Record rec = (Record) this.sections[3].get(count - 1);
        if (rec.type != 250) {
            return null;
        }
        return (TSIGRecord) rec;
    }

    public boolean isSigned() {
        return this.tsigState == 3 || this.tsigState == 1 || this.tsigState == 4;
    }

    public boolean isVerified() {
        return this.tsigState == 1;
    }

    public OPTRecord getOPT() {
        Record[] additional = getSectionArray(3);
        for (int i = 0; i < additional.length; i++) {
            if (additional[i] instanceof OPTRecord) {
                return (OPTRecord) additional[i];
            }
        }
        return null;
    }

    public int getRcode() {
        int rcode = this.header.getRcode();
        OPTRecord opt = getOPT();
        if (opt != null) {
            return rcode + (opt.getExtendedRcode() << 4);
        }
        return rcode;
    }

    public Record[] getSectionArray(int section) {
        if (this.sections[section] == null) {
            return emptyRecordArray;
        }
        List l = this.sections[section];
        return (Record[]) l.toArray(new Record[l.size()]);
    }

    private static boolean sameSet(Record r1, Record r2) {
        return r1.getRRsetType() == r2.getRRsetType() && r1.getDClass() == r2.getDClass() && r1.getName().equals(r2.getName());
    }

    public RRset[] getSectionRRsets(int section) {
        if (this.sections[section] == null) {
            return emptyRRsetArray;
        }
        List sets = new LinkedList();
        Record[] recs = getSectionArray(section);
        Set hash = new HashSet();
        for (int i = 0; i < recs.length; i++) {
            Name name = recs[i].getName();
            boolean newset = true;
            if (hash.contains(name)) {
                int j = sets.size() - 1;
                while (true) {
                    if (j < 0) {
                        break;
                    }
                    RRset set = (RRset) sets.get(j);
                    if (set.getType() == recs[i].getRRsetType() && set.getDClass() == recs[i].getDClass() && set.getName().equals(name)) {
                        set.addRR(recs[i]);
                        newset = false;
                        break;
                    }
                    j--;
                }
            }
            if (newset) {
                sets.add(new RRset(recs[i]));
                hash.add(name);
            }
        }
        return (RRset[]) sets.toArray(new RRset[sets.size()]);
    }

    /* access modifiers changed from: package-private */
    public void toWire(DNSOutput out) {
        this.header.toWire(out);
        Compression c = new Compression();
        for (int i = 0; i < 4; i++) {
            if (this.sections[i] != null) {
                for (int j = 0; j < this.sections[i].size(); j++) {
                    ((Record) this.sections[i].get(j)).toWire(out, i, c);
                }
            }
        }
    }

    private int sectionToWire(DNSOutput out, int section, Compression c, int maxLength) {
        int n = this.sections[section].size();
        Record lastrec = null;
        int rendered = 0;
        int pos = out.current();
        for (int i = 0; i < n; i++) {
            Record rec = (Record) this.sections[section].get(i);
            if (lastrec != null && !sameSet(rec, lastrec)) {
                pos = out.current();
                rendered = i;
            }
            lastrec = rec;
            rec.toWire(out, section, c);
            if (out.current() > maxLength) {
                out.jump(pos);
                return n - rendered;
            }
        }
        return 0;
    }

    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r5v4, resolved type: java.lang.Object} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r1v4, resolved type: org.xbill.DNS.Header} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r9v3, resolved type: java.lang.Object} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r1v7, resolved type: org.xbill.DNS.Header} */
    /* JADX WARNING: Multi-variable type inference failed */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    private boolean toWire(org.xbill.DNS.DNSOutput r12, int r13) {
        /*
            r11 = this;
            r0 = 0
            r1 = 12
            if (r13 >= r1) goto L_0x0006
            return r0
        L_0x0006:
            r1 = 0
            r2 = r13
            org.xbill.DNS.TSIG r3 = r11.tsigkey
            if (r3 == 0) goto L_0x0013
            org.xbill.DNS.TSIG r3 = r11.tsigkey
            int r3 = r3.recordLength()
            int r2 = r2 - r3
        L_0x0013:
            int r3 = r12.current()
            org.xbill.DNS.Header r4 = r11.header
            r4.toWire(r12)
            org.xbill.DNS.Compression r4 = new org.xbill.DNS.Compression
            r4.<init>()
            r5 = 0
        L_0x0022:
            r6 = 4
            r7 = 3
            if (r5 >= r6) goto L_0x0067
            java.util.List[] r8 = r11.sections
            r8 = r8[r5]
            if (r8 != 0) goto L_0x002d
            goto L_0x0064
        L_0x002d:
            int r8 = r11.sectionToWire(r12, r5, r4, r2)
            if (r8 == 0) goto L_0x0064
            if (r1 != 0) goto L_0x003e
            org.xbill.DNS.Header r9 = r11.header
            java.lang.Object r9 = r9.clone()
            r1 = r9
            org.xbill.DNS.Header r1 = (org.xbill.DNS.Header) r1
        L_0x003e:
            if (r5 == r7) goto L_0x0044
            r9 = 6
            r1.setFlag(r9)
        L_0x0044:
            int r9 = r1.getCount(r5)
            int r10 = r9 - r8
            r1.setCount(r5, r10)
            int r10 = r5 + 1
        L_0x004f:
            if (r10 >= r6) goto L_0x0057
            r1.setCount(r10, r0)
            int r10 = r10 + 1
            goto L_0x004f
        L_0x0057:
            r12.save()
            r12.jump(r3)
            r1.toWire(r12)
            r12.restore()
            goto L_0x0067
        L_0x0064:
            int r5 = r5 + 1
            goto L_0x0022
        L_0x0067:
            org.xbill.DNS.TSIG r0 = r11.tsigkey
            if (r0 == 0) goto L_0x0096
            org.xbill.DNS.TSIG r0 = r11.tsigkey
            byte[] r5 = r12.toByteArray()
            int r6 = r11.tsigerror
            org.xbill.DNS.TSIGRecord r8 = r11.querytsig
            org.xbill.DNS.TSIGRecord r0 = r0.generate(r11, r5, r6, r8)
            if (r1 != 0) goto L_0x0084
            org.xbill.DNS.Header r5 = r11.header
            java.lang.Object r5 = r5.clone()
            r1 = r5
            org.xbill.DNS.Header r1 = (org.xbill.DNS.Header) r1
        L_0x0084:
            r0.toWire(r12, r7, r4)
            r1.incCount(r7)
            r12.save()
            r12.jump(r3)
            r1.toWire(r12)
            r12.restore()
        L_0x0096:
            r0 = 1
            return r0
        */
        throw new UnsupportedOperationException("Method not decompiled: org.xbill.DNS.Message.toWire(org.xbill.DNS.DNSOutput, int):boolean");
    }

    public byte[] toWire() {
        DNSOutput out = new DNSOutput();
        toWire(out);
        this.size = out.current();
        return out.toByteArray();
    }

    public byte[] toWire(int maxLength) {
        DNSOutput out = new DNSOutput();
        toWire(out, maxLength);
        this.size = out.current();
        return out.toByteArray();
    }

    public void setTSIG(TSIG key, int error, TSIGRecord querytsig2) {
        this.tsigkey = key;
        this.tsigerror = error;
        this.querytsig = querytsig2;
    }

    public int numBytes() {
        return this.size;
    }

    public String sectionToString(int i) {
        if (i > 3) {
            return null;
        }
        StringBuffer sb = new StringBuffer();
        Record[] records = getSectionArray(i);
        for (Record rec : records) {
            if (i == 0) {
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append(";;\t");
                stringBuffer.append(rec.name);
                sb.append(stringBuffer.toString());
                StringBuffer stringBuffer2 = new StringBuffer();
                stringBuffer2.append(", type = ");
                stringBuffer2.append(Type.string(rec.type));
                sb.append(stringBuffer2.toString());
                StringBuffer stringBuffer3 = new StringBuffer();
                stringBuffer3.append(", class = ");
                stringBuffer3.append(DClass.string(rec.dclass));
                sb.append(stringBuffer3.toString());
            } else {
                sb.append(rec);
            }
            sb.append("\n");
        }
        return sb.toString();
    }

    public String toString() {
        StringBuffer sb = new StringBuffer();
        if (getOPT() != null) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append(this.header.toStringWithRcode(getRcode()));
            stringBuffer.append("\n");
            sb.append(stringBuffer.toString());
        } else {
            StringBuffer stringBuffer2 = new StringBuffer();
            stringBuffer2.append(this.header);
            stringBuffer2.append("\n");
            sb.append(stringBuffer2.toString());
        }
        if (isSigned()) {
            sb.append(";; TSIG ");
            if (isVerified()) {
                sb.append(MasterDescription.OK);
            } else {
                sb.append(Connection.INVALID);
            }
            sb.append(10);
        }
        for (int i = 0; i < 4; i++) {
            if (this.header.getOpcode() != 5) {
                StringBuffer stringBuffer3 = new StringBuffer();
                stringBuffer3.append(";; ");
                stringBuffer3.append(Section.longString(i));
                stringBuffer3.append(":\n");
                sb.append(stringBuffer3.toString());
            } else {
                StringBuffer stringBuffer4 = new StringBuffer();
                stringBuffer4.append(";; ");
                stringBuffer4.append(Section.updString(i));
                stringBuffer4.append(":\n");
                sb.append(stringBuffer4.toString());
            }
            StringBuffer stringBuffer5 = new StringBuffer();
            stringBuffer5.append(sectionToString(i));
            stringBuffer5.append("\n");
            sb.append(stringBuffer5.toString());
        }
        StringBuffer stringBuffer6 = new StringBuffer();
        stringBuffer6.append(";; Message size: ");
        stringBuffer6.append(numBytes());
        stringBuffer6.append(" bytes");
        sb.append(stringBuffer6.toString());
        return sb.toString();
    }

    public Object clone() {
        Message m = new Message();
        for (int i = 0; i < this.sections.length; i++) {
            if (this.sections[i] != null) {
                m.sections[i] = new LinkedList(this.sections[i]);
            }
        }
        m.header = (Header) this.header.clone();
        m.size = this.size;
        return m;
    }
}
