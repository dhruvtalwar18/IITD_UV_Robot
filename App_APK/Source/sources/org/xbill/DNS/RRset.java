package org.xbill.DNS;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;

public class RRset implements Serializable {
    private static final long serialVersionUID = -3270249290171239695L;
    private short nsigs;
    private short position;
    private List rrs;

    public RRset() {
        this.rrs = new ArrayList(1);
        this.nsigs = 0;
        this.position = 0;
    }

    public RRset(Record record) {
        this();
        safeAddRR(record);
    }

    public RRset(RRset rrset) {
        synchronized (rrset) {
            this.rrs = (List) ((ArrayList) rrset.rrs).clone();
            this.nsigs = rrset.nsigs;
            this.position = rrset.position;
        }
    }

    private void safeAddRR(Record r) {
        if (r instanceof RRSIGRecord) {
            this.rrs.add(r);
            this.nsigs = (short) (this.nsigs + 1);
        } else if (this.nsigs == 0) {
            this.rrs.add(r);
        } else {
            this.rrs.add(this.rrs.size() - this.nsigs, r);
        }
    }

    /* JADX WARNING: Code restructure failed: missing block: B:23:0x006e, code lost:
        return;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public synchronized void addRR(org.xbill.DNS.Record r7) {
        /*
            r6 = this;
            monitor-enter(r6)
            java.util.List r0 = r6.rrs     // Catch:{ all -> 0x0077 }
            int r0 = r0.size()     // Catch:{ all -> 0x0077 }
            if (r0 != 0) goto L_0x000e
            r6.safeAddRR(r7)     // Catch:{ all -> 0x0077 }
            monitor-exit(r6)
            return
        L_0x000e:
            org.xbill.DNS.Record r0 = r6.first()     // Catch:{ all -> 0x0077 }
            boolean r1 = r7.sameRRset(r0)     // Catch:{ all -> 0x0077 }
            if (r1 == 0) goto L_0x006f
            long r1 = r7.getTTL()     // Catch:{ all -> 0x0077 }
            long r3 = r0.getTTL()     // Catch:{ all -> 0x0077 }
            int r5 = (r1 > r3 ? 1 : (r1 == r3 ? 0 : -1))
            if (r5 == 0) goto L_0x0062
            long r1 = r7.getTTL()     // Catch:{ all -> 0x0077 }
            long r3 = r0.getTTL()     // Catch:{ all -> 0x0077 }
            int r5 = (r1 > r3 ? 1 : (r1 == r3 ? 0 : -1))
            if (r5 <= 0) goto L_0x003d
            org.xbill.DNS.Record r1 = r7.cloneRecord()     // Catch:{ all -> 0x0077 }
            r7 = r1
            long r1 = r0.getTTL()     // Catch:{ all -> 0x0077 }
            r7.setTTL(r1)     // Catch:{ all -> 0x0077 }
            goto L_0x0062
        L_0x003d:
            r1 = 0
        L_0x003e:
            java.util.List r2 = r6.rrs     // Catch:{ all -> 0x0077 }
            int r2 = r2.size()     // Catch:{ all -> 0x0077 }
            if (r1 >= r2) goto L_0x0062
            java.util.List r2 = r6.rrs     // Catch:{ all -> 0x0077 }
            java.lang.Object r2 = r2.get(r1)     // Catch:{ all -> 0x0077 }
            org.xbill.DNS.Record r2 = (org.xbill.DNS.Record) r2     // Catch:{ all -> 0x0077 }
            org.xbill.DNS.Record r3 = r2.cloneRecord()     // Catch:{ all -> 0x0077 }
            r2 = r3
            long r3 = r7.getTTL()     // Catch:{ all -> 0x0077 }
            r2.setTTL(r3)     // Catch:{ all -> 0x0077 }
            java.util.List r3 = r6.rrs     // Catch:{ all -> 0x0077 }
            r3.set(r1, r2)     // Catch:{ all -> 0x0077 }
            int r1 = r1 + 1
            goto L_0x003e
        L_0x0062:
            java.util.List r1 = r6.rrs     // Catch:{ all -> 0x0077 }
            boolean r1 = r1.contains(r7)     // Catch:{ all -> 0x0077 }
            if (r1 != 0) goto L_0x006d
            r6.safeAddRR(r7)     // Catch:{ all -> 0x0077 }
        L_0x006d:
            monitor-exit(r6)
            return
        L_0x006f:
            java.lang.IllegalArgumentException r1 = new java.lang.IllegalArgumentException     // Catch:{ all -> 0x0077 }
            java.lang.String r2 = "record does not match rrset"
            r1.<init>(r2)     // Catch:{ all -> 0x0077 }
            throw r1     // Catch:{ all -> 0x0077 }
        L_0x0077:
            r7 = move-exception
            monitor-exit(r6)
            throw r7
        */
        throw new UnsupportedOperationException("Method not decompiled: org.xbill.DNS.RRset.addRR(org.xbill.DNS.Record):void");
    }

    public synchronized void deleteRR(Record r) {
        if (this.rrs.remove(r) && (r instanceof RRSIGRecord)) {
            this.nsigs = (short) (this.nsigs - 1);
        }
    }

    public synchronized void clear() {
        this.rrs.clear();
        this.position = 0;
        this.nsigs = 0;
    }

    private synchronized Iterator iterator(boolean data, boolean cycle) {
        int size;
        int start;
        int total = this.rrs.size();
        if (data) {
            size = total - this.nsigs;
        } else {
            size = this.nsigs;
        }
        if (size == 0) {
            return Collections.EMPTY_LIST.iterator();
        }
        if (!data) {
            start = total - this.nsigs;
        } else if (!cycle) {
            start = 0;
        } else {
            if (this.position >= size) {
                this.position = 0;
            }
            start = this.position;
            this.position = (short) (start + 1);
        }
        List list = new ArrayList(size);
        if (data) {
            list.addAll(this.rrs.subList(start, size));
            if (start != 0) {
                list.addAll(this.rrs.subList(0, start));
            }
        } else {
            list.addAll(this.rrs.subList(start, total));
        }
        return list.iterator();
    }

    public synchronized Iterator rrs(boolean cycle) {
        return iterator(true, cycle);
    }

    public synchronized Iterator rrs() {
        return iterator(true, true);
    }

    public synchronized Iterator sigs() {
        return iterator(false, false);
    }

    public synchronized int size() {
        return this.rrs.size() - this.nsigs;
    }

    public Name getName() {
        return first().getName();
    }

    public int getType() {
        return first().getRRsetType();
    }

    public int getDClass() {
        return first().getDClass();
    }

    public synchronized long getTTL() {
        return first().getTTL();
    }

    public synchronized Record first() {
        if (this.rrs.size() != 0) {
        } else {
            throw new IllegalStateException("rrset is empty");
        }
        return (Record) this.rrs.get(0);
    }

    private String iteratorToString(Iterator it) {
        StringBuffer sb = new StringBuffer();
        while (it.hasNext()) {
            sb.append("[");
            sb.append(((Record) it.next()).rdataToString());
            sb.append("]");
            if (it.hasNext()) {
                sb.append(" ");
            }
        }
        return sb.toString();
    }

    public String toString() {
        if (this.rrs == null) {
            return "{empty}";
        }
        StringBuffer sb = new StringBuffer();
        sb.append("{ ");
        StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append(getName());
        stringBuffer.append(" ");
        sb.append(stringBuffer.toString());
        StringBuffer stringBuffer2 = new StringBuffer();
        stringBuffer2.append(getTTL());
        stringBuffer2.append(" ");
        sb.append(stringBuffer2.toString());
        StringBuffer stringBuffer3 = new StringBuffer();
        stringBuffer3.append(DClass.string(getDClass()));
        stringBuffer3.append(" ");
        sb.append(stringBuffer3.toString());
        StringBuffer stringBuffer4 = new StringBuffer();
        stringBuffer4.append(Type.string(getType()));
        stringBuffer4.append(" ");
        sb.append(stringBuffer4.toString());
        sb.append(iteratorToString(iterator(true, false)));
        if (this.nsigs > 0) {
            sb.append(" sigs: ");
            sb.append(iteratorToString(iterator(false, false)));
        }
        sb.append(" }");
        return sb.toString();
    }
}
