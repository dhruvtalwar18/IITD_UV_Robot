package org.xbill.DNS;

import java.io.IOException;
import java.io.Serializable;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.TreeMap;

public class Zone implements Serializable {
    public static final int PRIMARY = 1;
    public static final int SECONDARY = 2;
    private static final long serialVersionUID = -9220510891189510942L;
    private RRset NS;
    private SOARecord SOA;
    /* access modifiers changed from: private */
    public Map data;
    private int dclass;
    private boolean hasWild;
    /* access modifiers changed from: private */
    public Name origin;
    /* access modifiers changed from: private */
    public Object originNode;

    class ZoneIterator implements Iterator {
        private int count;
        private RRset[] current;
        private boolean wantLastSOA;
        private Iterator zentries;

        ZoneIterator(boolean axfr) {
            synchronized (Zone.this) {
                this.zentries = Zone.this.data.entrySet().iterator();
            }
            this.wantLastSOA = axfr;
            RRset[] sets = Zone.this.allRRsets(Zone.this.originNode);
            this.current = new RRset[sets.length];
            int j = 2;
            for (int i = 0; i < sets.length; i++) {
                int type = sets[i].getType();
                if (type == 6) {
                    this.current[0] = sets[i];
                } else if (type == 2) {
                    this.current[1] = sets[i];
                } else {
                    this.current[j] = sets[i];
                    j++;
                }
            }
        }

        public boolean hasNext() {
            return this.current != null || this.wantLastSOA;
        }

        public Object next() {
            if (!hasNext()) {
                throw new NoSuchElementException();
            } else if (this.current == null) {
                this.wantLastSOA = false;
                return Zone.this.oneRRset(Zone.this.originNode, 6);
            } else {
                Object[] objArr = this.current;
                int i = this.count;
                this.count = i + 1;
                Object set = objArr[i];
                if (this.count == this.current.length) {
                    this.current = null;
                    while (true) {
                        if (!this.zentries.hasNext()) {
                            break;
                        }
                        Map.Entry entry = (Map.Entry) this.zentries.next();
                        if (!entry.getKey().equals(Zone.this.origin)) {
                            RRset[] sets = Zone.this.allRRsets(entry.getValue());
                            if (sets.length != 0) {
                                this.current = sets;
                                this.count = 0;
                                break;
                            }
                        }
                    }
                }
                return set;
            }
        }

        public void remove() {
            throw new UnsupportedOperationException();
        }
    }

    private void validate() throws IOException {
        this.originNode = exactName(this.origin);
        if (this.originNode != null) {
            RRset rrset = oneRRset(this.originNode, 6);
            if (rrset == null || rrset.size() != 1) {
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append(this.origin);
                stringBuffer.append(": exactly 1 SOA must be specified");
                throw new IOException(stringBuffer.toString());
            }
            this.SOA = (SOARecord) rrset.rrs().next();
            this.NS = oneRRset(this.originNode, 2);
            if (this.NS == null) {
                StringBuffer stringBuffer2 = new StringBuffer();
                stringBuffer2.append(this.origin);
                stringBuffer2.append(": no NS set specified");
                throw new IOException(stringBuffer2.toString());
            }
            return;
        }
        StringBuffer stringBuffer3 = new StringBuffer();
        stringBuffer3.append(this.origin);
        stringBuffer3.append(": no data specified");
        throw new IOException(stringBuffer3.toString());
    }

    private final void maybeAddRecord(Record record) throws IOException {
        int rtype = record.getType();
        Name name = record.getName();
        if (rtype == 6 && !name.equals(this.origin)) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("SOA owner ");
            stringBuffer.append(name);
            stringBuffer.append(" does not match zone origin ");
            stringBuffer.append(this.origin);
            throw new IOException(stringBuffer.toString());
        } else if (name.subdomain(this.origin)) {
            addRecord(record);
        }
    }

    public Zone(Name zone, String file) throws IOException {
        this.dclass = 1;
        this.data = new TreeMap();
        if (zone != null) {
            Master m = new Master(file, zone);
            this.origin = zone;
            while (true) {
                Record nextRecord = m.nextRecord();
                Record record = nextRecord;
                if (nextRecord != null) {
                    maybeAddRecord(record);
                } else {
                    validate();
                    return;
                }
            }
        } else {
            throw new IllegalArgumentException("no zone name specified");
        }
    }

    public Zone(Name zone, Record[] records) throws IOException {
        this.dclass = 1;
        this.data = new TreeMap();
        if (zone != null) {
            this.origin = zone;
            for (Record maybeAddRecord : records) {
                maybeAddRecord(maybeAddRecord);
            }
            validate();
            return;
        }
        throw new IllegalArgumentException("no zone name specified");
    }

    private void fromXFR(ZoneTransferIn xfrin) throws IOException, ZoneTransferException {
        this.data = new TreeMap();
        this.origin = xfrin.getName();
        for (Record record : xfrin.run()) {
            maybeAddRecord(record);
        }
        if (xfrin.isAXFR()) {
            validate();
            return;
        }
        throw new IllegalArgumentException("zones can only be created from AXFRs");
    }

    public Zone(ZoneTransferIn xfrin) throws IOException, ZoneTransferException {
        this.dclass = 1;
        fromXFR(xfrin);
    }

    public Zone(Name zone, int dclass2, String remote) throws IOException, ZoneTransferException {
        this.dclass = 1;
        ZoneTransferIn xfrin = ZoneTransferIn.newAXFR(zone, remote, (TSIG) null);
        xfrin.setDClass(dclass2);
        fromXFR(xfrin);
    }

    public Name getOrigin() {
        return this.origin;
    }

    public RRset getNS() {
        return this.NS;
    }

    public SOARecord getSOA() {
        return this.SOA;
    }

    public int getDClass() {
        return this.dclass;
    }

    private synchronized Object exactName(Name name) {
        return this.data.get(name);
    }

    /* access modifiers changed from: private */
    public synchronized RRset[] allRRsets(Object types) {
        if (types instanceof List) {
            List typelist = (List) types;
            return (RRset[]) typelist.toArray(new RRset[typelist.size()]);
        }
        return new RRset[]{(RRset) types};
    }

    /* access modifiers changed from: private */
    public synchronized RRset oneRRset(Object types, int type) {
        if (type != 255) {
            if (types instanceof List) {
                List list = (List) types;
                for (int i = 0; i < list.size(); i++) {
                    RRset set = (RRset) list.get(i);
                    if (set.getType() == type) {
                        return set;
                    }
                }
            } else {
                RRset set2 = (RRset) types;
                if (set2.getType() == type) {
                    return set2;
                }
            }
            return null;
        }
        throw new IllegalArgumentException("oneRRset(ANY)");
    }

    private synchronized RRset findRRset(Name name, int type) {
        Object types = exactName(name);
        if (types == null) {
            return null;
        }
        return oneRRset(types, type);
    }

    /* JADX WARNING: Code restructure failed: missing block: B:31:0x0067, code lost:
        return;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    private synchronized void addRRset(org.xbill.DNS.Name r7, org.xbill.DNS.RRset r8) {
        /*
            r6 = this;
            monitor-enter(r6)
            boolean r0 = r6.hasWild     // Catch:{ all -> 0x0068 }
            if (r0 != 0) goto L_0x000e
            boolean r0 = r7.isWild()     // Catch:{ all -> 0x0068 }
            if (r0 == 0) goto L_0x000e
            r0 = 1
            r6.hasWild = r0     // Catch:{ all -> 0x0068 }
        L_0x000e:
            java.util.Map r0 = r6.data     // Catch:{ all -> 0x0068 }
            java.lang.Object r0 = r0.get(r7)     // Catch:{ all -> 0x0068 }
            if (r0 != 0) goto L_0x001d
            java.util.Map r1 = r6.data     // Catch:{ all -> 0x0068 }
            r1.put(r7, r8)     // Catch:{ all -> 0x0068 }
            monitor-exit(r6)
            return
        L_0x001d:
            int r1 = r8.getType()     // Catch:{ all -> 0x0068 }
            boolean r2 = r0 instanceof java.util.List     // Catch:{ all -> 0x0068 }
            if (r2 == 0) goto L_0x0047
            r2 = r0
            java.util.List r2 = (java.util.List) r2     // Catch:{ all -> 0x0068 }
            r3 = 0
        L_0x0029:
            int r4 = r2.size()     // Catch:{ all -> 0x0068 }
            if (r3 >= r4) goto L_0x0043
            java.lang.Object r4 = r2.get(r3)     // Catch:{ all -> 0x0068 }
            org.xbill.DNS.RRset r4 = (org.xbill.DNS.RRset) r4     // Catch:{ all -> 0x0068 }
            int r5 = r4.getType()     // Catch:{ all -> 0x0068 }
            if (r5 != r1) goto L_0x0040
            r2.set(r3, r8)     // Catch:{ all -> 0x0068 }
            monitor-exit(r6)
            return
        L_0x0040:
            int r3 = r3 + 1
            goto L_0x0029
        L_0x0043:
            r2.add(r8)     // Catch:{ all -> 0x0068 }
            goto L_0x0066
        L_0x0047:
            r2 = r0
            org.xbill.DNS.RRset r2 = (org.xbill.DNS.RRset) r2     // Catch:{ all -> 0x0068 }
            int r3 = r2.getType()     // Catch:{ all -> 0x0068 }
            if (r3 != r1) goto L_0x0056
            java.util.Map r3 = r6.data     // Catch:{ all -> 0x0068 }
            r3.put(r7, r8)     // Catch:{ all -> 0x0068 }
            goto L_0x0066
        L_0x0056:
            java.util.LinkedList r3 = new java.util.LinkedList     // Catch:{ all -> 0x0068 }
            r3.<init>()     // Catch:{ all -> 0x0068 }
            r3.add(r2)     // Catch:{ all -> 0x0068 }
            r3.add(r8)     // Catch:{ all -> 0x0068 }
            java.util.Map r4 = r6.data     // Catch:{ all -> 0x0068 }
            r4.put(r7, r3)     // Catch:{ all -> 0x0068 }
        L_0x0066:
            monitor-exit(r6)
            return
        L_0x0068:
            r7 = move-exception
            monitor-exit(r6)
            throw r7
        */
        throw new UnsupportedOperationException("Method not decompiled: org.xbill.DNS.Zone.addRRset(org.xbill.DNS.Name, org.xbill.DNS.RRset):void");
    }

    /* JADX WARNING: Code restructure failed: missing block: B:18:0x0034, code lost:
        return;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:29:0x004a, code lost:
        return;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    private synchronized void removeRRset(org.xbill.DNS.Name r6, int r7) {
        /*
            r5 = this;
            monitor-enter(r5)
            java.util.Map r0 = r5.data     // Catch:{ all -> 0x004b }
            java.lang.Object r0 = r0.get(r6)     // Catch:{ all -> 0x004b }
            if (r0 != 0) goto L_0x000b
            monitor-exit(r5)
            return
        L_0x000b:
            boolean r1 = r0 instanceof java.util.List     // Catch:{ all -> 0x004b }
            if (r1 == 0) goto L_0x0039
            r1 = r0
            java.util.List r1 = (java.util.List) r1     // Catch:{ all -> 0x004b }
            r2 = 0
        L_0x0013:
            int r3 = r1.size()     // Catch:{ all -> 0x004b }
            if (r2 >= r3) goto L_0x0038
            java.lang.Object r3 = r1.get(r2)     // Catch:{ all -> 0x004b }
            org.xbill.DNS.RRset r3 = (org.xbill.DNS.RRset) r3     // Catch:{ all -> 0x004b }
            int r4 = r3.getType()     // Catch:{ all -> 0x004b }
            if (r4 != r7) goto L_0x0035
            r1.remove(r2)     // Catch:{ all -> 0x004b }
            int r4 = r1.size()     // Catch:{ all -> 0x004b }
            if (r4 != 0) goto L_0x0033
            java.util.Map r4 = r5.data     // Catch:{ all -> 0x004b }
            r4.remove(r6)     // Catch:{ all -> 0x004b }
        L_0x0033:
            monitor-exit(r5)
            return
        L_0x0035:
            int r2 = r2 + 1
            goto L_0x0013
        L_0x0038:
            goto L_0x0049
        L_0x0039:
            r1 = r0
            org.xbill.DNS.RRset r1 = (org.xbill.DNS.RRset) r1     // Catch:{ all -> 0x004b }
            int r2 = r1.getType()     // Catch:{ all -> 0x004b }
            if (r2 == r7) goto L_0x0044
            monitor-exit(r5)
            return
        L_0x0044:
            java.util.Map r2 = r5.data     // Catch:{ all -> 0x004b }
            r2.remove(r6)     // Catch:{ all -> 0x004b }
        L_0x0049:
            monitor-exit(r5)
            return
        L_0x004b:
            r6 = move-exception
            monitor-exit(r5)
            throw r6
        */
        throw new UnsupportedOperationException("Method not decompiled: org.xbill.DNS.Zone.removeRRset(org.xbill.DNS.Name, int):void");
    }

    /* JADX WARNING: Code restructure failed: missing block: B:36:?, code lost:
        r1 = new org.xbill.DNS.SetResponse(6);
        r4 = allRRsets(r9);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:38:0x0062, code lost:
        if (r5 >= r4.length) goto L_0x006c;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:39:0x0064, code lost:
        r1.addRRset(r4[r5]);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:40:0x0069, code lost:
        r5 = r5 + 1;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:42:0x006d, code lost:
        return r1;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:72:0x00ac, code lost:
        if (r12.hasWild == false) goto L_0x00d3;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:74:0x00b1, code lost:
        if (r5 >= (r0 - r2)) goto L_0x00d3;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:75:0x00b3, code lost:
        r7 = exactName(r13.wild(r5 + 1));
     */
    /* JADX WARNING: Code restructure failed: missing block: B:76:0x00bd, code lost:
        if (r7 != null) goto L_0x00c0;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:78:0x00c0, code lost:
        r8 = oneRRset(r7, r14);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:79:0x00c4, code lost:
        if (r8 == null) goto L_0x00d0;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:80:0x00c6, code lost:
        r1 = new org.xbill.DNS.SetResponse(6);
        r1.addRRset(r8);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:82:0x00cf, code lost:
        return r1;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:83:0x00d0, code lost:
        r5 = r5 + 1;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:87:0x00d8, code lost:
        return org.xbill.DNS.SetResponse.ofType(1);
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    private synchronized org.xbill.DNS.SetResponse lookup(org.xbill.DNS.Name r13, int r14) {
        /*
            r12 = this;
            monitor-enter(r12)
            org.xbill.DNS.Name r0 = r12.origin     // Catch:{ all -> 0x00d9 }
            boolean r0 = r13.subdomain(r0)     // Catch:{ all -> 0x00d9 }
            r1 = 1
            if (r0 != 0) goto L_0x0010
            org.xbill.DNS.SetResponse r0 = org.xbill.DNS.SetResponse.ofType(r1)     // Catch:{ all -> 0x00d9 }
            monitor-exit(r12)
            return r0
        L_0x0010:
            int r0 = r13.labels()     // Catch:{ all -> 0x00d9 }
            org.xbill.DNS.Name r2 = r12.origin     // Catch:{ all -> 0x00d9 }
            int r2 = r2.labels()     // Catch:{ all -> 0x00d9 }
            r3 = r2
        L_0x001b:
            r4 = 6
            r5 = 0
            if (r3 > r0) goto L_0x00aa
            if (r3 != r2) goto L_0x0023
            r6 = 1
            goto L_0x0024
        L_0x0023:
            r6 = 0
        L_0x0024:
            if (r3 != r0) goto L_0x0028
            r7 = 1
            goto L_0x0029
        L_0x0028:
            r7 = 0
        L_0x0029:
            if (r6 == 0) goto L_0x002e
            org.xbill.DNS.Name r8 = r12.origin     // Catch:{ all -> 0x00d9 }
            goto L_0x0039
        L_0x002e:
            if (r7 == 0) goto L_0x0032
            r8 = r13
            goto L_0x0039
        L_0x0032:
            org.xbill.DNS.Name r8 = new org.xbill.DNS.Name     // Catch:{ all -> 0x00d9 }
            int r9 = r0 - r3
            r8.<init>((org.xbill.DNS.Name) r13, (int) r9)     // Catch:{ all -> 0x00d9 }
        L_0x0039:
            java.lang.Object r9 = r12.exactName(r8)     // Catch:{ all -> 0x00d9 }
            if (r9 != 0) goto L_0x0040
            goto L_0x00a6
        L_0x0040:
            r10 = 2
            if (r6 != 0) goto L_0x0051
            org.xbill.DNS.RRset r11 = r12.oneRRset(r9, r10)     // Catch:{ all -> 0x00d9 }
            if (r11 == 0) goto L_0x0051
            org.xbill.DNS.SetResponse r1 = new org.xbill.DNS.SetResponse     // Catch:{ all -> 0x00d9 }
            r4 = 3
            r1.<init>(r4, r11)     // Catch:{ all -> 0x00d9 }
            monitor-exit(r12)
            return r1
        L_0x0051:
            if (r7 == 0) goto L_0x006e
            r11 = 255(0xff, float:3.57E-43)
            if (r14 != r11) goto L_0x006e
            org.xbill.DNS.SetResponse r1 = new org.xbill.DNS.SetResponse     // Catch:{ all -> 0x00d9 }
            r1.<init>(r4)     // Catch:{ all -> 0x00d9 }
            org.xbill.DNS.RRset[] r4 = r12.allRRsets(r9)     // Catch:{ all -> 0x00d9 }
        L_0x0061:
            int r10 = r4.length     // Catch:{ all -> 0x00d9 }
            if (r5 >= r10) goto L_0x006c
            r10 = r4[r5]     // Catch:{ all -> 0x00d9 }
            r1.addRRset(r10)     // Catch:{ all -> 0x00d9 }
            int r5 = r5 + 1
            goto L_0x0061
        L_0x006c:
            monitor-exit(r12)
            return r1
        L_0x006e:
            r5 = 5
            if (r7 == 0) goto L_0x008f
            org.xbill.DNS.RRset r11 = r12.oneRRset(r9, r14)     // Catch:{ all -> 0x00d9 }
            if (r11 == 0) goto L_0x0081
            org.xbill.DNS.SetResponse r1 = new org.xbill.DNS.SetResponse     // Catch:{ all -> 0x00d9 }
            r1.<init>(r4)     // Catch:{ all -> 0x00d9 }
            r1.addRRset(r11)     // Catch:{ all -> 0x00d9 }
            monitor-exit(r12)
            return r1
        L_0x0081:
            org.xbill.DNS.RRset r4 = r12.oneRRset(r9, r5)     // Catch:{ all -> 0x00d9 }
            if (r4 == 0) goto L_0x009e
            org.xbill.DNS.SetResponse r1 = new org.xbill.DNS.SetResponse     // Catch:{ all -> 0x00d9 }
            r5 = 4
            r1.<init>(r5, r4)     // Catch:{ all -> 0x00d9 }
            monitor-exit(r12)
            return r1
        L_0x008f:
            r4 = 39
            org.xbill.DNS.RRset r4 = r12.oneRRset(r9, r4)     // Catch:{ all -> 0x00d9 }
            if (r4 == 0) goto L_0x009e
            org.xbill.DNS.SetResponse r1 = new org.xbill.DNS.SetResponse     // Catch:{ all -> 0x00d9 }
            r1.<init>(r5, r4)     // Catch:{ all -> 0x00d9 }
            monitor-exit(r12)
            return r1
        L_0x009e:
            if (r7 == 0) goto L_0x00a6
            org.xbill.DNS.SetResponse r1 = org.xbill.DNS.SetResponse.ofType(r10)     // Catch:{ all -> 0x00d9 }
            monitor-exit(r12)
            return r1
        L_0x00a6:
            int r3 = r3 + 1
            goto L_0x001b
        L_0x00aa:
            boolean r6 = r12.hasWild     // Catch:{ all -> 0x00d9 }
            if (r6 == 0) goto L_0x00d3
        L_0x00af:
            int r6 = r0 - r2
            if (r5 >= r6) goto L_0x00d3
            int r6 = r5 + 1
            org.xbill.DNS.Name r6 = r13.wild(r6)     // Catch:{ all -> 0x00d9 }
            java.lang.Object r7 = r12.exactName(r6)     // Catch:{ all -> 0x00d9 }
            if (r7 != 0) goto L_0x00c0
            goto L_0x00d0
        L_0x00c0:
            org.xbill.DNS.RRset r8 = r12.oneRRset(r7, r14)     // Catch:{ all -> 0x00d9 }
            if (r8 == 0) goto L_0x00d0
            org.xbill.DNS.SetResponse r1 = new org.xbill.DNS.SetResponse     // Catch:{ all -> 0x00d9 }
            r1.<init>(r4)     // Catch:{ all -> 0x00d9 }
            r1.addRRset(r8)     // Catch:{ all -> 0x00d9 }
            monitor-exit(r12)
            return r1
        L_0x00d0:
            int r5 = r5 + 1
            goto L_0x00af
        L_0x00d3:
            org.xbill.DNS.SetResponse r1 = org.xbill.DNS.SetResponse.ofType(r1)     // Catch:{ all -> 0x00d9 }
            monitor-exit(r12)
            return r1
        L_0x00d9:
            r13 = move-exception
            monitor-exit(r12)
            throw r13
        */
        throw new UnsupportedOperationException("Method not decompiled: org.xbill.DNS.Zone.lookup(org.xbill.DNS.Name, int):org.xbill.DNS.SetResponse");
    }

    public SetResponse findRecords(Name name, int type) {
        return lookup(name, type);
    }

    public RRset findExactMatch(Name name, int type) {
        Object types = exactName(name);
        if (types == null) {
            return null;
        }
        return oneRRset(types, type);
    }

    public void addRRset(RRset rrset) {
        addRRset(rrset.getName(), rrset);
    }

    public void addRecord(Record r) {
        Name name = r.getName();
        int rtype = r.getRRsetType();
        synchronized (this) {
            RRset rrset = findRRset(name, rtype);
            if (rrset == null) {
                addRRset(name, new RRset(r));
            } else {
                rrset.addRR(r);
            }
        }
    }

    /* JADX WARNING: Code restructure failed: missing block: B:14:0x002a, code lost:
        return;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public void removeRecord(org.xbill.DNS.Record r6) {
        /*
            r5 = this;
            org.xbill.DNS.Name r0 = r6.getName()
            int r1 = r6.getRRsetType()
            monitor-enter(r5)
            org.xbill.DNS.RRset r2 = r5.findRRset(r0, r1)     // Catch:{ all -> 0x002b }
            if (r2 != 0) goto L_0x0011
            monitor-exit(r5)     // Catch:{ all -> 0x002b }
            return
        L_0x0011:
            int r3 = r2.size()     // Catch:{ all -> 0x002b }
            r4 = 1
            if (r3 != r4) goto L_0x0026
            org.xbill.DNS.Record r3 = r2.first()     // Catch:{ all -> 0x002b }
            boolean r3 = r3.equals(r6)     // Catch:{ all -> 0x002b }
            if (r3 == 0) goto L_0x0026
            r5.removeRRset(r0, r1)     // Catch:{ all -> 0x002b }
            goto L_0x0029
        L_0x0026:
            r2.deleteRR(r6)     // Catch:{ all -> 0x002b }
        L_0x0029:
            monitor-exit(r5)     // Catch:{ all -> 0x002b }
            return
        L_0x002b:
            r2 = move-exception
            monitor-exit(r5)     // Catch:{ all -> 0x002b }
            throw r2
        */
        throw new UnsupportedOperationException("Method not decompiled: org.xbill.DNS.Zone.removeRecord(org.xbill.DNS.Record):void");
    }

    public Iterator iterator() {
        return new ZoneIterator(false);
    }

    public Iterator AXFR() {
        return new ZoneIterator(true);
    }

    private void nodeToString(StringBuffer sb, Object node) {
        RRset[] sets = allRRsets(node);
        for (RRset rrset : sets) {
            Iterator it = rrset.rrs();
            while (it.hasNext()) {
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append(it.next());
                stringBuffer.append("\n");
                sb.append(stringBuffer.toString());
            }
            Iterator it2 = rrset.sigs();
            while (it2.hasNext()) {
                StringBuffer stringBuffer2 = new StringBuffer();
                stringBuffer2.append(it2.next());
                stringBuffer2.append("\n");
                sb.append(stringBuffer2.toString());
            }
        }
    }

    public synchronized String toMasterFile() {
        StringBuffer sb;
        sb = new StringBuffer();
        nodeToString(sb, this.originNode);
        for (Map.Entry entry : this.data.entrySet()) {
            if (!this.origin.equals(entry.getKey())) {
                nodeToString(sb, entry.getValue());
            }
        }
        return sb.toString();
    }

    public String toString() {
        return toMasterFile();
    }
}
