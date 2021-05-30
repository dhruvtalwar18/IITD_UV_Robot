package org.xbill.DNS;

import java.io.IOException;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class Cache {
    private static final int defaultMaxEntries = 50000;
    private CacheMap data;
    private int dclass;
    private int maxcache;
    private int maxncache;

    private interface Element {
        int compareCredibility(int i);

        boolean expired();

        int getType();
    }

    /* access modifiers changed from: private */
    public static int limitExpire(long ttl, long maxttl) {
        if (maxttl >= 0 && maxttl < ttl) {
            ttl = maxttl;
        }
        long expire = (System.currentTimeMillis() / 1000) + ttl;
        if (expire < 0 || expire > TTL.MAX_VALUE) {
            return Integer.MAX_VALUE;
        }
        return (int) expire;
    }

    private static class CacheRRset extends RRset implements Element {
        private static final long serialVersionUID = 5971755205903597024L;
        int credibility;
        int expire;

        public CacheRRset(Record rec, int cred, long maxttl) {
            this.credibility = cred;
            this.expire = Cache.limitExpire(rec.getTTL(), maxttl);
            addRR(rec);
        }

        public CacheRRset(RRset rrset, int cred, long maxttl) {
            super(rrset);
            this.credibility = cred;
            this.expire = Cache.limitExpire(rrset.getTTL(), maxttl);
        }

        public final boolean expired() {
            return ((int) (System.currentTimeMillis() / 1000)) >= this.expire;
        }

        public final int compareCredibility(int cred) {
            return this.credibility - cred;
        }

        public String toString() {
            StringBuffer sb = new StringBuffer();
            sb.append(super.toString());
            sb.append(" cl = ");
            sb.append(this.credibility);
            return sb.toString();
        }
    }

    private static class NegativeElement implements Element {
        int credibility;
        int expire;
        Name name;
        int type;

        public NegativeElement(Name name2, int type2, SOARecord soa, int cred, long maxttl) {
            this.name = name2;
            this.type = type2;
            long cttl = soa != null ? soa.getMinimum() : 0;
            this.credibility = cred;
            this.expire = Cache.limitExpire(cttl, maxttl);
        }

        public int getType() {
            return this.type;
        }

        public final boolean expired() {
            return ((int) (System.currentTimeMillis() / 1000)) >= this.expire;
        }

        public final int compareCredibility(int cred) {
            return this.credibility - cred;
        }

        public String toString() {
            StringBuffer sb = new StringBuffer();
            if (this.type == 0) {
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append("NXDOMAIN ");
                stringBuffer.append(this.name);
                sb.append(stringBuffer.toString());
            } else {
                StringBuffer stringBuffer2 = new StringBuffer();
                stringBuffer2.append("NXRRSET ");
                stringBuffer2.append(this.name);
                stringBuffer2.append(" ");
                stringBuffer2.append(Type.string(this.type));
                sb.append(stringBuffer2.toString());
            }
            sb.append(" cl = ");
            sb.append(this.credibility);
            return sb.toString();
        }
    }

    private static class CacheMap extends LinkedHashMap {
        private int maxsize = -1;

        CacheMap(int maxsize2) {
            super(16, 0.75f, true);
            this.maxsize = maxsize2;
        }

        /* access modifiers changed from: package-private */
        public int getMaxSize() {
            return this.maxsize;
        }

        /* access modifiers changed from: package-private */
        public void setMaxSize(int maxsize2) {
            this.maxsize = maxsize2;
        }

        /* access modifiers changed from: protected */
        public boolean removeEldestEntry(Map.Entry eldest) {
            return this.maxsize >= 0 && size() > this.maxsize;
        }
    }

    public Cache(int dclass2) {
        this.maxncache = -1;
        this.maxcache = -1;
        this.dclass = dclass2;
        this.data = new CacheMap(defaultMaxEntries);
    }

    public Cache() {
        this(1);
    }

    public Cache(String file) throws IOException {
        this.maxncache = -1;
        this.maxcache = -1;
        this.data = new CacheMap(defaultMaxEntries);
        Master m = new Master(file);
        while (true) {
            Record nextRecord = m.nextRecord();
            Record record = nextRecord;
            if (nextRecord != null) {
                addRecord(record, 0, m);
            } else {
                return;
            }
        }
    }

    private synchronized Object exactName(Name name) {
        return this.data.get(name);
    }

    private synchronized void removeName(Name name) {
        this.data.remove(name);
    }

    private synchronized Element[] allElements(Object types) {
        if (types instanceof List) {
            List typelist = (List) types;
            return (Element[]) typelist.toArray(new Element[typelist.size()]);
        }
        return new Element[]{(Element) types};
    }

    private synchronized Element oneElement(Name name, Object types, int type, int minCred) {
        Element found = null;
        if (type != 255) {
            if (types instanceof List) {
                List list = (List) types;
                int i = 0;
                while (true) {
                    if (i >= list.size()) {
                        break;
                    }
                    Element set = (Element) list.get(i);
                    if (set.getType() == type) {
                        found = set;
                        break;
                    }
                    i++;
                }
            } else {
                Element set2 = (Element) types;
                if (set2.getType() == type) {
                    found = set2;
                }
            }
            if (found == null) {
                return null;
            }
            if (found.expired()) {
                removeElement(name, type);
                return null;
            } else if (found.compareCredibility(minCred) < 0) {
                return null;
            } else {
                return found;
            }
        } else {
            throw new IllegalArgumentException("oneElement(ANY)");
        }
    }

    private synchronized Element findElement(Name name, int type, int minCred) {
        Object types = exactName(name);
        if (types == null) {
            return null;
        }
        return oneElement(name, types, type, minCred);
    }

    /* JADX WARNING: Code restructure failed: missing block: B:26:0x005a, code lost:
        return;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    private synchronized void addElement(org.xbill.DNS.Name r7, org.xbill.DNS.Cache.Element r8) {
        /*
            r6 = this;
            monitor-enter(r6)
            org.xbill.DNS.Cache$CacheMap r0 = r6.data     // Catch:{ all -> 0x005b }
            java.lang.Object r0 = r0.get(r7)     // Catch:{ all -> 0x005b }
            if (r0 != 0) goto L_0x0010
            org.xbill.DNS.Cache$CacheMap r1 = r6.data     // Catch:{ all -> 0x005b }
            r1.put(r7, r8)     // Catch:{ all -> 0x005b }
            monitor-exit(r6)
            return
        L_0x0010:
            int r1 = r8.getType()     // Catch:{ all -> 0x005b }
            boolean r2 = r0 instanceof java.util.List     // Catch:{ all -> 0x005b }
            if (r2 == 0) goto L_0x003a
            r2 = r0
            java.util.List r2 = (java.util.List) r2     // Catch:{ all -> 0x005b }
            r3 = 0
        L_0x001c:
            int r4 = r2.size()     // Catch:{ all -> 0x005b }
            if (r3 >= r4) goto L_0x0036
            java.lang.Object r4 = r2.get(r3)     // Catch:{ all -> 0x005b }
            org.xbill.DNS.Cache$Element r4 = (org.xbill.DNS.Cache.Element) r4     // Catch:{ all -> 0x005b }
            int r5 = r4.getType()     // Catch:{ all -> 0x005b }
            if (r5 != r1) goto L_0x0033
            r2.set(r3, r8)     // Catch:{ all -> 0x005b }
            monitor-exit(r6)
            return
        L_0x0033:
            int r3 = r3 + 1
            goto L_0x001c
        L_0x0036:
            r2.add(r8)     // Catch:{ all -> 0x005b }
            goto L_0x0059
        L_0x003a:
            r2 = r0
            org.xbill.DNS.Cache$Element r2 = (org.xbill.DNS.Cache.Element) r2     // Catch:{ all -> 0x005b }
            int r3 = r2.getType()     // Catch:{ all -> 0x005b }
            if (r3 != r1) goto L_0x0049
            org.xbill.DNS.Cache$CacheMap r3 = r6.data     // Catch:{ all -> 0x005b }
            r3.put(r7, r8)     // Catch:{ all -> 0x005b }
            goto L_0x0059
        L_0x0049:
            java.util.LinkedList r3 = new java.util.LinkedList     // Catch:{ all -> 0x005b }
            r3.<init>()     // Catch:{ all -> 0x005b }
            r3.add(r2)     // Catch:{ all -> 0x005b }
            r3.add(r8)     // Catch:{ all -> 0x005b }
            org.xbill.DNS.Cache$CacheMap r4 = r6.data     // Catch:{ all -> 0x005b }
            r4.put(r7, r3)     // Catch:{ all -> 0x005b }
        L_0x0059:
            monitor-exit(r6)
            return
        L_0x005b:
            r7 = move-exception
            monitor-exit(r6)
            throw r7
        */
        throw new UnsupportedOperationException("Method not decompiled: org.xbill.DNS.Cache.addElement(org.xbill.DNS.Name, org.xbill.DNS.Cache$Element):void");
    }

    /* JADX WARNING: Code restructure failed: missing block: B:18:0x0034, code lost:
        return;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:29:0x004a, code lost:
        return;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    private synchronized void removeElement(org.xbill.DNS.Name r6, int r7) {
        /*
            r5 = this;
            monitor-enter(r5)
            org.xbill.DNS.Cache$CacheMap r0 = r5.data     // Catch:{ all -> 0x004b }
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
            org.xbill.DNS.Cache$Element r3 = (org.xbill.DNS.Cache.Element) r3     // Catch:{ all -> 0x004b }
            int r4 = r3.getType()     // Catch:{ all -> 0x004b }
            if (r4 != r7) goto L_0x0035
            r1.remove(r2)     // Catch:{ all -> 0x004b }
            int r4 = r1.size()     // Catch:{ all -> 0x004b }
            if (r4 != 0) goto L_0x0033
            org.xbill.DNS.Cache$CacheMap r4 = r5.data     // Catch:{ all -> 0x004b }
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
            org.xbill.DNS.Cache$Element r1 = (org.xbill.DNS.Cache.Element) r1     // Catch:{ all -> 0x004b }
            int r2 = r1.getType()     // Catch:{ all -> 0x004b }
            if (r2 == r7) goto L_0x0044
            monitor-exit(r5)
            return
        L_0x0044:
            org.xbill.DNS.Cache$CacheMap r2 = r5.data     // Catch:{ all -> 0x004b }
            r2.remove(r6)     // Catch:{ all -> 0x004b }
        L_0x0049:
            monitor-exit(r5)
            return
        L_0x004b:
            r6 = move-exception
            monitor-exit(r5)
            throw r6
        */
        throw new UnsupportedOperationException("Method not decompiled: org.xbill.DNS.Cache.removeElement(org.xbill.DNS.Name, int):void");
    }

    public synchronized void clearCache() {
        this.data.clear();
    }

    /* JADX WARNING: Code restructure failed: missing block: B:16:0x0034, code lost:
        return;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public synchronized void addRecord(org.xbill.DNS.Record r7, int r8, java.lang.Object r9) {
        /*
            r6 = this;
            monitor-enter(r6)
            org.xbill.DNS.Name r0 = r7.getName()     // Catch:{ all -> 0x0035 }
            int r1 = r7.getRRsetType()     // Catch:{ all -> 0x0035 }
            boolean r2 = org.xbill.DNS.Type.isRR(r1)     // Catch:{ all -> 0x0035 }
            if (r2 != 0) goto L_0x0011
            monitor-exit(r6)
            return
        L_0x0011:
            org.xbill.DNS.Cache$Element r2 = r6.findElement(r0, r1, r8)     // Catch:{ all -> 0x0035 }
            if (r2 != 0) goto L_0x0023
            org.xbill.DNS.Cache$CacheRRset r3 = new org.xbill.DNS.Cache$CacheRRset     // Catch:{ all -> 0x0035 }
            int r4 = r6.maxcache     // Catch:{ all -> 0x0035 }
            long r4 = (long) r4     // Catch:{ all -> 0x0035 }
            r3.<init>((org.xbill.DNS.Record) r7, (int) r8, (long) r4)     // Catch:{ all -> 0x0035 }
            r6.addRRset(r3, r8)     // Catch:{ all -> 0x0035 }
            goto L_0x0033
        L_0x0023:
            int r3 = r2.compareCredibility(r8)     // Catch:{ all -> 0x0035 }
            if (r3 != 0) goto L_0x0033
            boolean r3 = r2 instanceof org.xbill.DNS.Cache.CacheRRset     // Catch:{ all -> 0x0035 }
            if (r3 == 0) goto L_0x0033
            r3 = r2
            org.xbill.DNS.Cache$CacheRRset r3 = (org.xbill.DNS.Cache.CacheRRset) r3     // Catch:{ all -> 0x0035 }
            r3.addRR(r7)     // Catch:{ all -> 0x0035 }
        L_0x0033:
            monitor-exit(r6)
            return
        L_0x0035:
            r7 = move-exception
            monitor-exit(r6)
            throw r7
        */
        throw new UnsupportedOperationException("Method not decompiled: org.xbill.DNS.Cache.addRecord(org.xbill.DNS.Record, int, java.lang.Object):void");
    }

    public synchronized void addRRset(RRset rrset, int cred) {
        CacheRRset crrset;
        long ttl = rrset.getTTL();
        Name name = rrset.getName();
        int type = rrset.getType();
        Element element = findElement(name, type, 0);
        if (ttl != 0) {
            if (element != null && element.compareCredibility(cred) <= 0) {
                element = null;
            }
            if (element == null) {
                if (rrset instanceof CacheRRset) {
                    crrset = (CacheRRset) rrset;
                } else {
                    crrset = new CacheRRset(rrset, cred, (long) this.maxcache);
                }
                addElement(name, crrset);
            }
        } else if (element != null && element.compareCredibility(cred) <= 0) {
            removeElement(name, type);
        }
    }

    public synchronized void addNegative(Name name, int type, SOARecord soa, int cred) {
        long ttl = 0;
        if (soa != null) {
            try {
                ttl = soa.getTTL();
            } catch (Throwable th) {
                throw th;
            }
        }
        Element element = findElement(name, type, 0);
        if (ttl != 0) {
            if (element != null && element.compareCredibility(cred) <= 0) {
                element = null;
            }
            if (element == null) {
                addElement(name, new NegativeElement(name, type, soa, cred, (long) this.maxncache));
            }
        } else if (element != null && element.compareCredibility(cred) <= 0) {
            removeElement(name, type);
        }
    }

    /* access modifiers changed from: protected */
    /* JADX WARNING: Removed duplicated region for block: B:20:0x0036  */
    /* JADX WARNING: Removed duplicated region for block: B:21:0x0038  */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public synchronized org.xbill.DNS.SetResponse lookup(org.xbill.DNS.Name r19, int r20, int r21) {
        /*
            r18 = this;
            r1 = r18
            r0 = r20
            r2 = r21
            monitor-enter(r18)
            int r3 = r19.labels()     // Catch:{ all -> 0x00ff }
            r4 = r3
        L_0x000c:
            r6 = 1
            if (r4 < r6) goto L_0x00f6
            if (r4 != r6) goto L_0x0013
            r7 = 1
            goto L_0x0014
        L_0x0013:
            r7 = 0
        L_0x0014:
            if (r4 != r3) goto L_0x0018
            r8 = 1
            goto L_0x0019
        L_0x0018:
            r8 = 0
        L_0x0019:
            if (r7 == 0) goto L_0x0020
            org.xbill.DNS.Name r9 = org.xbill.DNS.Name.root     // Catch:{ all -> 0x00ff }
        L_0x001d:
            r11 = r19
            goto L_0x002e
        L_0x0020:
            if (r8 == 0) goto L_0x0025
            r9 = r19
            goto L_0x001d
        L_0x0025:
            org.xbill.DNS.Name r9 = new org.xbill.DNS.Name     // Catch:{ all -> 0x00ff }
            int r10 = r3 - r4
            r11 = r19
            r9.<init>((org.xbill.DNS.Name) r11, (int) r10)     // Catch:{ all -> 0x00ff }
        L_0x002e:
            org.xbill.DNS.Cache$CacheMap r10 = r1.data     // Catch:{ all -> 0x00ff }
            java.lang.Object r10 = r10.get(r9)     // Catch:{ all -> 0x00ff }
            if (r10 != 0) goto L_0x0038
            goto L_0x00f2
        L_0x0038:
            r12 = 6
            if (r8 == 0) goto L_0x007b
            r13 = 255(0xff, float:3.57E-43)
            if (r0 != r13) goto L_0x007b
            org.xbill.DNS.SetResponse r13 = new org.xbill.DNS.SetResponse     // Catch:{ all -> 0x00ff }
            r13.<init>(r12)     // Catch:{ all -> 0x00ff }
            org.xbill.DNS.Cache$Element[] r14 = r1.allElements(r10)     // Catch:{ all -> 0x00ff }
            r15 = 0
            r16 = r15
            r15 = 0
        L_0x004c:
            int r6 = r14.length     // Catch:{ all -> 0x00ff }
            if (r15 >= r6) goto L_0x0077
            r6 = r14[r15]     // Catch:{ all -> 0x00ff }
            boolean r17 = r6.expired()     // Catch:{ all -> 0x00ff }
            if (r17 == 0) goto L_0x005f
            int r5 = r6.getType()     // Catch:{ all -> 0x00ff }
            r1.removeElement(r9, r5)     // Catch:{ all -> 0x00ff }
            goto L_0x0073
        L_0x005f:
            boolean r5 = r6 instanceof org.xbill.DNS.Cache.CacheRRset     // Catch:{ all -> 0x00ff }
            if (r5 != 0) goto L_0x0064
            goto L_0x0073
        L_0x0064:
            int r5 = r6.compareCredibility(r2)     // Catch:{ all -> 0x00ff }
            if (r5 >= 0) goto L_0x006b
            goto L_0x0073
        L_0x006b:
            r5 = r6
            org.xbill.DNS.Cache$CacheRRset r5 = (org.xbill.DNS.Cache.CacheRRset) r5     // Catch:{ all -> 0x00ff }
            r13.addRRset(r5)     // Catch:{ all -> 0x00ff }
            int r16 = r16 + 1
        L_0x0073:
            int r15 = r15 + 1
            r6 = 1
            goto L_0x004c
        L_0x0077:
            if (r16 <= 0) goto L_0x007b
            monitor-exit(r18)
            return r13
        L_0x007b:
            r5 = 5
            r6 = 2
            if (r8 == 0) goto L_0x00b6
            org.xbill.DNS.Cache$Element r13 = r1.oneElement(r9, r10, r0, r2)     // Catch:{ all -> 0x00ff }
            if (r13 == 0) goto L_0x0096
            boolean r14 = r13 instanceof org.xbill.DNS.Cache.CacheRRset     // Catch:{ all -> 0x00ff }
            if (r14 == 0) goto L_0x0096
            org.xbill.DNS.SetResponse r5 = new org.xbill.DNS.SetResponse     // Catch:{ all -> 0x00ff }
            r5.<init>(r12)     // Catch:{ all -> 0x00ff }
            r6 = r13
            org.xbill.DNS.Cache$CacheRRset r6 = (org.xbill.DNS.Cache.CacheRRset) r6     // Catch:{ all -> 0x00ff }
            r5.addRRset(r6)     // Catch:{ all -> 0x00ff }
            monitor-exit(r18)
            return r5
        L_0x0096:
            if (r13 == 0) goto L_0x009f
            org.xbill.DNS.SetResponse r5 = new org.xbill.DNS.SetResponse     // Catch:{ all -> 0x00ff }
            r5.<init>(r6)     // Catch:{ all -> 0x00ff }
            monitor-exit(r18)
            return r5
        L_0x009f:
            org.xbill.DNS.Cache$Element r5 = r1.oneElement(r9, r10, r5, r2)     // Catch:{ all -> 0x00ff }
            if (r5 == 0) goto L_0x00b4
            boolean r12 = r5 instanceof org.xbill.DNS.Cache.CacheRRset     // Catch:{ all -> 0x00ff }
            if (r12 == 0) goto L_0x00b4
            org.xbill.DNS.SetResponse r6 = new org.xbill.DNS.SetResponse     // Catch:{ all -> 0x00ff }
            r12 = 4
            r13 = r5
            org.xbill.DNS.Cache$CacheRRset r13 = (org.xbill.DNS.Cache.CacheRRset) r13     // Catch:{ all -> 0x00ff }
            r6.<init>(r12, r13)     // Catch:{ all -> 0x00ff }
            monitor-exit(r18)
            return r6
        L_0x00b4:
            r12 = r5
            goto L_0x00cc
        L_0x00b6:
            r12 = 39
            org.xbill.DNS.Cache$Element r12 = r1.oneElement(r9, r10, r12, r2)     // Catch:{ all -> 0x00ff }
            if (r12 == 0) goto L_0x00cc
            boolean r13 = r12 instanceof org.xbill.DNS.Cache.CacheRRset     // Catch:{ all -> 0x00ff }
            if (r13 == 0) goto L_0x00cc
            org.xbill.DNS.SetResponse r6 = new org.xbill.DNS.SetResponse     // Catch:{ all -> 0x00ff }
            r13 = r12
            org.xbill.DNS.Cache$CacheRRset r13 = (org.xbill.DNS.Cache.CacheRRset) r13     // Catch:{ all -> 0x00ff }
            r6.<init>(r5, r13)     // Catch:{ all -> 0x00ff }
            monitor-exit(r18)
            return r6
        L_0x00cc:
            org.xbill.DNS.Cache$Element r5 = r1.oneElement(r9, r10, r6, r2)     // Catch:{ all -> 0x00ff }
            if (r5 == 0) goto L_0x00e1
            boolean r6 = r5 instanceof org.xbill.DNS.Cache.CacheRRset     // Catch:{ all -> 0x00ff }
            if (r6 == 0) goto L_0x00e1
            org.xbill.DNS.SetResponse r6 = new org.xbill.DNS.SetResponse     // Catch:{ all -> 0x00ff }
            r12 = 3
            r13 = r5
            org.xbill.DNS.Cache$CacheRRset r13 = (org.xbill.DNS.Cache.CacheRRset) r13     // Catch:{ all -> 0x00ff }
            r6.<init>(r12, r13)     // Catch:{ all -> 0x00ff }
            monitor-exit(r18)
            return r6
        L_0x00e1:
            if (r8 == 0) goto L_0x00f2
            r6 = 0
            org.xbill.DNS.Cache$Element r6 = r1.oneElement(r9, r10, r6, r2)     // Catch:{ all -> 0x00ff }
            r5 = r6
            if (r5 == 0) goto L_0x00f2
            r6 = 1
            org.xbill.DNS.SetResponse r6 = org.xbill.DNS.SetResponse.ofType(r6)     // Catch:{ all -> 0x00ff }
            monitor-exit(r18)
            return r6
        L_0x00f2:
            int r4 = r4 + -1
            goto L_0x000c
        L_0x00f6:
            r11 = r19
            r5 = 0
            org.xbill.DNS.SetResponse r5 = org.xbill.DNS.SetResponse.ofType(r5)     // Catch:{ all -> 0x00ff }
            monitor-exit(r18)
            return r5
        L_0x00ff:
            r0 = move-exception
            monitor-exit(r18)
            throw r0
        */
        throw new UnsupportedOperationException("Method not decompiled: org.xbill.DNS.Cache.lookup(org.xbill.DNS.Name, int, int):org.xbill.DNS.SetResponse");
    }

    public SetResponse lookupRecords(Name name, int type, int minCred) {
        return lookup(name, type, minCred);
    }

    private RRset[] findRecords(Name name, int type, int minCred) {
        SetResponse cr = lookupRecords(name, type, minCred);
        if (cr.isSuccessful()) {
            return cr.answers();
        }
        return null;
    }

    public RRset[] findRecords(Name name, int type) {
        return findRecords(name, type, 3);
    }

    public RRset[] findAnyRecords(Name name, int type) {
        return findRecords(name, type, 2);
    }

    private final int getCred(int section, boolean isAuth) {
        if (section == 1) {
            return isAuth ? 4 : 3;
        }
        if (section == 2) {
            return isAuth ? 4 : 3;
        }
        if (section == 3) {
            return 1;
        }
        throw new IllegalArgumentException("getCred: invalid section");
    }

    private static void markAdditional(RRset rrset, Set names) {
        if (rrset.first().getAdditionalName() != null) {
            Iterator it = rrset.rrs();
            while (it.hasNext()) {
                Name name = ((Record) it.next()).getAdditionalName();
                if (name != null) {
                    names.add(name);
                }
            }
        }
    }

    /* JADX WARNING: type inference failed for: r18v1, types: [org.xbill.DNS.Record] */
    /* JADX WARNING: Multi-variable type inference failed */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public org.xbill.DNS.SetResponse addMessage(org.xbill.DNS.Message r26) {
        /*
            r25 = this;
            r1 = r25
            r2 = r26
            org.xbill.DNS.Header r0 = r26.getHeader()
            r3 = 5
            boolean r4 = r0.getFlag(r3)
            org.xbill.DNS.Record r5 = r26.getQuestion()
            org.xbill.DNS.Header r0 = r26.getHeader()
            int r6 = r0.getRcode()
            r0 = 0
            r7 = 0
            java.lang.String r8 = "verbosecache"
            boolean r8 = org.xbill.DNS.Options.check(r8)
            r9 = 3
            if (r6 == 0) goto L_0x0026
            if (r6 != r9) goto L_0x0028
        L_0x0026:
            if (r5 != 0) goto L_0x002a
        L_0x0028:
            r3 = 0
            return r3
        L_0x002a:
            org.xbill.DNS.Name r10 = r5.getName()
            int r11 = r5.getType()
            int r12 = r5.getDClass()
            r13 = r10
            java.util.HashSet r14 = new java.util.HashSet
            r14.<init>()
            r15 = 1
            org.xbill.DNS.RRset[] r9 = r2.getSectionRRsets(r15)
            r16 = 0
            r17 = r7
            r7 = r0
            r0 = 0
        L_0x0047:
            r18 = r0
            int r0 = r9.length
            r3 = r18
            if (r3 >= r0) goto L_0x0119
            r0 = r9[r3]
            int r0 = r0.getDClass()
            if (r0 == r12) goto L_0x0060
            r19 = r5
            r21 = r8
            r22 = r11
            r20 = r12
            goto L_0x00a7
        L_0x0060:
            r0 = r9[r3]
            int r15 = r0.getType()
            r0 = r9[r3]
            r19 = r5
            org.xbill.DNS.Name r5 = r0.getName()
            r21 = r8
            r20 = r12
            r12 = 1
            int r8 = r1.getCred(r12, r4)
            if (r15 == r11) goto L_0x007d
            r0 = 255(0xff, float:3.57E-43)
            if (r11 != r0) goto L_0x00aa
        L_0x007d:
            boolean r0 = r5.equals(r13)
            if (r0 == 0) goto L_0x00aa
            r0 = r9[r3]
            r1.addRRset(r0, r8)
            r0 = 1
            if (r13 != r10) goto L_0x009f
            if (r17 != 0) goto L_0x0096
            org.xbill.DNS.SetResponse r7 = new org.xbill.DNS.SetResponse
            r12 = 6
            r7.<init>(r12)
            r17 = r7
            goto L_0x0098
        L_0x0096:
            r7 = r17
        L_0x0098:
            r12 = r9[r3]
            r7.addRRset(r12)
            r17 = r7
        L_0x009f:
            r7 = r9[r3]
            markAdditional(r7, r14)
            r7 = r0
            r22 = r11
        L_0x00a7:
            r12 = 5
            goto L_0x010b
        L_0x00aa:
            r0 = 5
            if (r15 != r0) goto L_0x00d8
            boolean r0 = r5.equals(r13)
            if (r0 == 0) goto L_0x00d8
            r0 = r9[r3]
            r1.addRRset(r0, r8)
            if (r13 != r10) goto L_0x00c7
            org.xbill.DNS.SetResponse r0 = new org.xbill.DNS.SetResponse
            r12 = 4
            r22 = r11
            r11 = r9[r3]
            r0.<init>(r12, r11)
            r17 = r0
            goto L_0x00c9
        L_0x00c7:
            r22 = r11
        L_0x00c9:
            r0 = r9[r3]
            org.xbill.DNS.Record r0 = r0.first()
            org.xbill.DNS.CNAMERecord r0 = (org.xbill.DNS.CNAMERecord) r0
            org.xbill.DNS.Name r0 = r0.getTarget()
            r13 = r0
            goto L_0x00a7
        L_0x00d8:
            r22 = r11
            r0 = 39
            if (r15 != r0) goto L_0x00a7
            boolean r0 = r13.subdomain(r5)
            if (r0 == 0) goto L_0x00a7
            r0 = r9[r3]
            r1.addRRset(r0, r8)
            if (r13 != r10) goto L_0x00f6
            org.xbill.DNS.SetResponse r0 = new org.xbill.DNS.SetResponse
            r11 = r9[r3]
            r12 = 5
            r0.<init>(r12, r11)
            r17 = r0
            goto L_0x00f7
        L_0x00f6:
            r12 = 5
        L_0x00f7:
            r0 = r9[r3]
            org.xbill.DNS.Record r0 = r0.first()
            org.xbill.DNS.DNAMERecord r0 = (org.xbill.DNS.DNAMERecord) r0
            r11 = r0
            org.xbill.DNS.Name r0 = r13.fromDNAME(r11)     // Catch:{ NameTooLongException -> 0x0107 }
            r13 = r0
            goto L_0x010b
        L_0x0107:
            r0 = move-exception
            r12 = r0
            r0 = r12
            goto L_0x0121
        L_0x010b:
            int r0 = r3 + 1
            r5 = r19
            r12 = r20
            r8 = r21
            r11 = r22
            r3 = 5
            r15 = 1
            goto L_0x0047
        L_0x0119:
            r19 = r5
            r21 = r8
            r22 = r11
            r20 = r12
        L_0x0121:
            r0 = 2
            org.xbill.DNS.RRset[] r3 = r2.getSectionRRsets(r0)
            r5 = 0
            r8 = 0
            r11 = r8
            r8 = r5
            r5 = 0
        L_0x012b:
            int r12 = r3.length
            if (r5 >= r12) goto L_0x015f
            r12 = r3[r5]
            int r12 = r12.getType()
            r15 = 6
            if (r12 != r15) goto L_0x0146
            r12 = r3[r5]
            org.xbill.DNS.Name r12 = r12.getName()
            boolean r12 = r13.subdomain(r12)
            if (r12 == 0) goto L_0x0146
            r8 = r3[r5]
            goto L_0x015c
        L_0x0146:
            r12 = r3[r5]
            int r12 = r12.getType()
            if (r12 != r0) goto L_0x015c
            r12 = r3[r5]
            org.xbill.DNS.Name r12 = r12.getName()
            boolean r12 = r13.subdomain(r12)
            if (r12 == 0) goto L_0x015c
            r11 = r3[r5]
        L_0x015c:
            int r5 = r5 + 1
            goto L_0x012b
        L_0x015f:
            if (r7 != 0) goto L_0x01ae
            r5 = 3
            if (r6 != r5) goto L_0x0166
            r12 = 0
            goto L_0x0168
        L_0x0166:
            r12 = r22
        L_0x0168:
            if (r6 == r5) goto L_0x0189
            if (r8 != 0) goto L_0x0189
            if (r11 != 0) goto L_0x016f
            goto L_0x0189
        L_0x016f:
            int r0 = r1.getCred(r0, r4)
            r1.addRRset(r11, r0)
            markAdditional(r11, r14)
            if (r17 != 0) goto L_0x0186
            org.xbill.DNS.SetResponse r5 = new org.xbill.DNS.SetResponse
            r15 = 3
            r5.<init>(r15, r11)
            r23 = r3
            r17 = r5
            goto L_0x01ad
        L_0x0186:
            r23 = r3
            goto L_0x01ad
        L_0x0189:
            int r5 = r1.getCred(r0, r4)
            r15 = 0
            if (r8 == 0) goto L_0x0198
            org.xbill.DNS.Record r18 = r8.first()
            r15 = r18
            org.xbill.DNS.SOARecord r15 = (org.xbill.DNS.SOARecord) r15
        L_0x0198:
            r1.addNegative(r13, r12, r15, r5)
            if (r17 != 0) goto L_0x01aa
            r23 = r3
            r3 = 3
            if (r6 != r3) goto L_0x01a4
            r0 = 1
            goto L_0x01a5
        L_0x01a4:
        L_0x01a5:
            org.xbill.DNS.SetResponse r17 = org.xbill.DNS.SetResponse.ofType(r0)
            goto L_0x01ac
        L_0x01aa:
            r23 = r3
        L_0x01ac:
            r0 = r5
        L_0x01ad:
            goto L_0x01be
        L_0x01ae:
            r23 = r3
            if (r6 != 0) goto L_0x01be
            if (r11 == 0) goto L_0x01be
            int r0 = r1.getCred(r0, r4)
            r1.addRRset(r11, r0)
            markAdditional(r11, r14)
        L_0x01be:
            r0 = r17
            r3 = 3
            org.xbill.DNS.RRset[] r5 = r2.getSectionRRsets(r3)
        L_0x01c6:
            r3 = r16
            int r12 = r5.length
            if (r3 >= r12) goto L_0x0200
            r12 = r5[r3]
            int r12 = r12.getType()
            r15 = 1
            if (r12 == r15) goto L_0x01dd
            r15 = 28
            if (r12 == r15) goto L_0x01dd
            r15 = 38
            if (r12 == r15) goto L_0x01dd
            goto L_0x01ea
        L_0x01dd:
            r15 = r5[r3]
            org.xbill.DNS.Name r15 = r15.getName()
            boolean r16 = r14.contains(r15)
            if (r16 != 0) goto L_0x01ed
        L_0x01ea:
            r24 = r6
            goto L_0x01f9
        L_0x01ed:
            r24 = r6
            r2 = 3
            int r6 = r1.getCred(r2, r4)
            r2 = r5[r3]
            r1.addRRset(r2, r6)
        L_0x01f9:
            int r16 = r3 + 1
            r6 = r24
            r2 = r26
            goto L_0x01c6
        L_0x0200:
            r24 = r6
            if (r21 == 0) goto L_0x021a
            java.io.PrintStream r2 = java.lang.System.out
            java.lang.StringBuffer r3 = new java.lang.StringBuffer
            r3.<init>()
            java.lang.String r6 = "addMessage: "
            r3.append(r6)
            r3.append(r0)
            java.lang.String r3 = r3.toString()
            r2.println(r3)
        L_0x021a:
            return r0
        */
        throw new UnsupportedOperationException("Method not decompiled: org.xbill.DNS.Cache.addMessage(org.xbill.DNS.Message):org.xbill.DNS.SetResponse");
    }

    public void flushSet(Name name, int type) {
        removeElement(name, type);
    }

    public void flushName(Name name) {
        removeName(name);
    }

    public void setMaxNCache(int seconds) {
        this.maxncache = seconds;
    }

    public int getMaxNCache() {
        return this.maxncache;
    }

    public void setMaxCache(int seconds) {
        this.maxcache = seconds;
    }

    public int getMaxCache() {
        return this.maxcache;
    }

    public int getSize() {
        return this.data.size();
    }

    public int getMaxEntries() {
        return this.data.getMaxSize();
    }

    public void setMaxEntries(int entries) {
        this.data.setMaxSize(entries);
    }

    public int getDClass() {
        return this.dclass;
    }

    public String toString() {
        StringBuffer sb = new StringBuffer();
        synchronized (this) {
            for (Object allElements : this.data.values()) {
                Element[] elements = allElements(allElements);
                for (Element append : elements) {
                    sb.append(append);
                    sb.append("\n");
                }
            }
        }
        return sb.toString();
    }
}
