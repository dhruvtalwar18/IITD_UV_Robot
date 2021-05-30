package org.apache.commons.net.ftp.parser;

import org.apache.commons.net.ftp.FTPClientConfig;

public class UnixFTPEntryParser extends ConfigurableFTPFileEntryParserImpl {
    static final String DEFAULT_DATE_FORMAT = "MMM d yyyy";
    static final String DEFAULT_RECENT_DATE_FORMAT = "MMM d HH:mm";
    public static final FTPClientConfig NUMERIC_DATE_CONFIG = new FTPClientConfig(FTPClientConfig.SYST_UNIX, NUMERIC_DATE_FORMAT, (String) null, (String) null, (String) null, (String) null);
    static final String NUMERIC_DATE_FORMAT = "yyyy-MM-dd HH:mm";
    private static final String REGEX = "([bcdelfmpSs-])(((r|-)(w|-)([xsStTL-]))((r|-)(w|-)([xsStTL-]))((r|-)(w|-)([xsStTL-])))\\+?\\s+(\\d+)\\s+(?:(\\S+(?:\\s\\S+)*?)\\s+)?(?:(\\S+(?:\\s\\S+)*)\\s+)?(\\d+(?:,\\s*\\d+)?)\\s+((?:\\d+[-/]\\d+[-/]\\d+)|(?:\\S+\\s+\\S+))\\s+(\\d+(?::\\d+)?)\\s+(\\S*)(\\s*.*)";

    public UnixFTPEntryParser() {
        this((FTPClientConfig) null);
    }

    public UnixFTPEntryParser(FTPClientConfig config) {
        super(REGEX);
        configure(config);
    }

    /* JADX WARNING: Removed duplicated region for block: B:21:0x008f  */
    /* JADX WARNING: Removed duplicated region for block: B:29:0x00e1 A[SYNTHETIC, Splitter:B:29:0x00e1] */
    /* JADX WARNING: Removed duplicated region for block: B:37:0x00fb  */
    /* JADX WARNING: Removed duplicated region for block: B:38:0x00ff  */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public org.apache.commons.net.ftp.FTPFile parseFTPEntry(java.lang.String r18) {
        /*
            r17 = this;
            r1 = r17
            org.apache.commons.net.ftp.FTPFile r0 = new org.apache.commons.net.ftp.FTPFile
            r0.<init>()
            r2 = r0
            r3 = r18
            r2.setRawListing(r3)
            r4 = 0
            boolean r0 = r17.matches(r18)
            if (r0 == 0) goto L_0x0134
            r5 = 1
            java.lang.String r6 = r1.group(r5)
            r0 = 15
            java.lang.String r7 = r1.group(r0)
            r0 = 16
            java.lang.String r8 = r1.group(r0)
            r0 = 17
            java.lang.String r9 = r1.group(r0)
            r0 = 18
            java.lang.String r10 = r1.group(r0)
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            r11 = 19
            java.lang.String r11 = r1.group(r11)
            r0.append(r11)
            java.lang.String r11 = " "
            r0.append(r11)
            r11 = 20
            java.lang.String r11 = r1.group(r11)
            r0.append(r11)
            java.lang.String r11 = r0.toString()
            r0 = 21
            java.lang.String r12 = r1.group(r0)
            r0 = 22
            java.lang.String r0 = r1.group(r0)
            r13 = r0
            java.util.Calendar r0 = super.parseTimestamp(r11)     // Catch:{ ParseException -> 0x0066 }
            r2.setTimestamp(r0)     // Catch:{ ParseException -> 0x0066 }
            goto L_0x0067
        L_0x0066:
            r0 = move-exception
        L_0x0067:
            r14 = 0
            char r0 = r6.charAt(r14)
            r15 = 45
            if (r0 == r15) goto L_0x0084
            r15 = 108(0x6c, float:1.51E-43)
            if (r0 == r15) goto L_0x0082
            switch(r0) {
                case 98: goto L_0x007f;
                case 99: goto L_0x007f;
                case 100: goto L_0x007d;
                case 101: goto L_0x007b;
                case 102: goto L_0x0084;
                default: goto L_0x0077;
            }
        L_0x0077:
            r0 = 3
        L_0x0078:
            r15 = r4
            r4 = r0
            goto L_0x0086
        L_0x007b:
            r0 = 2
            goto L_0x0078
        L_0x007d:
            r0 = 1
            goto L_0x0078
        L_0x007f:
            r0 = 1
            r4 = r0
            goto L_0x0084
        L_0x0082:
            r0 = 2
            goto L_0x0078
        L_0x0084:
            r0 = 0
            goto L_0x0078
        L_0x0086:
            r2.setType(r4)
            r0 = 4
            r14 = r0
            r0 = 0
        L_0x008c:
            r5 = 3
            if (r0 >= r5) goto L_0x00df
            java.lang.String r5 = r1.group(r14)
            java.lang.String r3 = "-"
            boolean r3 = r5.equals(r3)
            r5 = 1
            r3 = r3 ^ r5
            r5 = 0
            r2.setPermission(r0, r5, r3)
            int r3 = r14 + 1
            java.lang.String r3 = r1.group(r3)
            java.lang.String r5 = "-"
            boolean r3 = r3.equals(r5)
            r5 = 1
            r3 = r3 ^ r5
            r2.setPermission(r0, r5, r3)
            int r3 = r14 + 2
            java.lang.String r3 = r1.group(r3)
            java.lang.String r5 = "-"
            boolean r5 = r3.equals(r5)
            if (r5 != 0) goto L_0x00cf
            r5 = 0
            char r16 = r3.charAt(r5)
            boolean r16 = java.lang.Character.isUpperCase(r16)
            if (r16 != 0) goto L_0x00cf
            r1 = 2
            r5 = 1
            r2.setPermission(r0, r1, r5)
            goto L_0x00d5
        L_0x00cf:
            r1 = 2
            r5 = 1
            r5 = 0
            r2.setPermission(r0, r1, r5)
        L_0x00d5:
            int r0 = r0 + 1
            int r14 = r14 + 4
            r1 = r17
            r3 = r18
            r5 = 1
            goto L_0x008c
        L_0x00df:
            if (r15 != 0) goto L_0x00ea
            int r0 = java.lang.Integer.parseInt(r7)     // Catch:{ NumberFormatException -> 0x00e9 }
            r2.setHardLinkCount(r0)     // Catch:{ NumberFormatException -> 0x00e9 }
            goto L_0x00ea
        L_0x00e9:
            r0 = move-exception
        L_0x00ea:
            r2.setUser(r8)
            r2.setGroup(r9)
            long r0 = java.lang.Long.parseLong(r10)     // Catch:{ NumberFormatException -> 0x00f8 }
            r2.setSize(r0)     // Catch:{ NumberFormatException -> 0x00f8 }
            goto L_0x00f9
        L_0x00f8:
            r0 = move-exception
        L_0x00f9:
            if (r13 != 0) goto L_0x00ff
            r2.setName(r12)
            goto L_0x0133
        L_0x00ff:
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            r0.append(r12)
            r0.append(r13)
            java.lang.String r12 = r0.toString()
            r1 = 2
            if (r4 != r1) goto L_0x0130
            java.lang.String r0 = " -> "
            int r0 = r12.indexOf(r0)
            r1 = -1
            if (r0 != r1) goto L_0x011e
            r2.setName(r12)
            goto L_0x012f
        L_0x011e:
            r1 = 0
            java.lang.String r1 = r12.substring(r1, r0)
            r2.setName(r1)
            int r1 = r0 + 4
            java.lang.String r1 = r12.substring(r1)
            r2.setLink(r1)
        L_0x012f:
            goto L_0x0133
        L_0x0130:
            r2.setName(r12)
        L_0x0133:
            return r2
        L_0x0134:
            r0 = 0
            return r0
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.net.ftp.parser.UnixFTPEntryParser.parseFTPEntry(java.lang.String):org.apache.commons.net.ftp.FTPFile");
    }

    /* access modifiers changed from: protected */
    public FTPClientConfig getDefaultConfiguration() {
        return new FTPClientConfig(FTPClientConfig.SYST_UNIX, "MMM d yyyy", "MMM d HH:mm", (String) null, (String) null, (String) null);
    }
}
