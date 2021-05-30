package defpackage;

import com.github.rosjava.android_remocons.common_tools.master.MasterDescription;
import java.io.IOException;
import java.io.PrintStream;
import org.xbill.DNS.Message;
import org.xbill.DNS.Name;
import org.xbill.DNS.Record;

/* renamed from: dig  reason: default package */
public class dig {
    static int dclass = 1;
    static Name name = null;
    static int type = 1;

    static void usage() {
        System.out.println("Usage: dig [@server] name [<type>] [<class>] [options]");
        System.exit(0);
    }

    static void doQuery(Message response, long ms) throws IOException {
        System.out.println("; java dig 0.0");
        System.out.println(response);
        PrintStream printStream = System.out;
        StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append(";; Query time: ");
        stringBuffer.append(ms);
        stringBuffer.append(" ms");
        printStream.println(stringBuffer.toString());
    }

    static void doAXFR(Message response) throws IOException {
        PrintStream printStream = System.out;
        StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append("; java dig 0.0 <> ");
        stringBuffer.append(name);
        stringBuffer.append(" axfr");
        printStream.println(stringBuffer.toString());
        if (response.isSigned()) {
            System.out.print(";; TSIG ");
            if (response.isVerified()) {
                System.out.println(MasterDescription.OK);
            } else {
                System.out.println("failed");
            }
        }
        if (response.getRcode() != 0) {
            System.out.println(response);
            return;
        }
        Record[] records = response.getSectionArray(1);
        for (Record println : records) {
            System.out.println(println);
        }
        System.out.print(";; done (");
        System.out.print(response.getHeader().getCount(1));
        System.out.print(" records, ");
        System.out.print(response.getHeader().getCount(3));
        System.out.println(" additional)");
    }

    /* JADX WARNING: type inference failed for: r5v0 */
    /* JADX WARNING: type inference failed for: r5v4, types: [boolean, int] */
    /* JADX WARNING: type inference failed for: r5v5 */
    /* JADX WARNING: Removed duplicated region for block: B:100:0x01a6  */
    /* JADX WARNING: Removed duplicated region for block: B:103:0x01bd  */
    /* JADX WARNING: Removed duplicated region for block: B:104:0x01c1  */
    /* JADX WARNING: Removed duplicated region for block: B:95:0x018b  */
    /* JADX WARNING: Removed duplicated region for block: B:97:0x0190  */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public static void main(java.lang.String[] r16) throws java.io.IOException {
        /*
            r1 = r16
            r2 = 0
            r3 = 0
            r4 = 0
            int r0 = r1.length
            r5 = 1
            if (r0 >= r5) goto L_0x000c
            usage()
        L_0x000c:
            r6 = 0
            r0 = r1[r6]     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0185 }
            java.lang.String r7 = "@"
            boolean r0 = r0.startsWith(r7)     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0185 }
            if (r0 == 0) goto L_0x0025
            int r7 = r6 + 1
            r0 = r1[r6]     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            java.lang.String r0 = r0.substring(r5)     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            r2 = r0
            r6 = r7
            goto L_0x0025
        L_0x0022:
            r0 = move-exception
            goto L_0x0187
        L_0x0025:
            if (r2 == 0) goto L_0x002e
            org.xbill.DNS.SimpleResolver r0 = new org.xbill.DNS.SimpleResolver     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0185 }
            r0.<init>(r2)     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0185 }
        L_0x002c:
            r3 = r0
            goto L_0x0034
        L_0x002e:
            org.xbill.DNS.SimpleResolver r0 = new org.xbill.DNS.SimpleResolver     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0185 }
            r0.<init>()     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0185 }
            goto L_0x002c
        L_0x0034:
            int r7 = r6 + 1
            r0 = r1[r6]     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            r6 = r0
            java.lang.String r0 = "-x"
            boolean r0 = r6.equals(r0)     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            if (r0 == 0) goto L_0x005e
            int r10 = r7 + 1
            r0 = r1[r7]     // Catch:{ ArrayIndexOutOfBoundsException -> 0x005a }
            org.xbill.DNS.Name r0 = org.xbill.DNS.ReverseMap.fromAddress((java.lang.String) r0)     // Catch:{ ArrayIndexOutOfBoundsException -> 0x005a }
            name = r0     // Catch:{ ArrayIndexOutOfBoundsException -> 0x005a }
            r0 = 12
            type = r0     // Catch:{ ArrayIndexOutOfBoundsException -> 0x005a }
            dclass = r5     // Catch:{ ArrayIndexOutOfBoundsException -> 0x005a }
            r7 = r10
        L_0x0052:
            r0 = 0
            r10 = 0
            r11 = 0
            r12 = 0
            r13 = 0
            r14 = 0
            r15 = 0
            goto L_0x0089
        L_0x005a:
            r0 = move-exception
            r7 = r10
            goto L_0x0187
        L_0x005e:
            org.xbill.DNS.Name r0 = org.xbill.DNS.Name.root     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            org.xbill.DNS.Name r0 = org.xbill.DNS.Name.fromString(r6, r0)     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            name = r0     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            r0 = r1[r7]     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            int r0 = org.xbill.DNS.Type.value(r0)     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            type = r0     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            int r0 = type     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            if (r0 >= 0) goto L_0x0075
            type = r5     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            goto L_0x0077
        L_0x0075:
            int r7 = r7 + 1
        L_0x0077:
            r0 = r1[r7]     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            int r0 = org.xbill.DNS.DClass.value(r0)     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            dclass = r0     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            int r0 = dclass     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            if (r0 >= 0) goto L_0x0086
            dclass = r5     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            goto L_0x0052
        L_0x0086:
            int r7 = r7 + 1
            goto L_0x0052
        L_0x0089:
            r8 = r1[r7]     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            java.lang.String r9 = "-"
            boolean r8 = r8.startsWith(r9)     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            if (r8 == 0) goto L_0x0184
            r8 = r1[r7]     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            int r8 = r8.length()     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            if (r8 <= r5) goto L_0x0184
            r8 = r1[r7]     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            char r8 = r8.charAt(r5)     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            r9 = 2
            switch(r8) {
                case 98: goto L_0x0147;
                case 100: goto L_0x013c;
                case 101: goto L_0x0102;
                case 105: goto L_0x00fe;
                case 107: goto L_0x00e1;
                case 112: goto L_0x00b4;
                case 113: goto L_0x00af;
                case 116: goto L_0x00ab;
                default: goto L_0x00a5;
            }     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
        L_0x00a5:
            r5 = 0
            r8 = 0
            java.io.PrintStream r9 = java.lang.System.out     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            goto L_0x0174
        L_0x00ab:
            r3.setTCP(r5)     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            goto L_0x00b1
        L_0x00af:
            r4 = 1
        L_0x00b1:
            r8 = 0
            goto L_0x0180
        L_0x00b4:
            r8 = r1[r7]     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            int r8 = r8.length()     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            if (r8 <= r9) goto L_0x00c3
            r8 = r1[r7]     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            java.lang.String r8 = r8.substring(r9)     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            goto L_0x00c7
        L_0x00c3:
            int r7 = r7 + 1
            r8 = r1[r7]     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
        L_0x00c7:
            int r9 = java.lang.Integer.parseInt(r8)     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            if (r9 < 0) goto L_0x00d9
            r10 = 65536(0x10000, float:9.18355E-41)
            if (r9 <= r10) goto L_0x00d2
            goto L_0x00d9
        L_0x00d2:
            r3.setPort(r9)     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            r10 = r8
            r14 = r9
            goto L_0x00b1
        L_0x00d9:
            java.io.PrintStream r0 = java.lang.System.out     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            java.lang.String r5 = "Invalid port"
            r0.println(r5)     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            return
        L_0x00e1:
            r8 = r1[r7]     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            int r8 = r8.length()     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            if (r8 <= r9) goto L_0x00f0
            r8 = r1[r7]     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            java.lang.String r8 = r8.substring(r9)     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            goto L_0x00f4
        L_0x00f0:
            int r7 = r7 + 1
            r8 = r1[r7]     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
        L_0x00f4:
            org.xbill.DNS.TSIG r9 = org.xbill.DNS.TSIG.fromString(r8)     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            r3.setTSIGKey(r9)     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            r12 = r8
            goto L_0x00b1
        L_0x00fe:
            r3.setIgnoreTruncation(r5)     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            goto L_0x00b1
        L_0x0102:
            r8 = r1[r7]     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            int r8 = r8.length()     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            if (r8 <= r9) goto L_0x0111
            r8 = r1[r7]     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            java.lang.String r8 = r8.substring(r9)     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            goto L_0x0115
        L_0x0111:
            int r7 = r7 + 1
            r8 = r1[r7]     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
        L_0x0115:
            int r9 = java.lang.Integer.parseInt(r8)     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            if (r9 < 0) goto L_0x0125
            if (r9 <= r5) goto L_0x011e
            goto L_0x0125
        L_0x011e:
            r3.setEDNS(r9)     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            r13 = r8
            r15 = r9
            goto L_0x00b1
        L_0x0125:
            java.io.PrintStream r5 = java.lang.System.out     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            java.lang.StringBuffer r13 = new java.lang.StringBuffer     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            r13.<init>()     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            java.lang.String r15 = "Unsupported EDNS level: "
            r13.append(r15)     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            r13.append(r9)     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            java.lang.String r13 = r13.toString()     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            r5.println(r13)     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            return
        L_0x013c:
            r8 = 32768(0x8000, float:4.5918E-41)
            r5 = 0
            r9 = 0
            r3.setEDNS(r5, r5, r8, r9)     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            r8 = r9
            goto L_0x0180
        L_0x0147:
            r5 = 0
            r8 = 0
            r0 = r1[r7]     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            int r0 = r0.length()     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            if (r0 <= r9) goto L_0x0158
            r0 = r1[r7]     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            java.lang.String r0 = r0.substring(r9)     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            goto L_0x015c
        L_0x0158:
            int r7 = r7 + 1
            r0 = r1[r7]     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
        L_0x015c:
            r9 = r0
            java.net.InetAddress r0 = java.net.InetAddress.getByName(r9)     // Catch:{ Exception -> 0x0169 }
            r3.setLocalAddress((java.net.InetAddress) r0)     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            r11 = r0
            r0 = r9
            goto L_0x0180
        L_0x0169:
            r0 = move-exception
            r5 = r0
            r0 = r5
            java.io.PrintStream r5 = java.lang.System.out     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            java.lang.String r8 = "Invalid address"
            r5.println(r8)     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            return
        L_0x0174:
            java.lang.String r5 = "Invalid option: "
            r9.print(r5)     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            java.io.PrintStream r5 = java.lang.System.out     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            r9 = r1[r7]     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
            r5.println(r9)     // Catch:{ ArrayIndexOutOfBoundsException -> 0x0022 }
        L_0x0180:
            r5 = 1
            int r7 = r7 + r5
            goto L_0x0089
        L_0x0184:
            goto L_0x018e
        L_0x0185:
            r0 = move-exception
            r7 = r6
        L_0x0187:
            org.xbill.DNS.Name r5 = name
            if (r5 != 0) goto L_0x018e
            usage()
        L_0x018e:
            if (r3 != 0) goto L_0x0196
            org.xbill.DNS.SimpleResolver r0 = new org.xbill.DNS.SimpleResolver
            r0.<init>()
            r3 = r0
        L_0x0196:
            org.xbill.DNS.Name r0 = name
            int r5 = type
            int r6 = dclass
            org.xbill.DNS.Record r0 = org.xbill.DNS.Record.newRecord(r0, r5, r6)
            org.xbill.DNS.Message r5 = org.xbill.DNS.Message.newQuery(r0)
            if (r4 == 0) goto L_0x01ab
            java.io.PrintStream r6 = java.lang.System.out
            r6.println(r5)
        L_0x01ab:
            long r8 = java.lang.System.currentTimeMillis()
            org.xbill.DNS.Message r6 = r3.send(r5)
            long r10 = java.lang.System.currentTimeMillis()
            int r12 = type
            r13 = 252(0xfc, float:3.53E-43)
            if (r12 != r13) goto L_0x01c1
            doAXFR(r6)
            goto L_0x01c7
        L_0x01c1:
            r12 = 0
            long r12 = r10 - r8
            doQuery(r6, r12)
        L_0x01c7:
            return
        */
        throw new UnsupportedOperationException("Method not decompiled: defpackage.dig.main(java.lang.String[]):void");
    }
}
