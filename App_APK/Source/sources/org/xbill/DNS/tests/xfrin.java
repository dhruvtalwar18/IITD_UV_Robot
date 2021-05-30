package org.xbill.DNS.tests;

import java.io.PrintStream;
import java.util.Iterator;
import java.util.List;
import org.xbill.DNS.Lookup;
import org.xbill.DNS.Name;
import org.xbill.DNS.Record;
import org.xbill.DNS.TSIG;
import org.xbill.DNS.ZoneTransferIn;

public class xfrin {
    private static void usage(String s) {
        PrintStream printStream = System.out;
        StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append("Error: ");
        stringBuffer.append(s);
        printStream.println(stringBuffer.toString());
        System.out.println("usage: xfrin [-i serial] [-k keyname/secret] [-s server] [-p port] [-f] zone");
        System.exit(1);
    }

    public static void main(String[] args) throws Exception {
        ZoneTransferIn xfrin;
        int ixfr_serial = -1;
        boolean fallback = false;
        int port = 53;
        String server = null;
        TSIG key = null;
        int arg = 0;
        while (arg < args.length) {
            if (!args[arg].equals("-i")) {
                if (!args[arg].equals("-k")) {
                    if (!args[arg].equals("-s")) {
                        if (!args[arg].equals("-p")) {
                            if (!args[arg].equals("-f")) {
                                if (!args[arg].startsWith("-")) {
                                    break;
                                }
                                usage("invalid option");
                            } else {
                                fallback = true;
                            }
                        } else {
                            arg++;
                            port = Integer.parseInt(args[arg]);
                            if (port < 0 || port > 65535) {
                                usage("invalid port");
                            }
                        }
                    } else {
                        arg++;
                        server = args[arg];
                    }
                } else {
                    arg++;
                    String s = args[arg];
                    int index = s.indexOf(47);
                    if (index < 0) {
                        usage("invalid key");
                    }
                    key = new TSIG(s.substring(0, index), s.substring(index + 1));
                }
            } else {
                arg++;
                ixfr_serial = Integer.parseInt(args[arg]);
                if (ixfr_serial < 0) {
                    usage("invalid serial number");
                }
            }
            arg++;
        }
        if (arg >= args.length) {
            usage("no zone name specified");
        }
        Name zname = Name.fromString(args[arg]);
        if (server == null) {
            Lookup l = new Lookup(zname, 2);
            Record[] ns = l.run();
            if (ns == null) {
                PrintStream printStream = System.out;
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append("failed to look up NS record: ");
                stringBuffer.append(l.getErrorString());
                printStream.println(stringBuffer.toString());
                System.exit(1);
            }
            server = ns[0].rdataToString();
            PrintStream printStream2 = System.out;
            StringBuffer stringBuffer2 = new StringBuffer();
            stringBuffer2.append("sending to server '");
            stringBuffer2.append(server);
            stringBuffer2.append("'");
            printStream2.println(stringBuffer2.toString());
        }
        if (ixfr_serial >= 0) {
            xfrin = ZoneTransferIn.newIXFR(zname, (long) ixfr_serial, fallback, server, port, key);
        } else {
            xfrin = ZoneTransferIn.newAXFR(zname, server, port, key);
        }
        List<Object> response = xfrin.run();
        if (xfrin.isAXFR()) {
            if (ixfr_serial >= 0) {
                System.out.println("AXFR-like IXFR response");
            } else {
                System.out.println("AXFR response");
            }
            for (Object println : response) {
                System.out.println(println);
            }
        } else if (xfrin.isIXFR()) {
            System.out.println("IXFR response");
            Iterator it = response.iterator();
            while (it.hasNext()) {
                ZoneTransferIn.Delta delta = (ZoneTransferIn.Delta) it.next();
                PrintStream printStream3 = System.out;
                StringBuffer stringBuffer3 = new StringBuffer();
                stringBuffer3.append("delta from ");
                stringBuffer3.append(delta.start);
                stringBuffer3.append(" to ");
                stringBuffer3.append(delta.end);
                printStream3.println(stringBuffer3.toString());
                System.out.println("deletes");
                for (Object println2 : delta.deletes) {
                    System.out.println(println2);
                }
                System.out.println("adds");
                for (Object println3 : delta.adds) {
                    System.out.println(println3);
                }
            }
        } else if (xfrin.isCurrent()) {
            System.out.println("up to date");
        }
    }
}
