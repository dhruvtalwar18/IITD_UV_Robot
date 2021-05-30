package org.xbill.DNS.tests;

import java.io.PrintStream;
import java.util.Iterator;
import org.xbill.DNS.Name;
import org.xbill.DNS.Zone;

public class primary {
    private static void usage() {
        System.out.println("usage: primary [-t] [-a | -i] origin file");
        System.exit(1);
    }

    public static void main(String[] args) throws Exception {
        boolean time = false;
        boolean axfr = false;
        boolean iterator = false;
        int arg = 0;
        if (args.length < 2) {
            usage();
        }
        while (args.length - arg > 2) {
            if (args[0].equals("-t")) {
                time = true;
            } else if (args[0].equals("-a")) {
                axfr = true;
            } else if (args[0].equals("-i")) {
                iterator = true;
            }
            arg++;
        }
        int arg2 = arg + 1;
        Name origin = Name.fromString(args[arg], Name.root);
        int i = arg2 + 1;
        String file = args[arg2];
        long start = System.currentTimeMillis();
        Zone zone = new Zone(origin, file);
        long end = System.currentTimeMillis();
        if (axfr) {
            Iterator it = zone.AXFR();
            while (it.hasNext()) {
                System.out.println(it.next());
            }
        } else if (iterator) {
            Iterator it2 = zone.iterator();
            while (it2.hasNext()) {
                System.out.println(it2.next());
            }
        } else {
            System.out.println(zone);
        }
        if (time) {
            PrintStream printStream = System.out;
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("; Load time: ");
            stringBuffer.append(end - start);
            stringBuffer.append(" ms");
            printStream.println(stringBuffer.toString());
        }
    }
}
