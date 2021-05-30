package org.xbill.DNS;

import java.io.BufferedInputStream;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.ResourceBundle;
import java.util.StringTokenizer;
import org.apache.commons.httpclient.cookie.Cookie2;

public class ResolverConfig {
    static /* synthetic */ Class class$org$xbill$DNS$ResolverConfig;
    private static ResolverConfig currentConfig;
    private int ndots = -1;
    private Name[] searchlist = null;
    private String[] servers = null;

    static {
        refresh();
    }

    public ResolverConfig() {
        if (findProperty() || findSunJVM()) {
            return;
        }
        if (this.servers == null || this.searchlist == null) {
            String OS = System.getProperty("os.name");
            String vendor = System.getProperty("java.vendor");
            if (OS.indexOf("Windows") != -1) {
                if (OS.indexOf("95") == -1 && OS.indexOf("98") == -1 && OS.indexOf("ME") == -1) {
                    findNT();
                } else {
                    find95();
                }
            } else if (OS.indexOf("NetWare") != -1) {
                findNetware();
            } else if (vendor.indexOf("Android") != -1) {
                findAndroid();
            } else {
                findUnix();
            }
        }
    }

    private void addServer(String server, List list) {
        if (!list.contains(server)) {
            if (Options.check("verbose")) {
                PrintStream printStream = System.out;
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append("adding server ");
                stringBuffer.append(server);
                printStream.println(stringBuffer.toString());
            }
            list.add(server);
        }
    }

    private void addSearch(String search, List list) {
        if (Options.check("verbose")) {
            PrintStream printStream = System.out;
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("adding search ");
            stringBuffer.append(search);
            printStream.println(stringBuffer.toString());
        }
        try {
            Name name = Name.fromString(search, Name.root);
            if (!list.contains(name)) {
                list.add(name);
            }
        } catch (TextParseException e) {
        }
    }

    private int parseNdots(String token) {
        String token2 = token.substring(6);
        try {
            int ndots2 = Integer.parseInt(token2);
            if (ndots2 < 0) {
                return -1;
            }
            if (Options.check("verbose")) {
                PrintStream printStream = System.out;
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append("setting ndots ");
                stringBuffer.append(token2);
                printStream.println(stringBuffer.toString());
            }
            return ndots2;
        } catch (NumberFormatException e) {
            return -1;
        }
    }

    private void configureFromLists(List lserver, List lsearch) {
        if (this.servers == null && lserver.size() > 0) {
            this.servers = (String[]) lserver.toArray(new String[0]);
        }
        if (this.searchlist == null && lsearch.size() > 0) {
            this.searchlist = (Name[]) lsearch.toArray(new Name[0]);
        }
    }

    private void configureNdots(int lndots) {
        if (this.ndots < 0 && lndots > 0) {
            this.ndots = lndots;
        }
    }

    private boolean findProperty() {
        List lserver = new ArrayList(0);
        List lsearch = new ArrayList(0);
        String prop = System.getProperty("dns.server");
        if (prop != null) {
            StringTokenizer st = new StringTokenizer(prop, ",");
            while (st.hasMoreTokens()) {
                addServer(st.nextToken(), lserver);
            }
        }
        String prop2 = System.getProperty("dns.search");
        if (prop2 != null) {
            StringTokenizer st2 = new StringTokenizer(prop2, ",");
            while (st2.hasMoreTokens()) {
                addSearch(st2.nextToken(), lsearch);
            }
        }
        configureFromLists(lserver, lsearch);
        if (this.servers == null || this.searchlist == null) {
            return false;
        }
        return true;
    }

    private boolean findSunJVM() {
        List lserver = new ArrayList(0);
        List lsearch = new ArrayList(0);
        try {
            Class[] noClasses = new Class[0];
            Object[] noObjects = new Object[0];
            Class resConfClass = Class.forName("sun.net.dns.ResolverConfiguration");
            Object resConf = resConfClass.getDeclaredMethod("open", noClasses).invoke((Object) null, noObjects);
            List<String> lserver_tmp = (List) resConfClass.getMethod("nameservers", noClasses).invoke(resConf, noObjects);
            List<String> lsearch_tmp = (List) resConfClass.getMethod("searchlist", noClasses).invoke(resConf, noObjects);
            if (lserver_tmp.size() == 0) {
                return false;
            }
            if (lserver_tmp.size() > 0) {
                for (String addServer : lserver_tmp) {
                    addServer(addServer, lserver);
                }
            }
            if (lsearch_tmp.size() > 0) {
                for (String addSearch : lsearch_tmp) {
                    addSearch(addSearch, lsearch);
                }
            }
            configureFromLists(lserver, lsearch);
            return true;
        } catch (Exception e) {
            return false;
        }
    }

    private void findResolvConf(String file) {
        try {
            BufferedReader br = new BufferedReader(new InputStreamReader(new FileInputStream(file)));
            List lserver = new ArrayList(0);
            List lsearch = new ArrayList(0);
            int lndots = -1;
            while (true) {
                try {
                    String readLine = br.readLine();
                    String line = readLine;
                    if (readLine == null) {
                        break;
                    } else if (line.startsWith("nameserver")) {
                        StringTokenizer st = new StringTokenizer(line);
                        st.nextToken();
                        addServer(st.nextToken(), lserver);
                    } else if (line.startsWith(Cookie2.DOMAIN)) {
                        StringTokenizer st2 = new StringTokenizer(line);
                        st2.nextToken();
                        if (st2.hasMoreTokens()) {
                            if (lsearch.isEmpty()) {
                                addSearch(st2.nextToken(), lsearch);
                            }
                        }
                    } else if (line.startsWith("search")) {
                        if (!lsearch.isEmpty()) {
                            lsearch.clear();
                        }
                        StringTokenizer st3 = new StringTokenizer(line);
                        st3.nextToken();
                        while (st3.hasMoreTokens()) {
                            addSearch(st3.nextToken(), lsearch);
                        }
                    } else if (line.startsWith("options")) {
                        StringTokenizer st4 = new StringTokenizer(line);
                        st4.nextToken();
                        while (st4.hasMoreTokens()) {
                            String token = st4.nextToken();
                            if (token.startsWith("ndots:")) {
                                lndots = parseNdots(token);
                            }
                        }
                    }
                } catch (IOException e) {
                }
            }
            br.close();
            configureFromLists(lserver, lsearch);
            configureNdots(lndots);
        } catch (FileNotFoundException e2) {
        }
    }

    private void findUnix() {
        findResolvConf("/etc/resolv.conf");
    }

    private void findNetware() {
        findResolvConf("sys:/etc/resolv.cfg");
    }

    static /* synthetic */ Class class$(String x0) {
        try {
            return Class.forName(x0);
        } catch (ClassNotFoundException x1) {
            throw new NoClassDefFoundError().initCause(x1);
        }
    }

    private void findWin(InputStream in, Locale locale) {
        Class cls;
        ResourceBundle res;
        String packageName;
        boolean z;
        Locale locale2 = locale;
        if (class$org$xbill$DNS$ResolverConfig == null) {
            cls = class$("org.xbill.DNS.ResolverConfig");
            class$org$xbill$DNS$ResolverConfig = cls;
        } else {
            cls = class$org$xbill$DNS$ResolverConfig;
        }
        String packageName2 = cls.getPackage().getName();
        StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append(packageName2);
        stringBuffer.append(".windows.DNSServer");
        String resPackageName = stringBuffer.toString();
        if (locale2 != null) {
            res = ResourceBundle.getBundle(resPackageName, locale2);
        } else {
            res = ResourceBundle.getBundle(resPackageName);
        }
        ResourceBundle res2 = res;
        String host_name = res2.getString("host_name");
        String primary_dns_suffix = res2.getString("primary_dns_suffix");
        String dns_suffix = res2.getString("dns_suffix");
        String dns_servers = res2.getString("dns_servers");
        BufferedReader br = new BufferedReader(new InputStreamReader(in));
        try {
            List lserver = new ArrayList();
            List lsearch = new ArrayList();
            boolean readingServers = false;
            boolean readingSearches = false;
            while (true) {
                String readLine = br.readLine();
                String line = readLine;
                if (readLine != null) {
                    String line2 = line;
                    StringTokenizer st = new StringTokenizer(line2);
                    if (!st.hasMoreTokens()) {
                        readingServers = false;
                        readingSearches = false;
                        String str = line2;
                    } else {
                        String s = st.nextToken();
                        if (line2.indexOf(":") != -1) {
                            readingSearches = false;
                            readingServers = false;
                        }
                        if (line2.indexOf(host_name) != -1) {
                            String s2 = s;
                            while (st.hasMoreTokens()) {
                                try {
                                    s2 = st.nextToken();
                                } catch (IOException e) {
                                    String str2 = packageName2;
                                    return;
                                }
                            }
                            packageName = packageName2;
                            try {
                                try {
                                    if (Name.fromString(s2, (Name) null).labels() != 1) {
                                        addSearch(s2, lsearch);
                                    }
                                } catch (IOException e2) {
                                    return;
                                }
                            } catch (TextParseException e3) {
                                TextParseException textParseException = e3;
                            }
                        } else {
                            packageName = packageName2;
                            if (line2.indexOf(primary_dns_suffix) != -1) {
                                String s3 = s;
                                while (st.hasMoreTokens()) {
                                    s3 = st.nextToken();
                                }
                                if (!s3.equals(":")) {
                                    addSearch(s3, lsearch);
                                    z = true;
                                }
                            } else {
                                if (!readingSearches) {
                                    if (line2.indexOf(dns_suffix) == -1) {
                                        if (readingServers || line2.indexOf(dns_servers) != -1) {
                                            String s4 = s;
                                            while (st.hasMoreTokens()) {
                                                s4 = st.nextToken();
                                            }
                                            if (!s4.equals(":")) {
                                                addServer(s4, lserver);
                                                readingServers = true;
                                            }
                                        }
                                    }
                                }
                                String s5 = s;
                                while (st.hasMoreTokens()) {
                                    s5 = st.nextToken();
                                }
                                if (!s5.equals(":")) {
                                    addSearch(s5, lsearch);
                                    z = true;
                                }
                            }
                            readingSearches = z;
                        }
                        packageName2 = packageName;
                        Locale locale3 = locale;
                    }
                } else {
                    String str3 = line;
                    configureFromLists(lserver, lsearch);
                    return;
                }
            }
        } catch (IOException e4) {
            String str4 = packageName2;
        }
    }

    private void findWin(InputStream in) {
        int bufSize = Integer.getInteger("org.xbill.DNS.windows.parse.buffer", 8192).intValue();
        BufferedInputStream b = new BufferedInputStream(in, bufSize);
        b.mark(bufSize);
        findWin(b, (Locale) null);
        if (this.servers == null) {
            try {
                b.reset();
                findWin(b, new Locale("", ""));
            } catch (IOException e) {
            }
        }
    }

    private void find95() {
        try {
            Runtime runtime = Runtime.getRuntime();
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("winipcfg /all /batch ");
            stringBuffer.append("winipcfg.out");
            runtime.exec(stringBuffer.toString()).waitFor();
            findWin(new FileInputStream(new File("winipcfg.out")));
            new File("winipcfg.out").delete();
        } catch (Exception e) {
        }
    }

    private void findNT() {
        try {
            Process p = Runtime.getRuntime().exec("ipconfig /all");
            findWin(p.getInputStream());
            p.destroy();
        } catch (Exception e) {
        }
    }

    private void findAndroid() {
        try {
            ArrayList lserver = new ArrayList();
            ArrayList lsearch = new ArrayList();
            BufferedReader br = new BufferedReader(new InputStreamReader(Runtime.getRuntime().exec("getprop").getInputStream()));
            while (true) {
                String readLine = br.readLine();
                String line = readLine;
                if (readLine != null) {
                    StringTokenizer t = new StringTokenizer(line, ":");
                    if (t.nextToken().indexOf("net.dns") > -1) {
                        String v = t.nextToken().replaceAll("[ \\[\\]]", "");
                        if ((v.matches("^\\d+(\\.\\d+){3}$") || v.matches("^[0-9a-f]+(:[0-9a-f]*)+:[0-9a-f]+$")) && !lserver.contains(v)) {
                            lserver.add(v);
                        }
                    }
                } else {
                    configureFromLists(lserver, lsearch);
                    return;
                }
            }
        } catch (Exception e) {
        }
    }

    public String[] servers() {
        return this.servers;
    }

    public String server() {
        if (this.servers == null) {
            return null;
        }
        return this.servers[0];
    }

    public Name[] searchPath() {
        return this.searchlist;
    }

    public int ndots() {
        if (this.ndots < 0) {
            return 1;
        }
        return this.ndots;
    }

    public static synchronized ResolverConfig getCurrentConfig() {
        ResolverConfig resolverConfig;
        synchronized (ResolverConfig.class) {
            resolverConfig = currentConfig;
        }
        return resolverConfig;
    }

    public static void refresh() {
        Class cls;
        ResolverConfig newConfig = new ResolverConfig();
        if (class$org$xbill$DNS$ResolverConfig == null) {
            cls = class$("org.xbill.DNS.ResolverConfig");
            class$org$xbill$DNS$ResolverConfig = cls;
        } else {
            cls = class$org$xbill$DNS$ResolverConfig;
        }
        synchronized (cls) {
            currentConfig = newConfig;
        }
    }
}
