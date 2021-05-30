package org.apache.commons.net.ftp.parser;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.text.ParseException;
import java.util.StringTokenizer;
import org.apache.commons.net.ftp.FTPClientConfig;
import org.apache.commons.net.ftp.FTPFile;
import org.apache.commons.net.ftp.FTPListParseEngine;

public class VMSFTPEntryParser extends ConfigurableFTPFileEntryParserImpl {
    private static final String DEFAULT_DATE_FORMAT = "d-MMM-yyyy HH:mm:ss";
    private static final String REGEX = "(.*;[0-9]+)\\s*(\\d+)/\\d+\\s*(\\S+)\\s+(\\S+)\\s+\\[(([0-9$A-Za-z_]+)|([0-9$A-Za-z_]+),([0-9$a-zA-Z_]+))\\]?\\s*\\([a-zA-Z]*,([a-zA-Z]*),([a-zA-Z]*),([a-zA-Z]*)\\)";

    public VMSFTPEntryParser() {
        this((FTPClientConfig) null);
    }

    public VMSFTPEntryParser(FTPClientConfig config) {
        super(REGEX);
        configure(config);
    }

    public FTPFile[] parseFileList(InputStream listStream) throws IOException {
        FTPListParseEngine engine = new FTPListParseEngine(this);
        engine.readServerList(listStream);
        return engine.getFiles();
    }

    public FTPFile parseFTPEntry(String entry) {
        String user;
        String grp;
        int i;
        if (matches(entry)) {
            FTPFile f = new FTPFile();
            f.setRawListing(entry);
            String name = group(1);
            String size = group(2);
            String datestr = group(3) + " " + group(4);
            String owner = group(5);
            String[] permissions = {group(9), group(10), group(11)};
            try {
                f.setTimestamp(super.parseTimestamp(datestr));
            } catch (ParseException e) {
                ParseException parseException = e;
            }
            StringTokenizer t = new StringTokenizer(owner, ",");
            switch (t.countTokens()) {
                case 1:
                    grp = null;
                    user = t.nextToken();
                    break;
                case 2:
                    grp = t.nextToken();
                    user = t.nextToken();
                    break;
                default:
                    grp = null;
                    user = null;
                    break;
            }
            if (name.lastIndexOf(".DIR") != -1) {
                f.setType(1);
                i = 0;
            } else {
                i = 0;
                f.setType(0);
            }
            if (isVersioning()) {
                f.setName(name);
            } else {
                name = name.substring(i, name.lastIndexOf(";"));
                f.setName(name);
            }
            String str = name;
            f.setSize(Long.parseLong(size) * 512);
            f.setGroup(grp);
            f.setUser(user);
            int access = 0;
            while (access < 3) {
                String permission = permissions[access];
                StringTokenizer t2 = t;
                f.setPermission(access, 0, permission.indexOf(82) >= 0);
                f.setPermission(access, 1, permission.indexOf(87) >= 0);
                f.setPermission(access, 2, permission.indexOf(69) >= 0);
                access++;
                t = t2;
            }
            return f;
        }
        String str2 = entry;
        return null;
    }

    public String readNextEntry(BufferedReader reader) throws IOException {
        String line = reader.readLine();
        StringBuffer entry = new StringBuffer();
        while (line != null) {
            if (line.startsWith("Directory") || line.startsWith("Total")) {
                line = reader.readLine();
            } else {
                entry.append(line);
                if (line.trim().endsWith(")")) {
                    break;
                }
                line = reader.readLine();
            }
        }
        if (entry.length() == 0) {
            return null;
        }
        return entry.toString();
    }

    /* access modifiers changed from: protected */
    public boolean isVersioning() {
        return false;
    }

    /* access modifiers changed from: protected */
    public FTPClientConfig getDefaultConfiguration() {
        return new FTPClientConfig(FTPClientConfig.SYST_VMS, DEFAULT_DATE_FORMAT, (String) null, (String) null, (String) null, (String) null);
    }
}
