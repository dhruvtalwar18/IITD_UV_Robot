package org.apache.commons.net.ftp.parser;

import java.text.ParseException;
import org.apache.commons.httpclient.cookie.CookieSpec;
import org.apache.commons.net.ftp.FTPClientConfig;
import org.apache.commons.net.ftp.FTPFile;

public class OS400FTPEntryParser extends ConfigurableFTPFileEntryParserImpl {
    private static final String DEFAULT_DATE_FORMAT = "yy/MM/dd HH:mm:ss";
    private static final String REGEX = "(\\S+)\\s+(\\d+)\\s+(\\S+)\\s+(\\S+)\\s+(\\*\\S+)\\s+(\\S+/?)\\s*";

    public OS400FTPEntryParser() {
        this((FTPClientConfig) null);
    }

    public OS400FTPEntryParser(FTPClientConfig config) {
        super(REGEX);
        configure(config);
    }

    public FTPFile parseFTPEntry(String entry) {
        FTPFile file = new FTPFile();
        file.setRawListing(entry);
        if (!matches(entry)) {
            return null;
        }
        String usr = group(1);
        String filesize = group(2);
        StringBuilder sb = new StringBuilder();
        int type = 3;
        sb.append(group(3));
        sb.append(" ");
        sb.append(group(4));
        String datestr = sb.toString();
        String typeStr = group(5);
        String name = group(6);
        try {
            file.setTimestamp(super.parseTimestamp(datestr));
        } catch (ParseException e) {
        }
        if (typeStr.equalsIgnoreCase("*STMF")) {
            type = 0;
        } else if (typeStr.equalsIgnoreCase("*DIR")) {
            type = 1;
        }
        file.setType(type);
        file.setUser(usr);
        try {
            file.setSize(Long.parseLong(filesize));
        } catch (NumberFormatException e2) {
        }
        if (name.endsWith(CookieSpec.PATH_DELIM)) {
            name = name.substring(0, name.length() - 1);
        }
        int pos = name.lastIndexOf(47);
        if (pos > -1) {
            name = name.substring(pos + 1);
        }
        file.setName(name);
        return file;
    }

    /* access modifiers changed from: protected */
    public FTPClientConfig getDefaultConfiguration() {
        return new FTPClientConfig(FTPClientConfig.SYST_OS400, DEFAULT_DATE_FORMAT, (String) null, (String) null, (String) null, (String) null);
    }
}
