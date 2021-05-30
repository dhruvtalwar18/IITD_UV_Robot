package org.apache.commons.net.ftp.parser;

import java.text.ParseException;
import org.apache.commons.net.ftp.FTPClientConfig;
import org.apache.commons.net.ftp.FTPFile;

public class OS2FTPEntryParser extends ConfigurableFTPFileEntryParserImpl {
    private static final String DEFAULT_DATE_FORMAT = "MM-dd-yy HH:mm";
    private static final String REGEX = "\\s*([0-9]+)\\s*(\\s+|[A-Z]+)\\s*(DIR|\\s+)\\s*(\\S+)\\s+(\\S+)\\s+(\\S.*)";

    public OS2FTPEntryParser() {
        this((FTPClientConfig) null);
    }

    public OS2FTPEntryParser(FTPClientConfig config) {
        super(REGEX);
        configure(config);
    }

    public FTPFile parseFTPEntry(String entry) {
        FTPFile f = new FTPFile();
        if (!matches(entry)) {
            return null;
        }
        String size = group(1);
        String attrib = group(2);
        String dirString = group(3);
        String datestr = group(4) + " " + group(5);
        String name = group(6);
        try {
            f.setTimestamp(super.parseTimestamp(datestr));
        } catch (ParseException e) {
        }
        if (dirString.trim().equals("DIR") || attrib.trim().equals("DIR")) {
            f.setType(1);
        } else {
            f.setType(0);
        }
        f.setName(name.trim());
        f.setSize(Long.parseLong(size.trim()));
        return f;
    }

    /* access modifiers changed from: protected */
    public FTPClientConfig getDefaultConfiguration() {
        return new FTPClientConfig(FTPClientConfig.SYST_OS2, DEFAULT_DATE_FORMAT, (String) null, (String) null, (String) null, (String) null);
    }
}
