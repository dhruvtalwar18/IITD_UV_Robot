package org.apache.commons.net.ftp.parser;

import java.text.ParseException;
import java.util.List;
import org.apache.commons.net.ftp.FTPClientConfig;
import org.apache.commons.net.ftp.FTPFile;

public class MVSFTPEntryParser extends ConfigurableFTPFileEntryParserImpl {
    static final String DEFAULT_DATE_FORMAT = "yyyy/MM/dd HH:mm";
    static final String FILE_LIST_REGEX = "\\S+\\s+\\S+\\s+\\S+\\s+\\S+\\s+\\S+\\s+[FV]\\S*\\s+\\S+\\s+\\S+\\s+(PS|PO|PO-E)\\s+(\\S+)\\s*";
    static final int FILE_LIST_TYPE = 0;
    static final String JES_LEVEL_1_LIST_REGEX = "(\\S+)\\s+(\\S+)\\s+(\\S+)\\s+(\\S+)\\s+(\\S+)\\s+(\\S+)\\s*";
    static final int JES_LEVEL_1_LIST_TYPE = 3;
    static final String JES_LEVEL_2_LIST_REGEX = "(\\S+)\\s+(\\S+)\\s+(\\S+)\\s+(\\S+)\\s+(\\S+)\\s+(\\S+).*";
    static final int JES_LEVEL_2_LIST_TYPE = 4;
    static final String MEMBER_LIST_REGEX = "(\\S+)\\s+\\S+\\s+\\S+\\s+(\\S+)\\s+(\\S+)\\s+\\S+\\s+\\S+\\s+\\S+\\s+\\S+\\s*";
    static final int MEMBER_LIST_TYPE = 1;
    static final int UNIX_LIST_TYPE = 2;
    static final int UNKNOWN_LIST_TYPE = -1;
    private int isType = -1;
    private UnixFTPEntryParser unixFTPEntryParser;

    public MVSFTPEntryParser() {
        super("");
        super.configure((FTPClientConfig) null);
    }

    public FTPFile parseFTPEntry(String entry) {
        boolean isParsed = false;
        FTPFile f = new FTPFile();
        if (this.isType == 0) {
            isParsed = parseFileList(f, entry);
        } else if (this.isType == 1) {
            isParsed = parseMemberList(f, entry);
            if (!isParsed) {
                isParsed = parseSimpleEntry(f, entry);
            }
        } else if (this.isType == 2) {
            isParsed = parseUnixList(f, entry);
        } else if (this.isType == 3) {
            isParsed = parseJeslevel1List(f, entry);
        } else if (this.isType == 4) {
            isParsed = parseJeslevel2List(f, entry);
        }
        if (!isParsed) {
            return null;
        }
        return f;
    }

    private boolean parseFileList(FTPFile file, String entry) {
        if (!matches(entry)) {
            return false;
        }
        file.setRawListing(entry);
        String name = group(2);
        String dsorg = group(1);
        file.setName(name);
        if ("PS".equals(dsorg)) {
            file.setType(0);
        } else if (!"PO".equals(dsorg) && !"PO-E".equals(dsorg)) {
            return false;
        } else {
            file.setType(1);
        }
        return true;
    }

    private boolean parseMemberList(FTPFile file, String entry) {
        if (!matches(entry)) {
            return false;
        }
        file.setRawListing(entry);
        String datestr = group(2) + " " + group(3);
        file.setName(group(1));
        file.setType(0);
        try {
            file.setTimestamp(super.parseTimestamp(datestr));
            return true;
        } catch (ParseException e) {
            e.printStackTrace();
            return false;
        }
    }

    private boolean parseSimpleEntry(FTPFile file, String entry) {
        if (entry == null || entry.length() <= 0) {
            return false;
        }
        file.setRawListing(entry);
        file.setName(entry.split(" ")[0]);
        file.setType(0);
        return true;
    }

    private boolean parseUnixList(FTPFile file, String entry) {
        if (this.unixFTPEntryParser.parseFTPEntry(entry) == null) {
            return false;
        }
        return true;
    }

    private boolean parseJeslevel1List(FTPFile file, String entry) {
        if (!matches(entry) || !group(3).equalsIgnoreCase("OUTPUT")) {
            return false;
        }
        file.setRawListing(entry);
        file.setName(group(2));
        file.setType(0);
        return true;
    }

    private boolean parseJeslevel2List(FTPFile file, String entry) {
        if (!matches(entry) || !group(4).equalsIgnoreCase("OUTPUT")) {
            return false;
        }
        file.setRawListing(entry);
        file.setName(group(2));
        file.setType(0);
        return true;
    }

    public List<String> preParse(List<String> orig) {
        if (orig != null && orig.size() > 0) {
            String header = orig.get(0);
            if (header.indexOf("Volume") >= 0 && header.indexOf("Dsname") >= 0) {
                setType(0);
                super.setRegex(FILE_LIST_REGEX);
            } else if (header.indexOf("Name") >= 0 && header.indexOf("Id") >= 0) {
                setType(1);
                super.setRegex(MEMBER_LIST_REGEX);
            } else if (header.indexOf("total") == 0) {
                setType(2);
                this.unixFTPEntryParser = new UnixFTPEntryParser();
            } else if (header.indexOf("Spool Files") >= 30) {
                setType(3);
                super.setRegex(JES_LEVEL_1_LIST_REGEX);
            } else if (header.indexOf("JOBNAME") != 0 || header.indexOf("JOBID") <= 8) {
                setType(-1);
            } else {
                setType(4);
                super.setRegex(JES_LEVEL_2_LIST_REGEX);
            }
            if (this.isType != 3) {
                orig.remove(0);
            }
        }
        return orig;
    }

    /* access modifiers changed from: package-private */
    public void setType(int type) {
        this.isType = type;
    }

    /* access modifiers changed from: protected */
    public FTPClientConfig getDefaultConfiguration() {
        return new FTPClientConfig(FTPClientConfig.SYST_MVS, DEFAULT_DATE_FORMAT, (String) null, (String) null, (String) null, (String) null);
    }
}
