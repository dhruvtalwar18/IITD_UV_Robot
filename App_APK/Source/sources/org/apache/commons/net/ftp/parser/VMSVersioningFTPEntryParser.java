package org.apache.commons.net.ftp.parser;

import java.util.HashMap;
import java.util.List;
import java.util.ListIterator;
import java.util.regex.MatchResult;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.regex.PatternSyntaxException;
import org.apache.commons.net.ftp.FTPClientConfig;

public class VMSVersioningFTPEntryParser extends VMSFTPEntryParser {
    private static final String PRE_PARSE_REGEX = "(.*);([0-9]+)\\s*.*";
    private Matcher _preparse_matcher_;
    private Pattern _preparse_pattern_;

    public VMSVersioningFTPEntryParser() {
        this((FTPClientConfig) null);
    }

    public VMSVersioningFTPEntryParser(FTPClientConfig config) {
        configure(config);
        try {
            this._preparse_pattern_ = Pattern.compile(PRE_PARSE_REGEX);
        } catch (PatternSyntaxException e) {
            throw new IllegalArgumentException("Unparseable regex supplied:  (.*);([0-9]+)\\s*.*");
        }
    }

    private static class NameVersion {
        String name;
        int versionNumber;

        NameVersion(String name2, String vers) {
            this.name = name2;
            this.versionNumber = Integer.parseInt(vers);
        }
    }

    public List<String> preParse(List<String> original) {
        List<String> original2 = super.preParse(original);
        HashMap<String, NameVersion> existingEntries = new HashMap<>();
        ListIterator<String> iter = original2.listIterator();
        while (iter.hasNext()) {
            this._preparse_matcher_ = this._preparse_pattern_.matcher(iter.next().trim());
            if (this._preparse_matcher_.matches()) {
                MatchResult result = this._preparse_matcher_.toMatchResult();
                String name = result.group(1);
                NameVersion nv = new NameVersion(name, result.group(2));
                NameVersion existing = existingEntries.get(name);
                if (existing == null || nv.versionNumber >= existing.versionNumber) {
                    existingEntries.put(name, nv);
                } else {
                    iter.remove();
                }
            }
        }
        while (iter.hasPrevious()) {
            this._preparse_matcher_ = this._preparse_pattern_.matcher(iter.previous().trim());
            if (this._preparse_matcher_.matches()) {
                MatchResult result2 = this._preparse_matcher_.toMatchResult();
                String name2 = result2.group(1);
                NameVersion nv2 = new NameVersion(name2, result2.group(2));
                NameVersion existing2 = existingEntries.get(name2);
                if (existing2 != null && nv2.versionNumber < existing2.versionNumber) {
                    iter.remove();
                }
            }
        }
        return original2;
    }

    /* access modifiers changed from: protected */
    public boolean isVersioning() {
        return true;
    }
}
