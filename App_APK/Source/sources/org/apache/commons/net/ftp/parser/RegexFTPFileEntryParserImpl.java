package org.apache.commons.net.ftp.parser;

import java.util.regex.MatchResult;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.regex.PatternSyntaxException;
import org.apache.commons.net.ftp.FTPFileEntryParserImpl;

public abstract class RegexFTPFileEntryParserImpl extends FTPFileEntryParserImpl {
    protected Matcher _matcher_ = null;
    private Pattern pattern = null;
    private MatchResult result = null;

    public RegexFTPFileEntryParserImpl(String regex) {
        setRegex(regex);
    }

    public boolean matches(String s) {
        this.result = null;
        this._matcher_ = this.pattern.matcher(s);
        if (this._matcher_.matches()) {
            this.result = this._matcher_.toMatchResult();
        }
        return this.result != null;
    }

    public int getGroupCnt() {
        if (this.result == null) {
            return 0;
        }
        return this.result.groupCount();
    }

    public String group(int matchnum) {
        if (this.result == null) {
            return null;
        }
        return this.result.group(matchnum);
    }

    public String getGroupsAsString() {
        StringBuffer b = new StringBuffer();
        for (int i = 1; i <= this.result.groupCount(); i++) {
            b.append(i);
            b.append(") ");
            b.append(this.result.group(i));
            b.append(System.getProperty("line.separator"));
        }
        return b.toString();
    }

    public boolean setRegex(String regex) {
        try {
            this.pattern = Pattern.compile(regex);
            return this.pattern != null;
        } catch (PatternSyntaxException e) {
            throw new IllegalArgumentException("Unparseable regex supplied: " + regex);
        }
    }
}
