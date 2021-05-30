package org.apache.commons.net.nntp;

import java.util.Calendar;

public final class NewGroupsOrNewsQuery {
    private String __date;
    private StringBuffer __distributions = null;
    private boolean __isGMT;
    private StringBuffer __newsgroups = null;
    private String __time;

    public NewGroupsOrNewsQuery(Calendar date, boolean gmt) {
        this.__isGMT = gmt;
        StringBuffer buffer = new StringBuffer();
        String str = Integer.toString(date.get(1));
        int num = str.length();
        if (num >= 2) {
            buffer.append(str.substring(num - 2));
        } else {
            buffer.append("00");
        }
        String str2 = Integer.toString(date.get(2) + 1);
        int num2 = str2.length();
        if (num2 == 1) {
            buffer.append('0');
            buffer.append(str2);
        } else if (num2 == 2) {
            buffer.append(str2);
        } else {
            buffer.append("01");
        }
        String str3 = Integer.toString(date.get(5));
        int num3 = str3.length();
        if (num3 == 1) {
            buffer.append('0');
            buffer.append(str3);
        } else if (num3 == 2) {
            buffer.append(str3);
        } else {
            buffer.append("01");
        }
        this.__date = buffer.toString();
        buffer.setLength(0);
        String str4 = Integer.toString(date.get(11));
        int num4 = str4.length();
        if (num4 == 1) {
            buffer.append('0');
            buffer.append(str4);
        } else if (num4 == 2) {
            buffer.append(str4);
        } else {
            buffer.append("00");
        }
        String str5 = Integer.toString(date.get(12));
        int num5 = str5.length();
        if (num5 == 1) {
            buffer.append('0');
            buffer.append(str5);
        } else if (num5 == 2) {
            buffer.append(str5);
        } else {
            buffer.append("00");
        }
        String str6 = Integer.toString(date.get(13));
        int num6 = str6.length();
        if (num6 == 1) {
            buffer.append('0');
            buffer.append(str6);
        } else if (num6 == 2) {
            buffer.append(str6);
        } else {
            buffer.append("00");
        }
        this.__time = buffer.toString();
    }

    public void addNewsgroup(String newsgroup) {
        if (this.__newsgroups != null) {
            this.__newsgroups.append(',');
        } else {
            this.__newsgroups = new StringBuffer();
        }
        this.__newsgroups.append(newsgroup);
    }

    public void omitNewsgroup(String newsgroup) {
        addNewsgroup("!" + newsgroup);
    }

    public void addDistribution(String distribution) {
        if (this.__distributions != null) {
            this.__distributions.append(',');
        } else {
            this.__distributions = new StringBuffer();
        }
        this.__distributions.append(distribution);
    }

    public String getDate() {
        return this.__date;
    }

    public String getTime() {
        return this.__time;
    }

    public boolean isGMT() {
        return this.__isGMT;
    }

    public String getDistributions() {
        if (this.__distributions == null) {
            return null;
        }
        return this.__distributions.toString();
    }

    public String getNewsgroups() {
        if (this.__newsgroups == null) {
            return null;
        }
        return this.__newsgroups.toString();
    }
}
