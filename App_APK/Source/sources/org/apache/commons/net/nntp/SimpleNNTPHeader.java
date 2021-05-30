package org.apache.commons.net.nntp;

public class SimpleNNTPHeader {
    private String __from;
    private StringBuilder __headerFields = new StringBuilder();
    private int __newsgroupCount = 0;
    private StringBuilder __newsgroups = new StringBuilder();
    private String __subject;

    public SimpleNNTPHeader(String from, String subject) {
        this.__from = from;
        this.__subject = subject;
    }

    public void addNewsgroup(String newsgroup) {
        int i = this.__newsgroupCount;
        this.__newsgroupCount = i + 1;
        if (i > 0) {
            this.__newsgroups.append(',');
        }
        this.__newsgroups.append(newsgroup);
    }

    public void addHeaderField(String headerField, String value) {
        this.__headerFields.append(headerField);
        this.__headerFields.append(": ");
        this.__headerFields.append(value);
        this.__headerFields.append(10);
    }

    public String getFromAddress() {
        return this.__from;
    }

    public String getSubject() {
        return this.__subject;
    }

    public String getNewsgroups() {
        return this.__newsgroups.toString();
    }

    public String toString() {
        StringBuffer header = new StringBuffer();
        header.append("From: ");
        header.append(this.__from);
        header.append("\nNewsgroups: ");
        header.append(this.__newsgroups.toString());
        header.append("\nSubject: ");
        header.append(this.__subject);
        header.append(10);
        if (this.__headerFields.length() > 0) {
            header.append(this.__headerFields.toString());
        }
        header.append(10);
        return header.toString();
    }
}
