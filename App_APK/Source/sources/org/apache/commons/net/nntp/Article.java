package org.apache.commons.net.nntp;

import java.io.PrintStream;
import java.util.ArrayList;
import java.util.StringTokenizer;

public class Article implements Threadable {
    private String articleId;
    private int articleNumber;
    private String date;
    private String from;
    private StringBuffer header = new StringBuffer();
    private boolean isReply = false;
    public Article kid;
    public Article next;
    private StringBuffer references;
    private String simplifiedSubject;
    private String subject;

    public void addHeaderField(String name, String val) {
        this.header.append(name);
        this.header.append(": ");
        this.header.append(val);
        this.header.append(10);
    }

    public void addReference(String msgId) {
        if (this.references == null) {
            this.references = new StringBuffer();
            this.references.append("References: ");
        }
        this.references.append(msgId);
        this.references.append("\t");
    }

    public String[] getReferences() {
        if (this.references == null) {
            return new String[0];
        }
        ArrayList<String> list = new ArrayList<>();
        StringTokenizer st = new StringTokenizer(this.references.substring(this.references.toString().indexOf(58)), "\t");
        while (st.hasMoreTokens()) {
            list.add(st.nextToken());
        }
        return (String[]) list.toArray(new String[list.size()]);
    }

    private void simplifySubject() {
        int start = 0;
        String subject2 = getSubject();
        int len = subject2.length();
        boolean done = false;
        while (!done) {
            done = true;
            while (start < len && subject2.charAt(start) == ' ') {
                start++;
            }
            if (start < len - 2 && ((subject2.charAt(start) == 'r' || subject2.charAt(start) == 'R') && (subject2.charAt(start + 1) == 'e' || subject2.charAt(start + 1) == 'E'))) {
                if (subject2.charAt(start + 2) == ':') {
                    start += 3;
                    this.isReply = true;
                    done = false;
                } else if (start < len - 2 && (subject2.charAt(start + 2) == '[' || subject2.charAt(start + 2) == '(')) {
                    int i = start + 3;
                    while (i < len && subject2.charAt(i) >= '0' && subject2.charAt(i) <= '9') {
                        i++;
                    }
                    if (i < len - 1 && ((subject2.charAt(i) == ']' || subject2.charAt(i) == ')') && subject2.charAt(i + 1) == ':')) {
                        start = i + 2;
                        this.isReply = true;
                        done = false;
                    }
                }
            }
            if (this.simplifiedSubject == "(no subject)") {
                this.simplifiedSubject = "";
            }
            int end = len;
            while (end > start && subject2.charAt(end - 1) < ' ') {
                end--;
            }
            if (start == 0 && end == len) {
                this.simplifiedSubject = subject2;
            } else {
                this.simplifiedSubject = subject2.substring(start, end);
            }
        }
    }

    public static void printThread(Article article, int depth) {
        for (int i = 0; i < depth; i++) {
            System.out.print("==>");
        }
        PrintStream printStream = System.out;
        printStream.println(article.getSubject() + "\t" + article.getFrom());
        if (article.kid != null) {
            printThread(article.kid, depth + 1);
        }
        if (article.next != null) {
            printThread(article.next, depth);
        }
    }

    public String getArticleId() {
        return this.articleId;
    }

    public int getArticleNumber() {
        return this.articleNumber;
    }

    public String getDate() {
        return this.date;
    }

    public String getFrom() {
        return this.from;
    }

    public String getSubject() {
        return this.subject;
    }

    public void setArticleId(String string) {
        this.articleId = string;
    }

    public void setArticleNumber(int i) {
        this.articleNumber = i;
    }

    public void setDate(String string) {
        this.date = string;
    }

    public void setFrom(String string) {
        this.from = string;
    }

    public void setSubject(String string) {
        this.subject = string;
    }

    public boolean isDummy() {
        return getSubject() == null;
    }

    public String messageThreadId() {
        return this.articleId;
    }

    public String[] messageThreadReferences() {
        return getReferences();
    }

    public String simplifiedSubject() {
        if (this.simplifiedSubject == null) {
            simplifySubject();
        }
        return this.simplifiedSubject;
    }

    public boolean subjectIsReply() {
        if (this.simplifiedSubject == null) {
            simplifySubject();
        }
        return this.isReply;
    }

    public void setChild(Threadable child) {
        this.kid = (Article) child;
        flushSubjectCache();
    }

    private void flushSubjectCache() {
        this.simplifiedSubject = null;
    }

    public void setNext(Threadable next2) {
        this.next = (Article) next2;
        flushSubjectCache();
    }

    public Threadable makeDummy() {
        return new Article();
    }
}
