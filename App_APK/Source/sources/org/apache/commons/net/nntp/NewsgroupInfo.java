package org.apache.commons.net.nntp;

public final class NewsgroupInfo {
    public static final int MODERATED_POSTING_PERMISSION = 1;
    public static final int PERMITTED_POSTING_PERMISSION = 2;
    public static final int PROHIBITED_POSTING_PERMISSION = 3;
    public static final int UNKNOWN_POSTING_PERMISSION = 0;
    private int __estimatedArticleCount;
    private int __firstArticle;
    private int __lastArticle;
    private String __newsgroup;
    private int __postingPermission;

    /* access modifiers changed from: package-private */
    public void _setNewsgroup(String newsgroup) {
        this.__newsgroup = newsgroup;
    }

    /* access modifiers changed from: package-private */
    public void _setArticleCount(int count) {
        this.__estimatedArticleCount = count;
    }

    /* access modifiers changed from: package-private */
    public void _setFirstArticle(int first) {
        this.__firstArticle = first;
    }

    /* access modifiers changed from: package-private */
    public void _setLastArticle(int last) {
        this.__lastArticle = last;
    }

    /* access modifiers changed from: package-private */
    public void _setPostingPermission(int permission) {
        this.__postingPermission = permission;
    }

    public String getNewsgroup() {
        return this.__newsgroup;
    }

    public int getArticleCount() {
        return this.__estimatedArticleCount;
    }

    public int getFirstArticle() {
        return this.__firstArticle;
    }

    public int getLastArticle() {
        return this.__lastArticle;
    }

    public int getPostingPermission() {
        return this.__postingPermission;
    }
}
