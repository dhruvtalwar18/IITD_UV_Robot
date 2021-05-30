package org.apache.commons.net.nntp;

/* compiled from: Threader */
class ThreadContainer {
    ThreadContainer child;
    ThreadContainer next;
    ThreadContainer parent;
    ThreadContainer prev;
    Threadable threadable;

    ThreadContainer() {
    }

    /* access modifiers changed from: package-private */
    public boolean findChild(ThreadContainer target) {
        if (this.child == null) {
            return false;
        }
        if (this.child == target) {
            return true;
        }
        return this.child.findChild(target);
    }

    /* access modifiers changed from: package-private */
    public void flush() {
        if (this.parent == null || this.threadable != null) {
            this.parent = null;
            if (this.threadable != null) {
                this.threadable.setChild(this.child == null ? null : this.child.threadable);
            }
            if (this.child != null) {
                this.child.flush();
                this.child = null;
            }
            if (this.threadable != null) {
                this.threadable.setNext(this.next == null ? null : this.next.threadable);
            }
            if (this.next != null) {
                this.next.flush();
                this.next = null;
            }
            this.threadable = null;
            return;
        }
        throw new RuntimeException("no threadable in " + toString());
    }

    /* access modifiers changed from: package-private */
    public void reverseChildren() {
        if (this.child != null) {
            ThreadContainer prev2 = null;
            ThreadContainer kid = this.child;
            ThreadContainer rest = kid.next;
            while (kid != null) {
                kid.next = prev2;
                prev2 = kid;
                kid = rest;
                rest = rest == null ? null : rest.next;
            }
            this.child = prev2;
            for (ThreadContainer kid2 = this.child; kid2 != null; kid2 = kid2.next) {
                kid2.reverseChildren();
            }
        }
    }
}
