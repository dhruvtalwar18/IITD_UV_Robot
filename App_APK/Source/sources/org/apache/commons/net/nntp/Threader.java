package org.apache.commons.net.nntp;

import java.util.HashMap;

public class Threader {
    private int bogusIdCount = 0;
    private HashMap<String, ThreadContainer> idTable;
    private ThreadContainer root;

    public Threadable thread(Threadable[] messages) {
        if (messages == null) {
            return null;
        }
        this.idTable = new HashMap<>();
        for (int i = 0; i < messages.length; i++) {
            if (!messages[i].isDummy()) {
                buildContainer(messages[i]);
            }
        }
        this.root = findRootSet();
        this.idTable.clear();
        this.idTable = null;
        pruneEmptyContainers(this.root);
        this.root.reverseChildren();
        gatherSubjects();
        if (this.root.next == null) {
            for (ThreadContainer r = this.root.child; r != null; r = r.next) {
                if (r.threadable == null) {
                    r.threadable = r.child.threadable.makeDummy();
                }
            }
            Threadable result = this.root.child == null ? null : this.root.child.threadable;
            this.root.flush();
            this.root = null;
            return result;
        }
        throw new RuntimeException("root node has a next:" + this.root);
    }

    private void buildContainer(Threadable threadable) {
        String id = threadable.messageThreadId();
        ThreadContainer container = this.idTable.get(id);
        if (container != null) {
            if (container.threadable != null) {
                StringBuilder sb = new StringBuilder();
                sb.append("<Bogus-id:");
                int i = this.bogusIdCount;
                this.bogusIdCount = i + 1;
                sb.append(i);
                sb.append(">");
                id = sb.toString();
                container = null;
            } else {
                container.threadable = threadable;
            }
        }
        if (container == null) {
            container = new ThreadContainer();
            container.threadable = threadable;
            this.idTable.put(id, container);
        }
        ThreadContainer parentRef = null;
        String[] references = threadable.messageThreadReferences();
        for (String refString : references) {
            ThreadContainer ref = this.idTable.get(refString);
            if (ref == null) {
                ref = new ThreadContainer();
                this.idTable.put(refString, ref);
            }
            if (parentRef != null && ref.parent == null && parentRef != ref && !parentRef.findChild(ref)) {
                ref.parent = parentRef;
                ref.next = parentRef.child;
                parentRef.child = ref;
            }
            parentRef = ref;
        }
        if (parentRef != null && (parentRef == container || container.findChild(parentRef))) {
            parentRef = null;
        }
        if (container.parent != null) {
            ThreadContainer prev = null;
            ThreadContainer rest = container.parent.child;
            while (rest != null && rest != container) {
                prev = rest;
                rest = rest.next;
            }
            if (rest != null) {
                if (prev == null) {
                    container.parent.child = container.next;
                } else {
                    prev.next = container.next;
                }
                container.next = null;
                container.parent = null;
            } else {
                throw new RuntimeException("Didnt find " + container + " in parent" + container.parent);
            }
        }
        if (parentRef != null) {
            container.parent = parentRef;
            container.next = parentRef.child;
            parentRef.child = container;
        }
    }

    private ThreadContainer findRootSet() {
        ThreadContainer root2 = new ThreadContainer();
        for (Object key : this.idTable.keySet()) {
            ThreadContainer c = this.idTable.get(key);
            if (c.parent == null) {
                if (c.next == null) {
                    c.next = root2.child;
                    root2.child = c;
                } else {
                    throw new RuntimeException("c.next is " + c.next.toString());
                }
            }
        }
        return root2;
    }

    private void pruneEmptyContainers(ThreadContainer parent) {
        ThreadContainer prev = null;
        ThreadContainer container = parent.child;
        ThreadContainer next = container.next;
        while (container != null) {
            if (container.threadable == null && container.child == null) {
                if (prev == null) {
                    parent.child = container.next;
                } else {
                    prev.next = container.next;
                }
                container = prev;
            } else if (container.threadable == null && container.child != null && (container.parent != null || container.child.next == null)) {
                ThreadContainer kids = container.child;
                if (prev == null) {
                    parent.child = kids;
                } else {
                    prev.next = kids;
                }
                ThreadContainer tail = kids;
                while (tail.next != null) {
                    tail.parent = container.parent;
                    tail = tail.next;
                }
                tail.parent = container.parent;
                tail.next = container.next;
                next = kids;
                container = prev;
            } else if (container.child != null) {
                pruneEmptyContainers(container);
            }
            prev = container;
            container = next;
            next = container == null ? null : container.next;
        }
    }

    private void gatherSubjects() {
        ThreadContainer old;
        ThreadContainer old2;
        int count = 0;
        for (ThreadContainer c = this.root.child; c != null; c = c.next) {
            count++;
        }
        double d = (double) count;
        Double.isNaN(d);
        HashMap<String, ThreadContainer> subjectTable = new HashMap<>((int) (d * 1.2d), 0.9f);
        int count2 = 0;
        for (ThreadContainer c2 = this.root.child; c2 != null; c2 = c2.next) {
            Threadable threadable = c2.threadable;
            if (threadable == null) {
                threadable = c2.child.threadable;
            }
            String subj = threadable.simplifiedSubject();
            if (!(subj == null || subj == "" || ((old2 = subjectTable.get(subj)) != null && ((c2.threadable != null || old2.threadable == null) && (old2.threadable == null || !old2.threadable.subjectIsReply() || c2.threadable == null || c2.threadable.subjectIsReply()))))) {
                subjectTable.put(subj, c2);
                count2++;
            }
        }
        if (count2 != 0) {
            ThreadContainer prev = null;
            ThreadContainer c3 = this.root.child;
            ThreadContainer rest = c3.next;
            while (c3 != null) {
                Threadable threadable2 = c3.threadable;
                if (threadable2 == null) {
                    threadable2 = c3.child.threadable;
                }
                String subj2 = threadable2.simplifiedSubject();
                ThreadContainer threadContainer = null;
                if (!(subj2 == null || subj2 == "" || (old = subjectTable.get(subj2)) == c3)) {
                    if (prev == null) {
                        this.root.child = c3.next;
                    } else {
                        prev.next = c3.next;
                    }
                    c3.next = null;
                    if (old.threadable == null && c3.threadable == null) {
                        ThreadContainer tail = old.child;
                        while (tail != null && tail.next != null) {
                            tail = tail.next;
                        }
                        tail.next = c3.child;
                        for (ThreadContainer tail2 = c3.child; tail2 != null; tail2 = tail2.next) {
                            tail2.parent = old;
                        }
                        c3.child = null;
                    } else if (old.threadable == null || (c3.threadable != null && c3.threadable.subjectIsReply() && !old.threadable.subjectIsReply())) {
                        c3.parent = old;
                        c3.next = old.child;
                        old.child = c3;
                    } else {
                        ThreadContainer newc = new ThreadContainer();
                        newc.threadable = old.threadable;
                        newc.child = old.child;
                        for (ThreadContainer tail3 = newc.child; tail3 != null; tail3 = tail3.next) {
                            tail3.parent = newc;
                        }
                        old.threadable = null;
                        old.child = null;
                        c3.parent = old;
                        newc.parent = old;
                        old.child = c3;
                        c3.next = newc;
                    }
                    c3 = prev;
                }
                prev = c3;
                c3 = rest;
                if (rest != null) {
                    threadContainer = rest.next;
                }
                rest = threadContainer;
            }
            subjectTable.clear();
        }
    }
}
