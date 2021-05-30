package org.apache.commons.pool.impl;

import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.Serializable;
import java.lang.ref.WeakReference;
import java.util.ArrayList;
import java.util.Collection;
import java.util.ConcurrentModificationException;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;
import java.util.NoSuchElementException;

class CursorableLinkedList<E> implements List<E>, Serializable {
    private static final long serialVersionUID = 8836393098519411393L;
    protected transient List<WeakReference<CursorableLinkedList<E>.Cursor>> _cursors = new ArrayList();
    protected transient Listable<E> _head = new Listable<>((Listable) null, (Listable) null, null);
    protected transient int _modCount = 0;
    protected transient int _size = 0;

    CursorableLinkedList() {
    }

    public boolean add(E o) {
        insertListable(this._head.prev(), (Listable<E>) null, o);
        return true;
    }

    public void add(int index, E element) {
        if (index == this._size) {
            add(element);
        } else if (index < 0 || index > this._size) {
            throw new IndexOutOfBoundsException(String.valueOf(index) + " < 0 or " + String.valueOf(index) + " > " + this._size);
        } else {
            Listable<E> pred = null;
            Listable<E> succ = isEmpty() ? null : getListableAt(index);
            if (succ != null) {
                pred = succ.prev();
            }
            insertListable(pred, succ, element);
        }
    }

    public boolean addAll(Collection<? extends E> c) {
        if (c.isEmpty()) {
            return false;
        }
        for (Object insertListable : c) {
            insertListable(this._head.prev(), (Listable<E>) null, insertListable);
        }
        return true;
    }

    public boolean addAll(int index, Collection<? extends E> c) {
        if (c.isEmpty()) {
            return false;
        }
        if (this._size == index || this._size == 0) {
            return addAll(c);
        }
        Listable<E> succ = getListableAt(index);
        Listable<E> pred = succ == null ? null : succ.prev();
        for (Object insertListable : c) {
            pred = insertListable(pred, succ, insertListable);
        }
        return true;
    }

    public boolean addFirst(E o) {
        insertListable((Listable) null, this._head.next(), o);
        return true;
    }

    public boolean addLast(E o) {
        insertListable(this._head.prev(), (Listable<E>) null, o);
        return true;
    }

    public void clear() {
        Iterator<E> it = iterator();
        while (it.hasNext()) {
            it.next();
            it.remove();
        }
    }

    public boolean contains(Object o) {
        Listable<E> past = null;
        for (Listable<E> elt = this._head.next(); elt != null && past != this._head.prev(); elt = elt.next()) {
            if (o == null && elt.value() == null) {
                return true;
            }
            if (o != null && o.equals(elt.value())) {
                return true;
            }
            past = elt;
        }
        return false;
    }

    public boolean containsAll(Collection<?> c) {
        for (Object contains : c) {
            if (!contains(contains)) {
                return false;
            }
        }
        return true;
    }

    public CursorableLinkedList<E>.Cursor cursor() {
        return new Cursor(0);
    }

    public CursorableLinkedList<E>.Cursor cursor(int i) {
        return new Cursor(i);
    }

    public boolean equals(Object o) {
        if (o == this) {
            return true;
        }
        if (!(o instanceof List)) {
            return false;
        }
        Iterator<?> it = ((List) o).listIterator();
        Listable<E> elt = this._head.next();
        Listable<E> past = null;
        while (elt != null && past != this._head.prev()) {
            if (it.hasNext()) {
                if (elt.value() == null) {
                    if (it.next() != null) {
                    }
                } else if (!elt.value().equals(it.next())) {
                }
                past = elt;
                elt = elt.next();
            }
            return false;
        }
        return true ^ it.hasNext();
    }

    public E get(int index) {
        return getListableAt(index).value();
    }

    public E getFirst() {
        try {
            return this._head.next().value();
        } catch (NullPointerException e) {
            throw new NoSuchElementException();
        }
    }

    public E getLast() {
        try {
            return this._head.prev().value();
        } catch (NullPointerException e) {
            throw new NoSuchElementException();
        }
    }

    public int hashCode() {
        int hash = 1;
        Listable<E> past = null;
        for (Listable<E> elt = this._head.next(); elt != null && past != this._head.prev(); elt = elt.next()) {
            hash = (hash * 31) + (elt.value() == null ? 0 : elt.value().hashCode());
            past = elt;
        }
        return hash;
    }

    public int indexOf(Object o) {
        int ndx = 0;
        Listable<E> past = null;
        if (o == null) {
            Listable<E> elt = this._head;
            while (true) {
                elt = elt.next();
                if (elt == null || past == this._head.prev()) {
                    return -1;
                }
                if (elt.value() == null) {
                    return ndx;
                }
                ndx++;
                past = elt;
            }
        } else {
            Listable<E> elt2 = this._head;
            while (true) {
                elt2 = elt2.next();
                if (elt2 == null || past == this._head.prev()) {
                    return -1;
                }
                if (o.equals(elt2.value())) {
                    return ndx;
                }
                ndx++;
                past = elt2;
            }
        }
    }

    public boolean isEmpty() {
        return this._size == 0;
    }

    public Iterator<E> iterator() {
        return listIterator(0);
    }

    public int lastIndexOf(Object o) {
        int ndx = this._size - 1;
        Listable<E> past = null;
        if (o == null) {
            Listable<E> elt = this._head;
            while (true) {
                elt = elt.prev();
                if (elt == null || past == this._head.next()) {
                    return -1;
                }
                if (elt.value() == null) {
                    return ndx;
                }
                ndx--;
                past = elt;
            }
        } else {
            Listable<E> elt2 = this._head;
            while (true) {
                elt2 = elt2.prev();
                if (elt2 == null || past == this._head.next()) {
                    return -1;
                }
                if (o.equals(elt2.value())) {
                    return ndx;
                }
                ndx--;
                past = elt2;
            }
        }
    }

    public ListIterator<E> listIterator() {
        return listIterator(0);
    }

    public ListIterator<E> listIterator(int index) {
        if (index >= 0 && index <= this._size) {
            return new ListIter(index);
        }
        throw new IndexOutOfBoundsException(index + " < 0 or > " + this._size);
    }

    public boolean remove(Object o) {
        Listable<E> elt = this._head.next();
        Listable<E> past = null;
        while (elt != null && past != this._head.prev()) {
            if (o == null && elt.value() == null) {
                removeListable(elt);
                return true;
            } else if (o == null || !o.equals(elt.value())) {
                past = elt;
                elt = elt.next();
            } else {
                removeListable(elt);
                return true;
            }
        }
        return false;
    }

    public E remove(int index) {
        Listable<E> elt = getListableAt(index);
        E ret = elt.value();
        removeListable(elt);
        return ret;
    }

    public boolean removeAll(Collection<?> c) {
        if (c.size() == 0 || this._size == 0) {
            return false;
        }
        boolean changed = false;
        Iterator<?> it = iterator();
        while (it.hasNext()) {
            if (c.contains(it.next())) {
                it.remove();
                changed = true;
            }
        }
        return changed;
    }

    public E removeFirst() {
        if (this._head.next() != null) {
            E val = this._head.next().value();
            removeListable(this._head.next());
            return val;
        }
        throw new NoSuchElementException();
    }

    public E removeLast() {
        if (this._head.prev() != null) {
            E val = this._head.prev().value();
            removeListable(this._head.prev());
            return val;
        }
        throw new NoSuchElementException();
    }

    public boolean retainAll(Collection<?> c) {
        boolean changed = false;
        Iterator<?> it = iterator();
        while (it.hasNext()) {
            if (!c.contains(it.next())) {
                it.remove();
                changed = true;
            }
        }
        return changed;
    }

    public E set(int index, E element) {
        Listable<E> elt = getListableAt(index);
        E val = elt.setValue(element);
        broadcastListableChanged(elt);
        return val;
    }

    public int size() {
        return this._size;
    }

    public Object[] toArray() {
        Object[] array = new Object[this._size];
        int i = 0;
        Listable<E> elt = this._head.next();
        Listable<E> past = null;
        while (elt != null && past != this._head.prev()) {
            array[i] = elt.value();
            past = elt;
            elt = elt.next();
            i++;
        }
        return array;
    }

    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r7v1, resolved type: T[]} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r0v9, resolved type: java.lang.Object} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r7v2, resolved type: T[]} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r7v3, resolved type: T[]} */
    /* JADX WARNING: Multi-variable type inference failed */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public <T> T[] toArray(T[] r7) {
        /*
            r6 = this;
            int r0 = r7.length
            int r1 = r6._size
            if (r0 >= r1) goto L_0x0016
            java.lang.Class r0 = r7.getClass()
            java.lang.Class r0 = r0.getComponentType()
            int r1 = r6._size
            java.lang.Object r0 = java.lang.reflect.Array.newInstance(r0, r1)
            r7 = r0
            java.lang.Object[] r7 = (java.lang.Object[]) r7
        L_0x0016:
            r0 = 0
            org.apache.commons.pool.impl.CursorableLinkedList$Listable<E> r1 = r6._head
            org.apache.commons.pool.impl.CursorableLinkedList$Listable r1 = r1.next()
            r2 = 0
            r3 = r0
            r0 = r2
        L_0x0020:
            if (r1 == 0) goto L_0x0039
            org.apache.commons.pool.impl.CursorableLinkedList$Listable<E> r4 = r6._head
            org.apache.commons.pool.impl.CursorableLinkedList$Listable r4 = r4.prev()
            if (r0 == r4) goto L_0x0039
            int r4 = r3 + 1
            java.lang.Object r5 = r1.value()
            r7[r3] = r5
            r0 = r1
            org.apache.commons.pool.impl.CursorableLinkedList$Listable r1 = r1.next()
            r3 = r4
            goto L_0x0020
        L_0x0039:
            int r0 = r7.length
            int r1 = r6._size
            if (r0 <= r1) goto L_0x0042
            int r0 = r6._size
            r7[r0] = r2
        L_0x0042:
            return r7
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.pool.impl.CursorableLinkedList.toArray(java.lang.Object[]):java.lang.Object[]");
    }

    public String toString() {
        StringBuffer buf = new StringBuffer();
        buf.append("[");
        Listable<E> past = null;
        for (Listable<E> elt = this._head.next(); elt != null && past != this._head.prev(); elt = elt.next()) {
            if (this._head.next() != elt) {
                buf.append(", ");
            }
            buf.append(elt.value());
            past = elt;
        }
        buf.append("]");
        return buf.toString();
    }

    public List<E> subList(int i, int j) {
        if (i < 0 || j > this._size || i > j) {
            throw new IndexOutOfBoundsException();
        } else if (i == 0 && j == this._size) {
            return this;
        } else {
            return new CursorableSubList(this, i, j);
        }
    }

    /* access modifiers changed from: protected */
    public Listable<E> insertListable(Listable<E> before, Listable<E> after, E value) {
        this._modCount++;
        this._size++;
        Listable<E> elt = new Listable<>(before, after, value);
        if (before != null) {
            before.setNext(elt);
        } else {
            this._head.setNext(elt);
        }
        if (after != null) {
            after.setPrev(elt);
        } else {
            this._head.setPrev(elt);
        }
        broadcastListableInserted(elt);
        return elt;
    }

    /* access modifiers changed from: protected */
    public void removeListable(Listable<E> elt) {
        this._modCount++;
        this._size--;
        if (this._head.next() == elt) {
            this._head.setNext(elt.next());
        }
        if (elt.next() != null) {
            elt.next().setPrev(elt.prev());
        }
        if (this._head.prev() == elt) {
            this._head.setPrev(elt.prev());
        }
        if (elt.prev() != null) {
            elt.prev().setNext(elt.next());
        }
        broadcastListableRemoved(elt);
    }

    /* access modifiers changed from: protected */
    public Listable<E> getListableAt(int index) {
        if (index < 0 || index >= this._size) {
            throw new IndexOutOfBoundsException(String.valueOf(index) + " < 0 or " + String.valueOf(index) + " >= " + this._size);
        } else if (index <= this._size / 2) {
            Listable<E> elt = this._head.next();
            for (int i = 0; i < index; i++) {
                elt = elt.next();
            }
            return elt;
        } else {
            Listable<E> elt2 = this._head.prev();
            for (int i2 = this._size - 1; i2 > index; i2--) {
                elt2 = elt2.prev();
            }
            return elt2;
        }
    }

    /* access modifiers changed from: protected */
    public void registerCursor(CursorableLinkedList<E>.Cursor cur) {
        Iterator<WeakReference<CursorableLinkedList<E>.Cursor>> it = this._cursors.iterator();
        while (it.hasNext()) {
            if (it.next().get() == null) {
                it.remove();
            }
        }
        this._cursors.add(new WeakReference(cur));
    }

    /* access modifiers changed from: protected */
    public void unregisterCursor(CursorableLinkedList<E>.Cursor cur) {
        Iterator<WeakReference<CursorableLinkedList<E>.Cursor>> it = this._cursors.iterator();
        while (it.hasNext()) {
            WeakReference<CursorableLinkedList<E>.Cursor> ref = it.next();
            CursorableLinkedList<E>.Cursor cursor = (Cursor) ref.get();
            if (cursor == null) {
                it.remove();
            } else if (cursor == cur) {
                ref.clear();
                it.remove();
                return;
            }
        }
    }

    /* access modifiers changed from: protected */
    public void invalidateCursors() {
        Iterator<WeakReference<CursorableLinkedList<E>.Cursor>> it = this._cursors.iterator();
        while (it.hasNext()) {
            WeakReference<CursorableLinkedList<E>.Cursor> ref = it.next();
            CursorableLinkedList<E>.Cursor cursor = (Cursor) ref.get();
            if (cursor != null) {
                cursor.invalidate();
                ref.clear();
            }
            it.remove();
        }
    }

    /* access modifiers changed from: protected */
    public void broadcastListableChanged(Listable<E> elt) {
        Iterator<WeakReference<CursorableLinkedList<E>.Cursor>> it = this._cursors.iterator();
        while (it.hasNext()) {
            CursorableLinkedList<E>.Cursor cursor = (Cursor) it.next().get();
            if (cursor == null) {
                it.remove();
            } else {
                cursor.listableChanged(elt);
            }
        }
    }

    /* access modifiers changed from: protected */
    public void broadcastListableRemoved(Listable<E> elt) {
        Iterator<WeakReference<CursorableLinkedList<E>.Cursor>> it = this._cursors.iterator();
        while (it.hasNext()) {
            CursorableLinkedList<E>.Cursor cursor = (Cursor) it.next().get();
            if (cursor == null) {
                it.remove();
            } else {
                cursor.listableRemoved(elt);
            }
        }
    }

    /* access modifiers changed from: protected */
    public void broadcastListableInserted(Listable<E> elt) {
        Iterator<WeakReference<CursorableLinkedList<E>.Cursor>> it = this._cursors.iterator();
        while (it.hasNext()) {
            CursorableLinkedList<E>.Cursor cursor = (Cursor) it.next().get();
            if (cursor == null) {
                it.remove();
            } else {
                cursor.listableInserted(elt);
            }
        }
    }

    private void writeObject(ObjectOutputStream out) throws IOException {
        out.defaultWriteObject();
        out.writeInt(this._size);
        for (Listable<E> cur = this._head.next(); cur != null; cur = cur.next()) {
            out.writeObject(cur.value());
        }
    }

    private void readObject(ObjectInputStream in) throws IOException, ClassNotFoundException {
        in.defaultReadObject();
        this._size = 0;
        this._modCount = 0;
        this._cursors = new ArrayList();
        this._head = new Listable<>((Listable) null, (Listable) null, null);
        int size = in.readInt();
        for (int i = 0; i < size; i++) {
            add(in.readObject());
        }
    }

    static class Listable<E> implements Serializable {
        private Listable<E> _next = null;
        private Listable<E> _prev = null;
        private E _val = null;

        Listable(Listable<E> prev, Listable<E> next, E val) {
            this._prev = prev;
            this._next = next;
            this._val = val;
        }

        /* access modifiers changed from: package-private */
        public Listable<E> next() {
            return this._next;
        }

        /* access modifiers changed from: package-private */
        public Listable<E> prev() {
            return this._prev;
        }

        /* access modifiers changed from: package-private */
        public E value() {
            return this._val;
        }

        /* access modifiers changed from: package-private */
        public void setNext(Listable<E> next) {
            this._next = next;
        }

        /* access modifiers changed from: package-private */
        public void setPrev(Listable<E> prev) {
            this._prev = prev;
        }

        /* access modifiers changed from: package-private */
        public E setValue(E val) {
            E temp = this._val;
            this._val = val;
            return temp;
        }
    }

    class ListIter implements ListIterator<E> {
        Listable<E> _cur = null;
        int _expectedModCount = CursorableLinkedList.this._modCount;
        Listable<E> _lastReturned = null;
        int _nextIndex = 0;

        ListIter(int index) {
            if (index == 0) {
                this._cur = new Listable<>((Listable) null, CursorableLinkedList.this._head.next(), null);
                this._nextIndex = 0;
            } else if (index == CursorableLinkedList.this._size) {
                this._cur = new Listable<>(CursorableLinkedList.this._head.prev(), (Listable<E>) null, null);
                this._nextIndex = CursorableLinkedList.this._size;
            } else {
                Listable<E> temp = CursorableLinkedList.this.getListableAt(index);
                this._cur = new Listable<>(temp.prev(), temp, null);
                this._nextIndex = index;
            }
        }

        public E previous() {
            checkForComod();
            if (hasPrevious()) {
                E ret = this._cur.prev().value();
                this._lastReturned = this._cur.prev();
                this._cur.setNext(this._cur.prev());
                this._cur.setPrev(this._cur.prev().prev());
                this._nextIndex--;
                return ret;
            }
            throw new NoSuchElementException();
        }

        public boolean hasNext() {
            checkForComod();
            return (this._cur.next() == null || this._cur.prev() == CursorableLinkedList.this._head.prev()) ? false : true;
        }

        public E next() {
            checkForComod();
            if (hasNext()) {
                E ret = this._cur.next().value();
                this._lastReturned = this._cur.next();
                this._cur.setPrev(this._cur.next());
                this._cur.setNext(this._cur.next().next());
                this._nextIndex++;
                return ret;
            }
            throw new NoSuchElementException();
        }

        public int previousIndex() {
            checkForComod();
            if (!hasPrevious()) {
                return -1;
            }
            return this._nextIndex - 1;
        }

        public boolean hasPrevious() {
            checkForComod();
            return (this._cur.prev() == null || this._cur.next() == CursorableLinkedList.this._head.next()) ? false : true;
        }

        public void set(E o) {
            checkForComod();
            try {
                this._lastReturned.setValue(o);
            } catch (NullPointerException e) {
                throw new IllegalStateException();
            }
        }

        public int nextIndex() {
            checkForComod();
            if (!hasNext()) {
                return CursorableLinkedList.this.size();
            }
            return this._nextIndex;
        }

        public void remove() {
            checkForComod();
            if (this._lastReturned != null) {
                this._cur.setNext(this._lastReturned == CursorableLinkedList.this._head.prev() ? null : this._lastReturned.next());
                this._cur.setPrev(this._lastReturned == CursorableLinkedList.this._head.next() ? null : this._lastReturned.prev());
                CursorableLinkedList.this.removeListable(this._lastReturned);
                this._lastReturned = null;
                this._nextIndex--;
                this._expectedModCount++;
                return;
            }
            throw new IllegalStateException();
        }

        public void add(E o) {
            checkForComod();
            this._cur.setPrev(CursorableLinkedList.this.insertListable(this._cur.prev(), this._cur.next(), o));
            this._lastReturned = null;
            this._nextIndex++;
            this._expectedModCount++;
        }

        /* access modifiers changed from: protected */
        public void checkForComod() {
            if (this._expectedModCount != CursorableLinkedList.this._modCount) {
                throw new ConcurrentModificationException();
            }
        }
    }

    public class Cursor extends CursorableLinkedList<E>.ListIter implements ListIterator<E> {
        boolean _valid;

        public /* bridge */ /* synthetic */ boolean hasNext() {
            return super.hasNext();
        }

        public /* bridge */ /* synthetic */ boolean hasPrevious() {
            return super.hasPrevious();
        }

        public /* bridge */ /* synthetic */ Object next() {
            return super.next();
        }

        public /* bridge */ /* synthetic */ Object previous() {
            return super.previous();
        }

        public /* bridge */ /* synthetic */ void remove() {
            super.remove();
        }

        public /* bridge */ /* synthetic */ void set(Object x0) {
            super.set(x0);
        }

        Cursor(int index) {
            super(index);
            this._valid = false;
            this._valid = true;
            CursorableLinkedList.this.registerCursor(this);
        }

        public int previousIndex() {
            throw new UnsupportedOperationException();
        }

        public int nextIndex() {
            throw new UnsupportedOperationException();
        }

        public void add(E o) {
            checkForComod();
            Listable<E> elt = CursorableLinkedList.this.insertListable(this._cur.prev(), this._cur.next(), o);
            this._cur.setPrev(elt);
            this._cur.setNext(elt.next());
            this._lastReturned = null;
            this._nextIndex++;
            this._expectedModCount++;
        }

        /* access modifiers changed from: protected */
        public void listableRemoved(Listable<E> elt) {
            if (CursorableLinkedList.this._head.prev() == null) {
                this._cur.setNext((Listable) null);
            } else if (this._cur.next() == elt) {
                this._cur.setNext(elt.next());
            }
            if (CursorableLinkedList.this._head.next() == null) {
                this._cur.setPrev((Listable) null);
            } else if (this._cur.prev() == elt) {
                this._cur.setPrev(elt.prev());
            }
            if (this._lastReturned == elt) {
                this._lastReturned = null;
            }
        }

        /* access modifiers changed from: protected */
        public void listableInserted(Listable<E> elt) {
            if (this._cur.next() == null && this._cur.prev() == null) {
                this._cur.setNext(elt);
            } else if (this._cur.prev() == elt.prev()) {
                this._cur.setNext(elt);
            }
            if (this._cur.next() == elt.next()) {
                this._cur.setPrev(elt);
            }
            if (this._lastReturned == elt) {
                this._lastReturned = null;
            }
        }

        /* access modifiers changed from: protected */
        public void listableChanged(Listable<E> elt) {
            if (this._lastReturned == elt) {
                this._lastReturned = null;
            }
        }

        /* access modifiers changed from: protected */
        public void checkForComod() {
            if (!this._valid) {
                throw new ConcurrentModificationException();
            }
        }

        /* access modifiers changed from: protected */
        public void invalidate() {
            this._valid = false;
        }

        public void close() {
            if (this._valid) {
                this._valid = false;
                CursorableLinkedList.this.unregisterCursor(this);
            }
        }
    }
}
