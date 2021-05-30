package org.apache.commons.pool.impl;

import java.util.Collection;
import java.util.ConcurrentModificationException;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;
import org.apache.commons.pool.impl.CursorableLinkedList;

/* compiled from: CursorableLinkedList */
class CursorableSubList<E> extends CursorableLinkedList<E> implements List<E> {
    protected CursorableLinkedList<E> _list = null;
    protected CursorableLinkedList.Listable<E> _post = null;
    protected CursorableLinkedList.Listable<E> _pre = null;

    CursorableSubList(CursorableLinkedList<E> list, int from, int to) {
        if (from < 0 || list.size() < to) {
            throw new IndexOutOfBoundsException();
        } else if (from <= to) {
            this._list = list;
            if (from < list.size()) {
                this._head.setNext(this._list.getListableAt(from));
                this._pre = this._head.next() == null ? null : this._head.next().prev();
            } else {
                this._pre = this._list.getListableAt(from - 1);
            }
            if (from == to) {
                this._head.setNext((CursorableLinkedList.Listable) null);
                this._head.setPrev((CursorableLinkedList.Listable) null);
                if (to < list.size()) {
                    this._post = this._list.getListableAt(to);
                } else {
                    this._post = null;
                }
            } else {
                this._head.setPrev(this._list.getListableAt(to - 1));
                this._post = this._head.prev().next();
            }
            this._size = to - from;
            this._modCount = this._list._modCount;
        } else {
            throw new IllegalArgumentException();
        }
    }

    public void clear() {
        checkForComod();
        Iterator<E> it = iterator();
        while (it.hasNext()) {
            it.next();
            it.remove();
        }
    }

    public Iterator<E> iterator() {
        checkForComod();
        return super.iterator();
    }

    public int size() {
        checkForComod();
        return super.size();
    }

    public boolean isEmpty() {
        checkForComod();
        return super.isEmpty();
    }

    public Object[] toArray() {
        checkForComod();
        return super.toArray();
    }

    public <T> T[] toArray(T[] a) {
        checkForComod();
        return super.toArray(a);
    }

    public boolean contains(Object o) {
        checkForComod();
        return super.contains(o);
    }

    public boolean remove(Object o) {
        checkForComod();
        return super.remove(o);
    }

    public E removeFirst() {
        checkForComod();
        return super.removeFirst();
    }

    public E removeLast() {
        checkForComod();
        return super.removeLast();
    }

    public boolean addAll(Collection<? extends E> c) {
        checkForComod();
        return super.addAll(c);
    }

    public boolean add(E o) {
        checkForComod();
        return super.add(o);
    }

    public boolean addFirst(E o) {
        checkForComod();
        return super.addFirst(o);
    }

    public boolean addLast(E o) {
        checkForComod();
        return super.addLast(o);
    }

    public boolean removeAll(Collection<?> c) {
        checkForComod();
        return super.removeAll(c);
    }

    public boolean containsAll(Collection<?> c) {
        checkForComod();
        return super.containsAll(c);
    }

    public boolean addAll(int index, Collection<? extends E> c) {
        checkForComod();
        return super.addAll(index, c);
    }

    public int hashCode() {
        checkForComod();
        return super.hashCode();
    }

    public boolean retainAll(Collection<?> c) {
        checkForComod();
        return super.retainAll(c);
    }

    public E set(int index, E element) {
        checkForComod();
        return super.set(index, element);
    }

    public boolean equals(Object o) {
        checkForComod();
        return super.equals(o);
    }

    public E get(int index) {
        checkForComod();
        return super.get(index);
    }

    public E getFirst() {
        checkForComod();
        return super.getFirst();
    }

    public E getLast() {
        checkForComod();
        return super.getLast();
    }

    public void add(int index, E element) {
        checkForComod();
        super.add(index, element);
    }

    public ListIterator<E> listIterator(int index) {
        checkForComod();
        return super.listIterator(index);
    }

    public E remove(int index) {
        checkForComod();
        return super.remove(index);
    }

    public int indexOf(Object o) {
        checkForComod();
        return super.indexOf(o);
    }

    public int lastIndexOf(Object o) {
        checkForComod();
        return super.lastIndexOf(o);
    }

    public ListIterator<E> listIterator() {
        checkForComod();
        return super.listIterator();
    }

    public List<E> subList(int fromIndex, int toIndex) {
        checkForComod();
        return super.subList(fromIndex, toIndex);
    }

    /* access modifiers changed from: protected */
    public CursorableLinkedList.Listable<E> insertListable(CursorableLinkedList.Listable<E> before, CursorableLinkedList.Listable<E> after, E value) {
        this._modCount++;
        this._size++;
        CursorableLinkedList.Listable<E> elt = this._list.insertListable(before == null ? this._pre : before, after == null ? this._post : after, value);
        if (this._head.next() == null) {
            this._head.setNext(elt);
            this._head.setPrev(elt);
        }
        if (before == this._head.prev()) {
            this._head.setPrev(elt);
        }
        if (after == this._head.next()) {
            this._head.setNext(elt);
        }
        broadcastListableInserted(elt);
        return elt;
    }

    /* access modifiers changed from: protected */
    public void removeListable(CursorableLinkedList.Listable<E> elt) {
        this._modCount++;
        this._size--;
        if (this._head.next() == elt && this._head.prev() == elt) {
            this._head.setNext((CursorableLinkedList.Listable) null);
            this._head.setPrev((CursorableLinkedList.Listable) null);
        }
        if (this._head.next() == elt) {
            this._head.setNext(elt.next());
        }
        if (this._head.prev() == elt) {
            this._head.setPrev(elt.prev());
        }
        this._list.removeListable(elt);
        broadcastListableRemoved(elt);
    }

    /* access modifiers changed from: protected */
    public void checkForComod() throws ConcurrentModificationException {
        if (this._modCount != this._list._modCount) {
            throw new ConcurrentModificationException();
        }
    }
}
