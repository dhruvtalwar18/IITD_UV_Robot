package com.google.common.collect;

import com.google.common.annotations.Beta;
import com.google.common.annotations.GwtCompatible;
import com.google.common.base.Objects;
import com.google.common.collect.Multiset;
import com.google.common.collect.Multisets;
import java.util.Collection;
import java.util.Iterator;
import java.util.Set;
import javax.annotation.Nullable;

@GwtCompatible
public abstract class ForwardingMultiset<E> extends ForwardingCollection<E> implements Multiset<E> {
    /* access modifiers changed from: protected */
    public abstract Multiset<E> delegate();

    protected ForwardingMultiset() {
    }

    public int count(Object element) {
        return delegate().count(element);
    }

    public int add(E element, int occurrences) {
        return delegate().add(element, occurrences);
    }

    public int remove(Object element, int occurrences) {
        return delegate().remove(element, occurrences);
    }

    public Set<E> elementSet() {
        return delegate().elementSet();
    }

    public Set<Multiset.Entry<E>> entrySet() {
        return delegate().entrySet();
    }

    public boolean equals(@Nullable Object object) {
        return object == this || delegate().equals(object);
    }

    public int hashCode() {
        return delegate().hashCode();
    }

    public int setCount(E element, int count) {
        return delegate().setCount(element, count);
    }

    public boolean setCount(E element, int oldCount, int newCount) {
        return delegate().setCount(element, oldCount, newCount);
    }

    /* access modifiers changed from: protected */
    @Beta
    public boolean standardContains(@Nullable Object object) {
        return count(object) > 0;
    }

    /* access modifiers changed from: protected */
    @Beta
    public void standardClear() {
        Iterator<Multiset.Entry<E>> entryIterator = entrySet().iterator();
        while (entryIterator.hasNext()) {
            entryIterator.next();
            entryIterator.remove();
        }
    }

    /* access modifiers changed from: protected */
    @Beta
    public int standardCount(@Nullable Object object) {
        for (Multiset.Entry<?> entry : entrySet()) {
            if (Objects.equal(entry.getElement(), object)) {
                return entry.getCount();
            }
        }
        return 0;
    }

    /* access modifiers changed from: protected */
    @Beta
    public boolean standardAdd(E element) {
        add(element, 1);
        return true;
    }

    /* access modifiers changed from: protected */
    @Beta
    public boolean standardAddAll(Collection<? extends E> elementsToAdd) {
        return Multisets.addAllImpl(this, elementsToAdd);
    }

    /* access modifiers changed from: protected */
    @Beta
    public boolean standardRemove(Object element) {
        return remove(element, 1) > 0;
    }

    /* access modifiers changed from: protected */
    @Beta
    public boolean standardRemoveAll(Collection<?> elementsToRemove) {
        return Multisets.removeAllImpl(this, elementsToRemove);
    }

    /* access modifiers changed from: protected */
    @Beta
    public boolean standardRetainAll(Collection<?> elementsToRetain) {
        return Multisets.retainAllImpl(this, elementsToRetain);
    }

    /* access modifiers changed from: protected */
    @Beta
    public int standardSetCount(E element, int count) {
        return Multisets.setCountImpl(this, element, count);
    }

    /* access modifiers changed from: protected */
    @Beta
    public boolean standardSetCount(E element, int oldCount, int newCount) {
        return Multisets.setCountImpl(this, element, oldCount, newCount);
    }

    @Beta
    protected class StandardElementSet extends Multisets.ElementSet<E> {
        public StandardElementSet() {
        }

        /* access modifiers changed from: package-private */
        public Multiset<E> multiset() {
            return ForwardingMultiset.this;
        }
    }

    /* access modifiers changed from: protected */
    @Beta
    public Iterator<E> standardIterator() {
        return Multisets.iteratorImpl(this);
    }

    /* access modifiers changed from: protected */
    @Beta
    public int standardSize() {
        return Multisets.sizeImpl(this);
    }

    /* access modifiers changed from: protected */
    @Beta
    public boolean standardEquals(@Nullable Object object) {
        return Multisets.equalsImpl(this, object);
    }

    /* access modifiers changed from: protected */
    @Beta
    public int standardHashCode() {
        return entrySet().hashCode();
    }

    /* access modifiers changed from: protected */
    @Beta
    public String standardToString() {
        return entrySet().toString();
    }
}
