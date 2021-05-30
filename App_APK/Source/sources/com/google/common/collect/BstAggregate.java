package com.google.common.collect;

import com.google.common.annotations.GwtCompatible;
import com.google.common.collect.BstNode;
import javax.annotation.Nullable;

@GwtCompatible
interface BstAggregate<N extends BstNode<?, N>> {
    int entryValue(N n);

    long treeValue(@Nullable N n);
}
