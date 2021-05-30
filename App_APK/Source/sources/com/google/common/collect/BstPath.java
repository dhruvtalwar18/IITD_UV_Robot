package com.google.common.collect;

import com.google.common.annotations.GwtCompatible;
import com.google.common.base.Preconditions;
import com.google.common.collect.BstNode;
import com.google.common.collect.BstPath;
import javax.annotation.Nullable;

@GwtCompatible
abstract class BstPath<N extends BstNode<?, N>, P extends BstPath<N, P>> {
    @Nullable
    private final P prefix;
    private final N tip;

    BstPath(N tip2, @Nullable P prefix2) {
        this.tip = (BstNode) Preconditions.checkNotNull(tip2);
        this.prefix = prefix2;
    }

    public final N getTip() {
        return this.tip;
    }

    public final boolean hasPrefix() {
        return this.prefix != null;
    }

    @Nullable
    public final P prefixOrNull() {
        return this.prefix;
    }

    public final P getPrefix() {
        Preconditions.checkState(hasPrefix());
        return this.prefix;
    }
}
