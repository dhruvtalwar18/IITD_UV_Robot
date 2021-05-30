package com.google.common.collect;

import com.google.common.annotations.GwtCompatible;
import com.google.common.base.Preconditions;
import com.google.common.collect.BstNode;
import javax.annotation.Nullable;

@GwtCompatible
final class BstModificationResult<N extends BstNode<?, N>> {
    @Nullable
    private final N changedTarget;
    @Nullable
    private final N originalTarget;
    private final ModificationType type;

    enum ModificationType {
        IDENTITY,
        REBUILDING_CHANGE,
        REBALANCING_CHANGE
    }

    static <N extends BstNode<?, N>> BstModificationResult<N> identity(@Nullable N target) {
        return new BstModificationResult<>(target, target, ModificationType.IDENTITY);
    }

    static <N extends BstNode<?, N>> BstModificationResult<N> rebuildingChange(@Nullable N originalTarget2, @Nullable N changedTarget2) {
        return new BstModificationResult<>(originalTarget2, changedTarget2, ModificationType.REBUILDING_CHANGE);
    }

    static <N extends BstNode<?, N>> BstModificationResult<N> rebalancingChange(@Nullable N originalTarget2, @Nullable N changedTarget2) {
        return new BstModificationResult<>(originalTarget2, changedTarget2, ModificationType.REBALANCING_CHANGE);
    }

    private BstModificationResult(@Nullable N originalTarget2, @Nullable N changedTarget2, ModificationType type2) {
        this.originalTarget = originalTarget2;
        this.changedTarget = changedTarget2;
        this.type = (ModificationType) Preconditions.checkNotNull(type2);
    }

    /* access modifiers changed from: package-private */
    @Nullable
    public N getOriginalTarget() {
        return this.originalTarget;
    }

    /* access modifiers changed from: package-private */
    @Nullable
    public N getChangedTarget() {
        return this.changedTarget;
    }

    /* access modifiers changed from: package-private */
    public ModificationType getType() {
        return this.type;
    }
}
