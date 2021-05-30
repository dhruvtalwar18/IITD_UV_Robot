package com.google.common.collect;

import com.google.common.annotations.GwtCompatible;
import com.google.common.base.Preconditions;
import com.google.common.collect.BstModificationResult;
import com.google.common.collect.BstNode;
import javax.annotation.Nullable;

@GwtCompatible
final class BstMutationResult<K, N extends BstNode<K, N>> {
    static final /* synthetic */ boolean $assertionsDisabled = false;
    @Nullable
    private N changedRoot;
    private final BstModificationResult<N> modificationResult;
    @Nullable
    private N originalRoot;
    private final K targetKey;

    public static <K, N extends BstNode<K, N>> BstMutationResult<K, N> mutationResult(@Nullable K targetKey2, @Nullable N originalRoot2, @Nullable N changedRoot2, BstModificationResult<N> modificationResult2) {
        return new BstMutationResult<>(targetKey2, originalRoot2, changedRoot2, modificationResult2);
    }

    private BstMutationResult(@Nullable K targetKey2, @Nullable N originalRoot2, @Nullable N changedRoot2, BstModificationResult<N> modificationResult2) {
        this.targetKey = targetKey2;
        this.originalRoot = originalRoot2;
        this.changedRoot = changedRoot2;
        this.modificationResult = (BstModificationResult) Preconditions.checkNotNull(modificationResult2);
    }

    public K getTargetKey() {
        return this.targetKey;
    }

    @Nullable
    public N getOriginalRoot() {
        return this.originalRoot;
    }

    @Nullable
    public N getChangedRoot() {
        return this.changedRoot;
    }

    @Nullable
    public N getOriginalTarget() {
        return this.modificationResult.getOriginalTarget();
    }

    @Nullable
    public N getChangedTarget() {
        return this.modificationResult.getChangedTarget();
    }

    /* access modifiers changed from: package-private */
    public BstModificationResult.ModificationType modificationType() {
        return this.modificationResult.getType();
    }

    public BstMutationResult<K, N> lift(N liftOriginalRoot, BstSide side, BstNodeFactory<N> nodeFactory, BstBalancePolicy<N> balancePolicy) {
        switch (modificationType()) {
            case IDENTITY:
                this.changedRoot = liftOriginalRoot;
                this.originalRoot = liftOriginalRoot;
                return this;
            case REBUILDING_CHANGE:
            case REBALANCING_CHANGE:
                this.originalRoot = liftOriginalRoot;
                N resultLeft = liftOriginalRoot.childOrNull(BstSide.LEFT);
                N resultRight = liftOriginalRoot.childOrNull(BstSide.RIGHT);
                switch (side) {
                    case LEFT:
                        resultLeft = this.changedRoot;
                        break;
                    case RIGHT:
                        resultRight = this.changedRoot;
                        break;
                    default:
                        throw new AssertionError();
                }
                if (modificationType() == BstModificationResult.ModificationType.REBUILDING_CHANGE) {
                    this.changedRoot = nodeFactory.createNode(liftOriginalRoot, resultLeft, resultRight);
                } else {
                    this.changedRoot = balancePolicy.balance(nodeFactory, liftOriginalRoot, resultLeft, resultRight);
                }
                return this;
            default:
                throw new AssertionError();
        }
    }
}
