package com.google.common.collect;

import com.google.common.annotations.GwtCompatible;
import com.google.common.collect.BstNode;
import com.google.common.collect.BstPath;

@GwtCompatible
interface BstPathFactory<N extends BstNode<?, N>, P extends BstPath<N, P>> {
    P extension(P p, BstSide bstSide);

    P initialPath(N n);
}
