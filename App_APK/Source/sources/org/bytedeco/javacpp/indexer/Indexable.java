package org.bytedeco.javacpp.indexer;

public interface Indexable {
    <I extends Indexer> I createIndexer(boolean z);
}
