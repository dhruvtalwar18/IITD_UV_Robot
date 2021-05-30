package org.yaml.snakeyaml.emitter;

public final class ScalarAnalysis {
    public boolean allowBlock;
    public boolean allowBlockPlain;
    public boolean allowFlowPlain;
    public boolean allowSingleQuoted;
    public boolean empty;
    public boolean multiline;
    public String scalar;

    public ScalarAnalysis(String scalar2, boolean empty2, boolean multiline2, boolean allowFlowPlain2, boolean allowBlockPlain2, boolean allowSingleQuoted2, boolean allowBlock2) {
        this.scalar = scalar2;
        this.empty = empty2;
        this.multiline = multiline2;
        this.allowFlowPlain = allowFlowPlain2;
        this.allowBlockPlain = allowBlockPlain2;
        this.allowSingleQuoted = allowSingleQuoted2;
        this.allowBlock = allowBlock2;
    }
}
