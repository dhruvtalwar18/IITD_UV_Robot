package org.yaml.snakeyaml.nodes;

import org.yaml.snakeyaml.error.Mark;

public abstract class Node {
    protected Mark endMark;
    protected boolean resolved = true;
    private Mark startMark;
    private Tag tag;
    private boolean twoStepsConstruction = false;
    private Class<? extends Object> type = Object.class;
    protected Boolean useClassConstructor = null;

    public abstract NodeId getNodeId();

    public Node(Tag tag2, Mark startMark2, Mark endMark2) {
        setTag(tag2);
        this.startMark = startMark2;
        this.endMark = endMark2;
    }

    public Tag getTag() {
        return this.tag;
    }

    public Mark getEndMark() {
        return this.endMark;
    }

    public Mark getStartMark() {
        return this.startMark;
    }

    public void setTag(Tag tag2) {
        if (tag2 != null) {
            this.tag = tag2;
            return;
        }
        throw new NullPointerException("tag in a Node is required.");
    }

    public final boolean equals(Object obj) {
        return super.equals(obj);
    }

    public Class<? extends Object> getType() {
        return this.type;
    }

    public void setType(Class<? extends Object> type2) {
        if (!type2.isAssignableFrom(this.type)) {
            this.type = type2;
        }
    }

    public void setTwoStepsConstruction(boolean twoStepsConstruction2) {
        this.twoStepsConstruction = twoStepsConstruction2;
    }

    public boolean isTwoStepsConstruction() {
        return this.twoStepsConstruction;
    }

    public final int hashCode() {
        return super.hashCode();
    }

    public boolean useClassConstructor() {
        if (this.useClassConstructor != null) {
            return this.useClassConstructor.booleanValue();
        }
        if ((!isResolved() || Object.class.equals(this.type) || this.tag.equals(Tag.NULL)) && !this.tag.isCompatible(getType())) {
            return false;
        }
        return true;
    }

    public void setUseClassConstructor(Boolean useClassConstructor2) {
        this.useClassConstructor = useClassConstructor2;
    }

    public boolean isResolved() {
        return this.resolved;
    }
}
