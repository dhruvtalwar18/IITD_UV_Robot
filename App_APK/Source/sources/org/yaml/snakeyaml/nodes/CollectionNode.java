package org.yaml.snakeyaml.nodes;

import org.yaml.snakeyaml.error.Mark;

public abstract class CollectionNode extends Node {
    private Boolean flowStyle;

    public CollectionNode(Tag tag, Mark startMark, Mark endMark, Boolean flowStyle2) {
        super(tag, startMark, endMark);
        this.flowStyle = flowStyle2;
    }

    public Boolean getFlowStyle() {
        return this.flowStyle;
    }

    public void setFlowStyle(Boolean flowStyle2) {
        this.flowStyle = flowStyle2;
    }

    public void setEndMark(Mark endMark) {
        this.endMark = endMark;
    }
}
