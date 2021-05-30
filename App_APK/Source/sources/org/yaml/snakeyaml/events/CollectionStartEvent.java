package org.yaml.snakeyaml.events;

import org.yaml.snakeyaml.error.Mark;

public abstract class CollectionStartEvent extends NodeEvent {
    private final Boolean flowStyle;
    private final boolean implicit;
    private final String tag;

    public CollectionStartEvent(String anchor, String tag2, boolean implicit2, Mark startMark, Mark endMark, Boolean flowStyle2) {
        super(anchor, startMark, endMark);
        this.tag = tag2;
        this.implicit = implicit2;
        this.flowStyle = flowStyle2;
    }

    public String getTag() {
        return this.tag;
    }

    public boolean getImplicit() {
        return this.implicit;
    }

    public Boolean getFlowStyle() {
        return this.flowStyle;
    }

    /* access modifiers changed from: protected */
    public String getArguments() {
        return super.getArguments() + ", tag=" + this.tag + ", implicit=" + this.implicit;
    }
}
