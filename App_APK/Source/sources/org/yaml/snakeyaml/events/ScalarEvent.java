package org.yaml.snakeyaml.events;

import org.yaml.snakeyaml.error.Mark;
import org.yaml.snakeyaml.events.Event;

public final class ScalarEvent extends NodeEvent {
    private final ImplicitTuple implicit;
    private final Character style;
    private final String tag;
    private final String value;

    public ScalarEvent(String anchor, String tag2, ImplicitTuple implicit2, String value2, Mark startMark, Mark endMark, Character style2) {
        super(anchor, startMark, endMark);
        this.tag = tag2;
        this.implicit = implicit2;
        this.value = value2;
        this.style = style2;
    }

    public String getTag() {
        return this.tag;
    }

    public Character getStyle() {
        return this.style;
    }

    public String getValue() {
        return this.value;
    }

    public ImplicitTuple getImplicit() {
        return this.implicit;
    }

    /* access modifiers changed from: protected */
    public String getArguments() {
        return super.getArguments() + ", tag=" + this.tag + ", " + this.implicit + ", value=" + this.value;
    }

    public boolean is(Event.ID id) {
        return Event.ID.Scalar == id;
    }
}
