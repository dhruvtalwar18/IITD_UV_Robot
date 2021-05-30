package org.yaml.snakeyaml.parser;

import org.yaml.snakeyaml.events.Event;

public interface Parser {
    boolean checkEvent(Event.ID id);

    Event getEvent();

    Event peekEvent();
}
