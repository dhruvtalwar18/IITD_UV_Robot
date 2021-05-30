package org.ros;

import org.ros.namespace.GraphName;

public interface Topics {
    public static final GraphName CLOCK = GraphName.of("/clock");
    public static final GraphName ROSOUT = GraphName.of("/rosout");
}
