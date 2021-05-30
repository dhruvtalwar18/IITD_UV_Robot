package com.github.robotics_in_concert.rocon_rosjava_core.rocon_interactions;

public enum InteractionMode {
    STANDALONE,
    PAIRED,
    CONCERT;

    public String toString() {
        return name().toLowerCase();
    }
}
