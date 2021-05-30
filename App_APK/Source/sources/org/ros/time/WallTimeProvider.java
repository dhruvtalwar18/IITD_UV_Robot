package org.ros.time;

import org.ros.message.Time;

public class WallTimeProvider implements TimeProvider {
    public Time getCurrentTime() {
        return Time.fromMillis(System.currentTimeMillis());
    }
}
