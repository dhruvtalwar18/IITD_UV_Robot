package org.ros.message;

public class Time implements Comparable<Time> {
    public int nsecs;
    public int secs;

    public Time() {
        this.secs = 0;
        this.nsecs = 0;
    }

    public Time(int secs2, int nsecs2) {
        this.secs = secs2;
        this.nsecs = nsecs2;
        normalize();
    }

    public Time(double secs2) {
        this.secs = (int) secs2;
        double d = (double) this.secs;
        Double.isNaN(d);
        this.nsecs = (int) ((secs2 - d) * 1.0E9d);
        normalize();
    }

    public Time(Time t) {
        this.secs = t.secs;
        this.nsecs = t.nsecs;
    }

    public Time add(Duration d) {
        return new Time(this.secs + d.secs, this.nsecs + d.nsecs);
    }

    public Time subtract(Duration d) {
        return new Time(this.secs - d.secs, this.nsecs - d.nsecs);
    }

    public Duration subtract(Time t) {
        return new Duration(this.secs - t.secs, this.nsecs - t.nsecs);
    }

    public static Time fromMillis(long timeInMillis) {
        return new Time((int) (timeInMillis / 1000), ((int) (timeInMillis % 1000)) * 1000000);
    }

    public static Time fromNano(long timeInNs) {
        return new Time((int) (timeInNs / 1000000000), (int) (timeInNs % 1000000000));
    }

    public String toString() {
        return this.secs + ":" + this.nsecs;
    }

    public double toSeconds() {
        double d = (double) totalNsecs();
        Double.isNaN(d);
        return d / 1.0E9d;
    }

    public long totalNsecs() {
        return (((long) this.secs) * 1000000000) + ((long) this.nsecs);
    }

    public boolean isZero() {
        return totalNsecs() == 0;
    }

    public void normalize() {
        while (this.nsecs < 0) {
            this.nsecs += 1000000000;
            this.secs--;
        }
        while (this.nsecs >= 1000000000) {
            this.nsecs -= 1000000000;
            this.secs++;
        }
    }

    public int hashCode() {
        return (((1 * 31) + this.nsecs) * 31) + this.secs;
    }

    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (obj == null || getClass() != obj.getClass()) {
            return false;
        }
        Time other = (Time) obj;
        if (this.nsecs == other.nsecs && this.secs == other.secs) {
            return true;
        }
        return false;
    }

    public int compareTo(Time t) {
        if (this.secs > t.secs) {
            return 1;
        }
        if (this.secs == t.secs && this.nsecs > t.nsecs) {
            return 1;
        }
        if (this.secs == t.secs && this.nsecs == t.nsecs) {
            return 0;
        }
        return -1;
    }
}
