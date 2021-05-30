package org.apache.commons.net.ntp;

import java.util.ArrayList;
import java.util.List;

public class TimeInfo {
    private List<String> _comments;
    private Long _delay;
    private boolean _detailsComputed;
    private NtpV3Packet _message;
    private Long _offset;
    private long _returnTime;

    public TimeInfo(NtpV3Packet message, long returnTime) {
        this(message, returnTime, (List<String>) null, true);
    }

    public TimeInfo(NtpV3Packet message, long returnTime, List<String> comments) {
        this(message, returnTime, comments, true);
    }

    public TimeInfo(NtpV3Packet msgPacket, long returnTime, boolean doComputeDetails) {
        this(msgPacket, returnTime, (List<String>) null, doComputeDetails);
    }

    public TimeInfo(NtpV3Packet message, long returnTime, List<String> comments, boolean doComputeDetails) {
        if (message != null) {
            this._returnTime = returnTime;
            this._message = message;
            this._comments = comments;
            if (doComputeDetails) {
                computeDetails();
                return;
            }
            return;
        }
        throw new IllegalArgumentException("message cannot be null");
    }

    public void addComment(String comment) {
        if (this._comments == null) {
            this._comments = new ArrayList();
        }
        this._comments.add(comment);
    }

    public void computeDetails() {
        if (!this._detailsComputed) {
            this._detailsComputed = true;
            if (this._comments == null) {
                this._comments = new ArrayList();
            }
            TimeStamp origNtpTime = this._message.getOriginateTimeStamp();
            long origTime = origNtpTime.getTime();
            TimeStamp rcvNtpTime = this._message.getReceiveTimeStamp();
            long rcvTime = rcvNtpTime.getTime();
            TimeStamp xmitNtpTime = this._message.getTransmitTimeStamp();
            long xmitTime = xmitNtpTime.getTime();
            if (origNtpTime.ntpValue() == 0) {
                if (xmitNtpTime.ntpValue() != 0) {
                    this._offset = Long.valueOf(xmitTime - this._returnTime);
                    this._comments.add("Error: zero orig time -- cannot compute delay");
                    return;
                }
                this._comments.add("Error: zero orig time -- cannot compute delay/offset");
            } else if (rcvNtpTime.ntpValue() == 0 || xmitNtpTime.ntpValue() == 0) {
                this._comments.add("Warning: zero rcvNtpTime or xmitNtpTime");
                if (origTime > this._returnTime) {
                    this._comments.add("Error: OrigTime > DestRcvTime");
                } else {
                    this._delay = Long.valueOf(this._returnTime - origTime);
                }
                if (rcvNtpTime.ntpValue() != 0) {
                    this._offset = Long.valueOf(rcvTime - origTime);
                } else if (xmitNtpTime.ntpValue() != 0) {
                    this._offset = Long.valueOf(xmitTime - this._returnTime);
                }
            } else {
                long delayValue = this._returnTime - origTime;
                if (xmitTime < rcvTime) {
                    this._comments.add("Error: xmitTime < rcvTime");
                } else {
                    long delta = xmitTime - rcvTime;
                    if (delta <= delayValue) {
                        delayValue -= delta;
                    } else if (delta - delayValue != 1) {
                        this._comments.add("Warning: processing time > total network time");
                    } else if (delayValue != 0) {
                        this._comments.add("Info: processing time > total network time by 1 ms -> assume zero delay");
                        delayValue = 0;
                    }
                }
                this._delay = Long.valueOf(delayValue);
                if (origTime > this._returnTime) {
                    this._comments.add("Error: OrigTime > DestRcvTime");
                }
                this._offset = Long.valueOf(((rcvTime - origTime) + (xmitTime - this._returnTime)) / 2);
            }
        }
    }

    public List<String> getComments() {
        return this._comments;
    }

    public Long getDelay() {
        return this._delay;
    }

    public Long getOffset() {
        return this._offset;
    }

    public NtpV3Packet getMessage() {
        return this._message;
    }

    public long getReturnTime() {
        return this._returnTime;
    }
}
