package org.apache.commons.net.ftp.parser;

import java.text.DateFormatSymbols;
import java.text.ParseException;
import java.text.ParsePosition;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;
import java.util.TimeZone;
import org.apache.commons.net.ftp.Configurable;
import org.apache.commons.net.ftp.FTPClientConfig;

public class FTPTimestampParserImpl implements FTPTimestampParser, Configurable {
    private SimpleDateFormat defaultDateFormat;
    private boolean lenientFutureDates = false;
    private SimpleDateFormat recentDateFormat;

    public FTPTimestampParserImpl() {
        setDefaultDateFormat(FTPTimestampParser.DEFAULT_SDF);
        setRecentDateFormat(FTPTimestampParser.DEFAULT_RECENT_SDF);
    }

    public Calendar parseTimestamp(String timestampStr) throws ParseException {
        return parseTimestamp(timestampStr, Calendar.getInstance());
    }

    public Calendar parseTimestamp(String timestampStr, Calendar serverTime) throws ParseException {
        Calendar now = (Calendar) serverTime.clone();
        now.setTimeZone(getServerTimeZone());
        Calendar working = (Calendar) now.clone();
        working.setTimeZone(getServerTimeZone());
        ParsePosition pp = new ParsePosition(0);
        Date parsed = null;
        if (this.recentDateFormat != null) {
            if (this.lenientFutureDates) {
                now.add(5, 1);
            }
            parsed = this.recentDateFormat.parse(timestampStr, pp);
        }
        if (parsed == null || pp.getIndex() != timestampStr.length()) {
            if (this.recentDateFormat != null) {
                pp = new ParsePosition(0);
                SimpleDateFormat hackFormatter = new SimpleDateFormat(this.recentDateFormat.toPattern() + " yyyy", this.recentDateFormat.getDateFormatSymbols());
                hackFormatter.setLenient(false);
                hackFormatter.setTimeZone(this.recentDateFormat.getTimeZone());
                parsed = hackFormatter.parse(timestampStr + " " + now.get(1), pp);
            }
            if (parsed == null || pp.getIndex() != timestampStr.length() + 5) {
                ParsePosition pp2 = new ParsePosition(0);
                Date parsed2 = this.defaultDateFormat.parse(timestampStr, pp2);
                if (parsed2 == null || pp2.getIndex() != timestampStr.length()) {
                    throw new ParseException("Timestamp could not be parsed with older or recent DateFormat", pp2.getIndex());
                }
                working.setTime(parsed2);
            } else {
                working.setTime(parsed);
            }
        } else {
            working.setTime(parsed);
            working.set(1, now.get(1));
            if (working.after(now)) {
                working.add(1, -1);
            }
        }
        return working;
    }

    public SimpleDateFormat getDefaultDateFormat() {
        return this.defaultDateFormat;
    }

    public String getDefaultDateFormatString() {
        return this.defaultDateFormat.toPattern();
    }

    private void setDefaultDateFormat(String format) {
        if (format != null) {
            this.defaultDateFormat = new SimpleDateFormat(format);
            this.defaultDateFormat.setLenient(false);
        }
    }

    public SimpleDateFormat getRecentDateFormat() {
        return this.recentDateFormat;
    }

    public String getRecentDateFormatString() {
        return this.recentDateFormat.toPattern();
    }

    private void setRecentDateFormat(String format) {
        if (format != null) {
            this.recentDateFormat = new SimpleDateFormat(format);
            this.recentDateFormat.setLenient(false);
        }
    }

    public String[] getShortMonths() {
        return this.defaultDateFormat.getDateFormatSymbols().getShortMonths();
    }

    public TimeZone getServerTimeZone() {
        return this.defaultDateFormat.getTimeZone();
    }

    private void setServerTimeZone(String serverTimeZoneId) {
        TimeZone serverTimeZone = TimeZone.getDefault();
        if (serverTimeZoneId != null) {
            serverTimeZone = TimeZone.getTimeZone(serverTimeZoneId);
        }
        this.defaultDateFormat.setTimeZone(serverTimeZone);
        if (this.recentDateFormat != null) {
            this.recentDateFormat.setTimeZone(serverTimeZone);
        }
    }

    public void configure(FTPClientConfig config) {
        DateFormatSymbols dfs;
        String languageCode = config.getServerLanguageCode();
        String shortmonths = config.getShortMonthNames();
        if (shortmonths != null) {
            dfs = FTPClientConfig.getDateFormatSymbols(shortmonths);
        } else if (languageCode != null) {
            dfs = FTPClientConfig.lookupDateFormatSymbols(languageCode);
        } else {
            dfs = FTPClientConfig.lookupDateFormatSymbols("en");
        }
        String recentFormatString = config.getRecentDateFormatStr();
        if (recentFormatString == null) {
            this.recentDateFormat = null;
        } else {
            this.recentDateFormat = new SimpleDateFormat(recentFormatString, dfs);
            this.recentDateFormat.setLenient(false);
        }
        String defaultFormatString = config.getDefaultDateFormatStr();
        if (defaultFormatString != null) {
            this.defaultDateFormat = new SimpleDateFormat(defaultFormatString, dfs);
            this.defaultDateFormat.setLenient(false);
            setServerTimeZone(config.getServerTimeZoneId());
            this.lenientFutureDates = config.isLenientFutureDates();
            return;
        }
        throw new IllegalArgumentException("defaultFormatString cannot be null");
    }

    /* access modifiers changed from: package-private */
    public boolean isLenientFutureDates() {
        return this.lenientFutureDates;
    }

    /* access modifiers changed from: package-private */
    public void setLenientFutureDates(boolean lenientFutureDates2) {
        this.lenientFutureDates = lenientFutureDates2;
    }
}
