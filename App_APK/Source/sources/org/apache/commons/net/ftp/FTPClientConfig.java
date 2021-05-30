package org.apache.commons.net.ftp;

import java.text.DateFormatSymbols;
import java.util.Collection;
import java.util.Locale;
import java.util.Map;
import java.util.StringTokenizer;
import java.util.TreeMap;

public class FTPClientConfig {
    private static Map<String, Object> LANGUAGE_CODE_MAP = new TreeMap();
    public static final String SYST_AS400 = "AS/400";
    public static final String SYST_L8 = "TYPE: L8";
    public static final String SYST_MVS = "MVS";
    public static final String SYST_NETWARE = "NETWARE";
    public static final String SYST_NT = "WINDOWS";
    public static final String SYST_OS2 = "OS/2";
    public static final String SYST_OS400 = "OS/400";
    public static final String SYST_UNIX = "UNIX";
    public static final String SYST_VMS = "VMS";
    private String defaultDateFormatStr;
    private boolean lenientFutureDates;
    private String recentDateFormatStr;
    private String serverLanguageCode;
    private final String serverSystemKey;
    private String serverTimeZoneId;
    private String shortMonthNames;

    public FTPClientConfig(String systemKey) {
        this.defaultDateFormatStr = null;
        this.recentDateFormatStr = null;
        this.lenientFutureDates = false;
        this.serverLanguageCode = null;
        this.shortMonthNames = null;
        this.serverTimeZoneId = null;
        this.serverSystemKey = systemKey;
    }

    public FTPClientConfig() {
        this(SYST_UNIX);
    }

    public FTPClientConfig(String systemKey, String defaultDateFormatStr2, String recentDateFormatStr2, String serverLanguageCode2, String shortMonthNames2, String serverTimeZoneId2) {
        this(systemKey);
        this.defaultDateFormatStr = defaultDateFormatStr2;
        this.recentDateFormatStr = recentDateFormatStr2;
        this.serverLanguageCode = serverLanguageCode2;
        this.shortMonthNames = shortMonthNames2;
        this.serverTimeZoneId = serverTimeZoneId2;
    }

    static {
        LANGUAGE_CODE_MAP.put("en", Locale.ENGLISH);
        LANGUAGE_CODE_MAP.put("de", Locale.GERMAN);
        LANGUAGE_CODE_MAP.put("it", Locale.ITALIAN);
        LANGUAGE_CODE_MAP.put("es", new Locale("es", "", ""));
        LANGUAGE_CODE_MAP.put("pt", new Locale("pt", "", ""));
        LANGUAGE_CODE_MAP.put("da", new Locale("da", "", ""));
        LANGUAGE_CODE_MAP.put("sv", new Locale("sv", "", ""));
        LANGUAGE_CODE_MAP.put("no", new Locale("no", "", ""));
        LANGUAGE_CODE_MAP.put("nl", new Locale("nl", "", ""));
        LANGUAGE_CODE_MAP.put("ro", new Locale("ro", "", ""));
        LANGUAGE_CODE_MAP.put("sq", new Locale("sq", "", ""));
        LANGUAGE_CODE_MAP.put("sh", new Locale("sh", "", ""));
        LANGUAGE_CODE_MAP.put("sk", new Locale("sk", "", ""));
        LANGUAGE_CODE_MAP.put("sl", new Locale("sl", "", ""));
        LANGUAGE_CODE_MAP.put("fr", "jan|fév|mar|avr|mai|jun|jui|aoû|sep|oct|nov|déc");
    }

    public String getServerSystemKey() {
        return this.serverSystemKey;
    }

    public String getDefaultDateFormatStr() {
        return this.defaultDateFormatStr;
    }

    public String getRecentDateFormatStr() {
        return this.recentDateFormatStr;
    }

    public String getServerTimeZoneId() {
        return this.serverTimeZoneId;
    }

    public String getShortMonthNames() {
        return this.shortMonthNames;
    }

    public String getServerLanguageCode() {
        return this.serverLanguageCode;
    }

    public boolean isLenientFutureDates() {
        return this.lenientFutureDates;
    }

    public void setDefaultDateFormatStr(String defaultDateFormatStr2) {
        this.defaultDateFormatStr = defaultDateFormatStr2;
    }

    public void setRecentDateFormatStr(String recentDateFormatStr2) {
        this.recentDateFormatStr = recentDateFormatStr2;
    }

    public void setLenientFutureDates(boolean lenientFutureDates2) {
        this.lenientFutureDates = lenientFutureDates2;
    }

    public void setServerTimeZoneId(String serverTimeZoneId2) {
        this.serverTimeZoneId = serverTimeZoneId2;
    }

    public void setShortMonthNames(String shortMonthNames2) {
        this.shortMonthNames = shortMonthNames2;
    }

    public void setServerLanguageCode(String serverLanguageCode2) {
        this.serverLanguageCode = serverLanguageCode2;
    }

    public static DateFormatSymbols lookupDateFormatSymbols(String languageCode) {
        Object lang = LANGUAGE_CODE_MAP.get(languageCode);
        if (lang != null) {
            if (lang instanceof Locale) {
                return new DateFormatSymbols((Locale) lang);
            }
            if (lang instanceof String) {
                return getDateFormatSymbols((String) lang);
            }
        }
        return new DateFormatSymbols(Locale.US);
    }

    public static DateFormatSymbols getDateFormatSymbols(String shortmonths) {
        String[] months = splitShortMonthString(shortmonths);
        DateFormatSymbols dfs = new DateFormatSymbols(Locale.US);
        dfs.setShortMonths(months);
        return dfs;
    }

    private static String[] splitShortMonthString(String shortmonths) {
        StringTokenizer st = new StringTokenizer(shortmonths, "|");
        if (12 == st.countTokens()) {
            String[] months = new String[13];
            int pos = 0;
            while (st.hasMoreTokens()) {
                months[pos] = st.nextToken();
                pos++;
            }
            months[pos] = "";
            return months;
        }
        throw new IllegalArgumentException("expecting a pipe-delimited string containing 12 tokens");
    }

    public static Collection<String> getSupportedLanguageCodes() {
        return LANGUAGE_CODE_MAP.keySet();
    }
}
