package org.apache.commons.net.ftp.parser;

import java.text.ParseException;
import java.util.Calendar;
import org.apache.commons.net.ftp.Configurable;
import org.apache.commons.net.ftp.FTPClientConfig;

public abstract class ConfigurableFTPFileEntryParserImpl extends RegexFTPFileEntryParserImpl implements Configurable {
    private FTPTimestampParser timestampParser = new FTPTimestampParserImpl();

    /* access modifiers changed from: protected */
    public abstract FTPClientConfig getDefaultConfiguration();

    public ConfigurableFTPFileEntryParserImpl(String regex) {
        super(regex);
    }

    public Calendar parseTimestamp(String timestampStr) throws ParseException {
        return this.timestampParser.parseTimestamp(timestampStr);
    }

    public void configure(FTPClientConfig config) {
        if (this.timestampParser instanceof Configurable) {
            FTPClientConfig defaultCfg = getDefaultConfiguration();
            if (config != null) {
                if (config.getDefaultDateFormatStr() == null) {
                    config.setDefaultDateFormatStr(defaultCfg.getDefaultDateFormatStr());
                }
                if (config.getRecentDateFormatStr() == null) {
                    config.setRecentDateFormatStr(defaultCfg.getRecentDateFormatStr());
                }
                ((Configurable) this.timestampParser).configure(config);
                return;
            }
            ((Configurable) this.timestampParser).configure(defaultCfg);
        }
    }
}
