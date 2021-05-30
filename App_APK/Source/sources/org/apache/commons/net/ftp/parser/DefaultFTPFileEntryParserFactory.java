package org.apache.commons.net.ftp.parser;

import java.util.Locale;
import org.apache.commons.net.ftp.Configurable;
import org.apache.commons.net.ftp.FTPClientConfig;
import org.apache.commons.net.ftp.FTPFileEntryParser;

public class DefaultFTPFileEntryParserFactory implements FTPFileEntryParserFactory {
    private FTPClientConfig config = null;

    public FTPFileEntryParser createFileEntryParser(String key) {
        FTPFileEntryParser parser;
        if (key != null) {
            Class<?> parserClass = null;
            try {
                parserClass = Class.forName(key);
                parser = (FTPFileEntryParser) parserClass.newInstance();
            } catch (ClassNotFoundException e) {
                String ukey = null;
                if (key != null) {
                    ukey = key.toUpperCase(Locale.ENGLISH);
                }
                if (ukey.indexOf(FTPClientConfig.SYST_UNIX) < 0) {
                    if (ukey.indexOf(FTPClientConfig.SYST_L8) < 0) {
                        if (ukey.indexOf(FTPClientConfig.SYST_VMS) >= 0) {
                            parser = createVMSVersioningFTPEntryParser();
                        } else if (ukey.indexOf(FTPClientConfig.SYST_NT) >= 0) {
                            parser = createNTFTPEntryParser();
                        } else if (ukey.indexOf(FTPClientConfig.SYST_OS2) >= 0) {
                            parser = createOS2FTPEntryParser();
                        } else {
                            if (ukey.indexOf(FTPClientConfig.SYST_OS400) < 0) {
                                if (ukey.indexOf(FTPClientConfig.SYST_AS400) < 0) {
                                    if (ukey.indexOf(FTPClientConfig.SYST_MVS) >= 0) {
                                        parser = createMVSEntryParser();
                                    } else if (ukey.indexOf(FTPClientConfig.SYST_NETWARE) >= 0) {
                                        parser = createNetwareFTPEntryParser();
                                    } else {
                                        throw new ParserInitializationException("Unknown parser type: " + key);
                                    }
                                }
                            }
                            parser = createOS400FTPEntryParser();
                        }
                    }
                }
                parser = createUnixFTPEntryParser();
            } catch (NoClassDefFoundError e2) {
                throw new ParserInitializationException("Error initializing parser", e2);
            } catch (ClassCastException e3) {
                throw new ParserInitializationException(parserClass.getName() + " does not implement the interface " + "org.apache.commons.net.ftp.FTPFileEntryParser.", e3);
            } catch (NoClassDefFoundError nf) {
                throw new ParserInitializationException("Error initializing parser", nf);
            } catch (Throwable e4) {
                throw new ParserInitializationException("Error initializing parser", e4);
            }
            if (parser instanceof Configurable) {
                ((Configurable) parser).configure(this.config);
            }
            return parser;
        }
        throw new ParserInitializationException("Parser key cannot be null");
    }

    public FTPFileEntryParser createFileEntryParser(FTPClientConfig config2) throws ParserInitializationException {
        this.config = config2;
        return createFileEntryParser(config2.getServerSystemKey());
    }

    public FTPFileEntryParser createUnixFTPEntryParser() {
        return new UnixFTPEntryParser();
    }

    public FTPFileEntryParser createVMSVersioningFTPEntryParser() {
        return new VMSVersioningFTPEntryParser();
    }

    public FTPFileEntryParser createNetwareFTPEntryParser() {
        return new NetwareFTPEntryParser();
    }

    public FTPFileEntryParser createNTFTPEntryParser() {
        if (this.config != null && FTPClientConfig.SYST_NT.equals(this.config.getServerSystemKey())) {
            return new NTFTPEntryParser();
        }
        return new CompositeFileEntryParser(new FTPFileEntryParser[]{new NTFTPEntryParser(), new UnixFTPEntryParser()});
    }

    public FTPFileEntryParser createOS2FTPEntryParser() {
        return new OS2FTPEntryParser();
    }

    public FTPFileEntryParser createOS400FTPEntryParser() {
        if (this.config != null && FTPClientConfig.SYST_OS400.equals(this.config.getServerSystemKey())) {
            return new OS400FTPEntryParser();
        }
        return new CompositeFileEntryParser(new FTPFileEntryParser[]{new OS400FTPEntryParser(), new UnixFTPEntryParser()});
    }

    public FTPFileEntryParser createMVSEntryParser() {
        return new MVSFTPEntryParser();
    }
}
