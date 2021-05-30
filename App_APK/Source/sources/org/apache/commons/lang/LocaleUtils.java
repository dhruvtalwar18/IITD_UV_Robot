package org.apache.commons.lang;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;

public class LocaleUtils {
    private static final List cAvailableLocaleList = Collections.unmodifiableList(Arrays.asList(Locale.getAvailableLocales()));
    private static Set cAvailableLocaleSet;
    private static final Map cCountriesByLanguage = Collections.synchronizedMap(new HashMap());
    private static final Map cLanguagesByCountry = Collections.synchronizedMap(new HashMap());

    public static Locale toLocale(String str) {
        if (str == null) {
            return null;
        }
        int len = str.length();
        if (len == 2 || len == 5 || len >= 7) {
            char ch0 = str.charAt(0);
            char ch1 = str.charAt(1);
            if (ch0 < 'a' || ch0 > 'z' || ch1 < 'a' || ch1 > 'z') {
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append("Invalid locale format: ");
                stringBuffer.append(str);
                throw new IllegalArgumentException(stringBuffer.toString());
            } else if (len == 2) {
                return new Locale(str, "");
            } else {
                if (str.charAt(2) == '_') {
                    char ch3 = str.charAt(3);
                    if (ch3 == '_') {
                        return new Locale(str.substring(0, 2), "", str.substring(4));
                    }
                    char ch4 = str.charAt(4);
                    if (ch3 < 'A' || ch3 > 'Z' || ch4 < 'A' || ch4 > 'Z') {
                        StringBuffer stringBuffer2 = new StringBuffer();
                        stringBuffer2.append("Invalid locale format: ");
                        stringBuffer2.append(str);
                        throw new IllegalArgumentException(stringBuffer2.toString());
                    } else if (len == 5) {
                        return new Locale(str.substring(0, 2), str.substring(3, 5));
                    } else {
                        if (str.charAt(5) == '_') {
                            return new Locale(str.substring(0, 2), str.substring(3, 5), str.substring(6));
                        }
                        StringBuffer stringBuffer3 = new StringBuffer();
                        stringBuffer3.append("Invalid locale format: ");
                        stringBuffer3.append(str);
                        throw new IllegalArgumentException(stringBuffer3.toString());
                    }
                } else {
                    StringBuffer stringBuffer4 = new StringBuffer();
                    stringBuffer4.append("Invalid locale format: ");
                    stringBuffer4.append(str);
                    throw new IllegalArgumentException(stringBuffer4.toString());
                }
            }
        } else {
            StringBuffer stringBuffer5 = new StringBuffer();
            stringBuffer5.append("Invalid locale format: ");
            stringBuffer5.append(str);
            throw new IllegalArgumentException(stringBuffer5.toString());
        }
    }

    public static List localeLookupList(Locale locale) {
        return localeLookupList(locale, locale);
    }

    public static List localeLookupList(Locale locale, Locale defaultLocale) {
        List list = new ArrayList(4);
        if (locale != null) {
            list.add(locale);
            if (locale.getVariant().length() > 0) {
                list.add(new Locale(locale.getLanguage(), locale.getCountry()));
            }
            if (locale.getCountry().length() > 0) {
                list.add(new Locale(locale.getLanguage(), ""));
            }
            if (!list.contains(defaultLocale)) {
                list.add(defaultLocale);
            }
        }
        return Collections.unmodifiableList(list);
    }

    public static List availableLocaleList() {
        return cAvailableLocaleList;
    }

    public static Set availableLocaleSet() {
        Set set = cAvailableLocaleSet;
        if (set != null) {
            return set;
        }
        Set set2 = Collections.unmodifiableSet(new HashSet(availableLocaleList()));
        cAvailableLocaleSet = set2;
        return set2;
    }

    public static boolean isAvailableLocale(Locale locale) {
        return availableLocaleList().contains(locale);
    }

    public static List languagesByCountry(String countryCode) {
        List langs = (List) cLanguagesByCountry.get(countryCode);
        if (langs == null) {
            if (countryCode != null) {
                List langs2 = new ArrayList();
                List locales = availableLocaleList();
                for (int i = 0; i < locales.size(); i++) {
                    Locale locale = (Locale) locales.get(i);
                    if (countryCode.equals(locale.getCountry()) && locale.getVariant().length() == 0) {
                        langs2.add(locale);
                    }
                }
                langs = Collections.unmodifiableList(langs2);
            } else {
                langs = Collections.EMPTY_LIST;
            }
            cLanguagesByCountry.put(countryCode, langs);
        }
        return langs;
    }

    public static List countriesByLanguage(String languageCode) {
        List countries = (List) cCountriesByLanguage.get(languageCode);
        if (countries == null) {
            if (languageCode != null) {
                List countries2 = new ArrayList();
                List locales = availableLocaleList();
                for (int i = 0; i < locales.size(); i++) {
                    Locale locale = (Locale) locales.get(i);
                    if (languageCode.equals(locale.getLanguage()) && locale.getCountry().length() != 0 && locale.getVariant().length() == 0) {
                        countries2.add(locale);
                    }
                }
                countries = Collections.unmodifiableList(countries2);
            } else {
                countries = Collections.EMPTY_LIST;
            }
            cCountriesByLanguage.put(languageCode, countries);
        }
        return countries;
    }
}
