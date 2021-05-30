package com.google.common.net;

import com.google.common.annotations.Beta;
import com.google.common.annotations.GwtCompatible;
import com.google.common.base.Ascii;
import com.google.common.base.CharMatcher;
import com.google.common.base.Charsets;
import com.google.common.base.Function;
import com.google.common.base.Joiner;
import com.google.common.base.Objects;
import com.google.common.base.Optional;
import com.google.common.base.Preconditions;
import com.google.common.collect.ImmutableListMultimap;
import com.google.common.collect.ImmutableMap;
import com.google.common.collect.ImmutableMultiset;
import com.google.common.collect.ImmutableSet;
import com.google.common.collect.Iterables;
import com.google.common.collect.Maps;
import com.google.common.collect.Multimap;
import com.google.common.collect.Multimaps;
import java.nio.charset.Charset;
import java.util.Collection;
import java.util.Iterator;
import java.util.Map;
import javax.annotation.Nullable;
import javax.annotation.concurrent.Immutable;
import org.apache.commons.io.IOUtils;

@GwtCompatible
@Immutable
@Beta
public final class MediaType {
    public static final MediaType ANY_APPLICATION_TYPE = createConstant(APPLICATION_TYPE, "*");
    public static final MediaType ANY_AUDIO_TYPE = createConstant(AUDIO_TYPE, "*");
    public static final MediaType ANY_IMAGE_TYPE = createConstant(IMAGE_TYPE, "*");
    public static final MediaType ANY_TEXT_TYPE = createConstant(TEXT_TYPE, "*");
    public static final MediaType ANY_TYPE = createConstant("*", "*");
    public static final MediaType ANY_VIDEO_TYPE = createConstant(VIDEO_TYPE, "*");
    private static final String APPLICATION_TYPE = "application";
    public static final MediaType ATOM_UTF_8 = createConstantUtf8(APPLICATION_TYPE, "atom+xml");
    private static final String AUDIO_TYPE = "audio";
    public static final MediaType BZIP2 = createConstant(APPLICATION_TYPE, "x-bzip2");
    public static final MediaType CACHE_MANIFEST_UTF_8 = createConstantUtf8(TEXT_TYPE, "cache-manifest");
    private static final String CHARSET_ATTRIBUTE = "charset";
    public static final MediaType CSS_UTF_8 = createConstantUtf8(TEXT_TYPE, "css");
    public static final MediaType CSV_UTF_8 = createConstantUtf8(TEXT_TYPE, "csv");
    public static final MediaType FORM_DATA = createConstant(APPLICATION_TYPE, "x-www-form-urlencoded");
    public static final MediaType GIF = createConstant(IMAGE_TYPE, "gif");
    public static final MediaType GZIP = createConstant(APPLICATION_TYPE, "x-gzip");
    public static final MediaType HTML_UTF_8 = createConstantUtf8(TEXT_TYPE, "html");
    public static final MediaType ICO = createConstant(IMAGE_TYPE, "vnd.microsoft.icon");
    private static final String IMAGE_TYPE = "image";
    public static final MediaType I_CALENDAR_UTF_8 = createConstantUtf8(TEXT_TYPE, "calendar");
    public static final MediaType JAVASCRIPT_UTF_8 = createConstantUtf8(APPLICATION_TYPE, "javascript");
    public static final MediaType JPEG = createConstant(IMAGE_TYPE, "jpeg");
    public static final MediaType JSON_UTF_8 = createConstantUtf8(APPLICATION_TYPE, "json");
    public static final MediaType KML = createConstant(APPLICATION_TYPE, "vnd.google-earth.kml+xml");
    public static final MediaType KMZ = createConstant(APPLICATION_TYPE, "vnd.google-earth.kmz");
    private static final ImmutableMap<MediaType, MediaType> KNOWN_TYPES = new ImmutableMap.Builder().put(ANY_TYPE, ANY_TYPE).put(ANY_TEXT_TYPE, ANY_TEXT_TYPE).put(ANY_IMAGE_TYPE, ANY_IMAGE_TYPE).put(ANY_AUDIO_TYPE, ANY_AUDIO_TYPE).put(ANY_VIDEO_TYPE, ANY_VIDEO_TYPE).put(ANY_APPLICATION_TYPE, ANY_APPLICATION_TYPE).put(CACHE_MANIFEST_UTF_8, CACHE_MANIFEST_UTF_8).put(CSS_UTF_8, CSS_UTF_8).put(CSV_UTF_8, CSV_UTF_8).put(HTML_UTF_8, HTML_UTF_8).put(I_CALENDAR_UTF_8, I_CALENDAR_UTF_8).put(PLAIN_TEXT_UTF_8, PLAIN_TEXT_UTF_8).put(TEXT_JAVASCRIPT_UTF_8, TEXT_JAVASCRIPT_UTF_8).put(VCARD_UTF_8, VCARD_UTF_8).put(XML_UTF_8, XML_UTF_8).put(GIF, GIF).put(ICO, ICO).put(JPEG, JPEG).put(PNG, PNG).put(SVG_UTF_8, SVG_UTF_8).put(TIFF, TIFF).put(MP4_AUDIO, MP4_AUDIO).put(MPEG_AUDIO, MPEG_AUDIO).put(OGG_AUDIO, OGG_AUDIO).put(WEBM_AUDIO, WEBM_AUDIO).put(MP4_VIDEO, MP4_VIDEO).put(MPEG_VIDEO, MPEG_VIDEO).put(OGG_VIDEO, OGG_VIDEO).put(QUICKTIME, QUICKTIME).put(WEBM_VIDEO, WEBM_VIDEO).put(WMV, WMV).put(ATOM_UTF_8, ATOM_UTF_8).put(BZIP2, BZIP2).put(FORM_DATA, FORM_DATA).put(GZIP, GZIP).put(JAVASCRIPT_UTF_8, JAVASCRIPT_UTF_8).put(JSON_UTF_8, JSON_UTF_8).put(KML, KML).put(KMZ, KMZ).put(MICROSOFT_EXCEL, MICROSOFT_EXCEL).put(MICROSOFT_POWERPOINT, MICROSOFT_POWERPOINT).put(MICROSOFT_WORD, MICROSOFT_WORD).put(OCTET_STREAM, OCTET_STREAM).put(OGG_CONTAINER, OGG_CONTAINER).put(OOXML_DOCUMENT, OOXML_DOCUMENT).put(OOXML_PRESENTATION, OOXML_PRESENTATION).put(OOXML_SHEET, OOXML_SHEET).put(OPENDOCUMENT_GRAPHICS, OPENDOCUMENT_GRAPHICS).put(OPENDOCUMENT_PRESENTATION, OPENDOCUMENT_PRESENTATION).put(OPENDOCUMENT_SPREADSHEET, OPENDOCUMENT_SPREADSHEET).put(OPENDOCUMENT_TEXT, OPENDOCUMENT_TEXT).put(PDF, PDF).put(POSTSCRIPT, POSTSCRIPT).put(RTF_UTF_8, RTF_UTF_8).put(SHOCKWAVE_FLASH, SHOCKWAVE_FLASH).put(TAR, TAR).put(XHTML_UTF_8, XHTML_UTF_8).put(ZIP, ZIP).build();
    private static final CharMatcher LINEAR_WHITE_SPACE = CharMatcher.anyOf(" \t\r\n");
    public static final MediaType MICROSOFT_EXCEL = createConstant(APPLICATION_TYPE, "vnd.ms-excel");
    public static final MediaType MICROSOFT_POWERPOINT = createConstant(APPLICATION_TYPE, "vnd.ms-powerpoint");
    public static final MediaType MICROSOFT_WORD = createConstant(APPLICATION_TYPE, "msword");
    public static final MediaType MP4_AUDIO = createConstant(AUDIO_TYPE, "mp4");
    public static final MediaType MP4_VIDEO = createConstant(VIDEO_TYPE, "mp4");
    public static final MediaType MPEG_AUDIO = createConstant(AUDIO_TYPE, "mpeg");
    public static final MediaType MPEG_VIDEO = createConstant(VIDEO_TYPE, "mpeg");
    public static final MediaType OCTET_STREAM = createConstant(APPLICATION_TYPE, "octet-stream");
    public static final MediaType OGG_AUDIO = createConstant(AUDIO_TYPE, "ogg");
    public static final MediaType OGG_CONTAINER = createConstant(APPLICATION_TYPE, "ogg");
    public static final MediaType OGG_VIDEO = createConstant(VIDEO_TYPE, "ogg");
    public static final MediaType OOXML_DOCUMENT = createConstant(APPLICATION_TYPE, "vnd.openxmlformats-officedocument.wordprocessingml.document");
    public static final MediaType OOXML_PRESENTATION = createConstant(APPLICATION_TYPE, "vnd.openxmlformats-officedocument.presentationml.presentation");
    public static final MediaType OOXML_SHEET = createConstant(APPLICATION_TYPE, "vnd.openxmlformats-officedocument.spreadsheetml.sheet");
    public static final MediaType OPENDOCUMENT_GRAPHICS = createConstant(APPLICATION_TYPE, "vnd.oasis.opendocument.graphics");
    public static final MediaType OPENDOCUMENT_PRESENTATION = createConstant(APPLICATION_TYPE, "vnd.oasis.opendocument.presentation");
    public static final MediaType OPENDOCUMENT_SPREADSHEET = createConstant(APPLICATION_TYPE, "vnd.oasis.opendocument.spreadsheet");
    public static final MediaType OPENDOCUMENT_TEXT = createConstant(APPLICATION_TYPE, "vnd.oasis.opendocument.text");
    private static final Joiner.MapJoiner PARAMETER_JOINER = Joiner.on("; ").withKeyValueSeparator("=");
    public static final MediaType PDF = createConstant(APPLICATION_TYPE, "pdf");
    public static final MediaType PLAIN_TEXT_UTF_8 = createConstantUtf8(TEXT_TYPE, "plain");
    public static final MediaType PNG = createConstant(IMAGE_TYPE, "png");
    public static final MediaType POSTSCRIPT = createConstant(APPLICATION_TYPE, "postscript");
    public static final MediaType QUICKTIME = createConstant(VIDEO_TYPE, "quicktime");
    private static final CharMatcher QUOTED_TEXT_MATCHER = CharMatcher.ASCII.and(CharMatcher.noneOf("\"\\\r"));
    public static final MediaType RTF_UTF_8 = createConstantUtf8(APPLICATION_TYPE, "rtf");
    public static final MediaType SHOCKWAVE_FLASH = createConstant(APPLICATION_TYPE, "x-shockwave-flash");
    public static final MediaType SVG_UTF_8 = createConstantUtf8(IMAGE_TYPE, "svg+xml");
    public static final MediaType TAR = createConstant(APPLICATION_TYPE, "x-tar");
    public static final MediaType TEXT_JAVASCRIPT_UTF_8 = createConstantUtf8(TEXT_TYPE, "javascript");
    private static final String TEXT_TYPE = "text";
    public static final MediaType TIFF = createConstant(IMAGE_TYPE, "tiff");
    /* access modifiers changed from: private */
    public static final CharMatcher TOKEN_MATCHER = CharMatcher.ASCII.and(CharMatcher.JAVA_ISO_CONTROL.negate()).and(CharMatcher.isNot(' ')).and(CharMatcher.noneOf("()<>@,;:\\\"/[]?="));
    private static final ImmutableListMultimap<String, String> UTF_8_CONSTANT_PARAMETERS = ImmutableListMultimap.of("charset", Ascii.toLowerCase(Charsets.UTF_8.name()));
    public static final MediaType VCARD_UTF_8 = createConstantUtf8(TEXT_TYPE, "vcard");
    private static final String VIDEO_TYPE = "video";
    public static final MediaType WEBM_AUDIO = createConstant(AUDIO_TYPE, "webm");
    public static final MediaType WEBM_VIDEO = createConstant(VIDEO_TYPE, "webm");
    private static final String WILDCARD = "*";
    public static final MediaType WMV = createConstant(VIDEO_TYPE, "x-ms-wmv");
    public static final MediaType XHTML_UTF_8 = createConstantUtf8(APPLICATION_TYPE, "xhtml+xml");
    public static final MediaType XML_UTF_8 = createConstantUtf8(TEXT_TYPE, "xml");
    public static final MediaType ZIP = createConstant(APPLICATION_TYPE, "zip");
    private final ImmutableListMultimap<String, String> parameters;
    private final String subtype;
    private final String type;

    private MediaType(String type2, String subtype2, ImmutableListMultimap<String, String> parameters2) {
        this.type = type2;
        this.subtype = subtype2;
        this.parameters = parameters2;
    }

    private static MediaType createConstant(String type2, String subtype2) {
        return new MediaType(type2, subtype2, ImmutableListMultimap.of());
    }

    private static MediaType createConstantUtf8(String type2, String subtype2) {
        return new MediaType(type2, subtype2, UTF_8_CONSTANT_PARAMETERS);
    }

    public String type() {
        return this.type;
    }

    public String subtype() {
        return this.subtype;
    }

    public ImmutableListMultimap<String, String> parameters() {
        return this.parameters;
    }

    private Map<String, ImmutableMultiset<String>> parametersAsMap() {
        return Maps.transformValues(this.parameters.asMap(), new Function<Collection<String>, ImmutableMultiset<String>>() {
            public ImmutableMultiset<String> apply(Collection<String> input) {
                return ImmutableMultiset.copyOf(input);
            }
        });
    }

    public Optional<Charset> charset() {
        ImmutableSet<String> charsetValues = ImmutableSet.copyOf(this.parameters.get((Object) "charset"));
        switch (charsetValues.size()) {
            case 0:
                return Optional.absent();
            case 1:
                return Optional.of(Charset.forName((String) Iterables.getOnlyElement(charsetValues)));
            default:
                throw new IllegalStateException("Multiple charset values defined: " + charsetValues);
        }
    }

    public MediaType withoutParameters() {
        return this.parameters.isEmpty() ? this : create(this.type, this.subtype);
    }

    public MediaType withParameters(Multimap<String, String> parameters2) {
        return create(this.type, this.subtype, parameters2);
    }

    public MediaType withParameter(String attribute, String value) {
        Preconditions.checkNotNull(attribute);
        Preconditions.checkNotNull(value);
        String normalizedAttribute = normalizeToken(attribute);
        ImmutableListMultimap.Builder<String, String> builder = ImmutableListMultimap.builder();
        Iterator i$ = this.parameters.entries().iterator();
        while (i$.hasNext()) {
            Map.Entry<String, String> entry = i$.next();
            String key = entry.getKey();
            if (!normalizedAttribute.equals(key)) {
                builder.put(key, entry.getValue());
            }
        }
        builder.put(normalizedAttribute, normalizeParameterValue(normalizedAttribute, value));
        MediaType mediaType = new MediaType(this.type, this.subtype, builder.build());
        return (MediaType) Objects.firstNonNull(KNOWN_TYPES.get(mediaType), mediaType);
    }

    public MediaType withCharset(Charset charset) {
        Preconditions.checkNotNull(charset);
        return withParameter("charset", charset.name());
    }

    public boolean hasWildcard() {
        return "*".equals(this.type) || "*".equals(this.subtype);
    }

    public boolean is(MediaType mediaTypeRange) {
        return (mediaTypeRange.type.equals("*") || mediaTypeRange.type.equals(this.type)) && (mediaTypeRange.subtype.equals("*") || mediaTypeRange.subtype.equals(this.subtype)) && this.parameters.entries().containsAll(mediaTypeRange.parameters.entries());
    }

    public static MediaType create(String type2, String subtype2) {
        return create(type2, subtype2, ImmutableListMultimap.of());
    }

    static MediaType createApplicationType(String subtype2) {
        return create(APPLICATION_TYPE, subtype2);
    }

    static MediaType createAudioType(String subtype2) {
        return create(AUDIO_TYPE, subtype2);
    }

    static MediaType createImageType(String subtype2) {
        return create(IMAGE_TYPE, subtype2);
    }

    static MediaType createTextType(String subtype2) {
        return create(TEXT_TYPE, subtype2);
    }

    static MediaType createVideoType(String subtype2) {
        return create(VIDEO_TYPE, subtype2);
    }

    private static MediaType create(String type2, String subtype2, Multimap<String, String> parameters2) {
        Preconditions.checkNotNull(type2);
        Preconditions.checkNotNull(subtype2);
        Preconditions.checkNotNull(parameters2);
        String normalizedType = normalizeToken(type2);
        String normalizedSubtype = normalizeToken(subtype2);
        Preconditions.checkArgument(!"*".equals(normalizedType) || "*".equals(normalizedSubtype), "A wildcard type cannot be used with a non-wildcard subtype");
        ImmutableListMultimap.Builder<String, String> builder = ImmutableListMultimap.builder();
        for (Map.Entry<String, String> entry : parameters2.entries()) {
            String attribute = normalizeToken(entry.getKey());
            builder.put(attribute, normalizeParameterValue(attribute, entry.getValue()));
        }
        MediaType mediaType = new MediaType(normalizedType, normalizedSubtype, builder.build());
        return (MediaType) Objects.firstNonNull(KNOWN_TYPES.get(mediaType), mediaType);
    }

    private static String normalizeToken(String token) {
        Preconditions.checkArgument(TOKEN_MATCHER.matchesAllOf(token));
        return Ascii.toLowerCase(token);
    }

    private static String normalizeParameterValue(String attribute, String value) {
        return "charset".equals(attribute) ? Ascii.toLowerCase(value) : value;
    }

    public static MediaType parse(String input) {
        String value;
        Preconditions.checkNotNull(input);
        Tokenizer tokenizer = new Tokenizer(input);
        try {
            String type2 = tokenizer.consumeToken(TOKEN_MATCHER);
            tokenizer.consumeCharacter((char) IOUtils.DIR_SEPARATOR_UNIX);
            String subtype2 = tokenizer.consumeToken(TOKEN_MATCHER);
            ImmutableListMultimap.Builder<String, String> parameters2 = ImmutableListMultimap.builder();
            while (tokenizer.hasMore()) {
                tokenizer.consumeCharacter(';');
                tokenizer.consumeTokenIfPresent(LINEAR_WHITE_SPACE);
                String attribute = tokenizer.consumeToken(TOKEN_MATCHER);
                tokenizer.consumeCharacter('=');
                if ('\"' == tokenizer.previewChar()) {
                    tokenizer.consumeCharacter('\"');
                    StringBuilder valueBuilder = new StringBuilder();
                    while ('\"' != tokenizer.previewChar()) {
                        if ('\\' == tokenizer.previewChar()) {
                            tokenizer.consumeCharacter((char) IOUtils.DIR_SEPARATOR_WINDOWS);
                            valueBuilder.append(tokenizer.consumeCharacter(CharMatcher.ASCII));
                        } else {
                            valueBuilder.append(tokenizer.consumeToken(QUOTED_TEXT_MATCHER));
                        }
                    }
                    value = valueBuilder.toString();
                    tokenizer.consumeCharacter('\"');
                } else {
                    value = tokenizer.consumeToken(TOKEN_MATCHER);
                }
                parameters2.put(attribute, value);
            }
            return create(type2, subtype2, parameters2.build());
        } catch (IllegalStateException e) {
            throw new IllegalArgumentException(e);
        }
    }

    private static final class Tokenizer {
        final String input;
        int position = 0;

        Tokenizer(String input2) {
            this.input = input2;
        }

        /* access modifiers changed from: package-private */
        public String consumeTokenIfPresent(CharMatcher matcher) {
            Preconditions.checkState(hasMore());
            int startPosition = this.position;
            this.position = matcher.negate().indexIn(this.input, startPosition);
            return hasMore() ? this.input.substring(startPosition, this.position) : this.input.substring(startPosition);
        }

        /* access modifiers changed from: package-private */
        public String consumeToken(CharMatcher matcher) {
            int startPosition = this.position;
            String token = consumeTokenIfPresent(matcher);
            Preconditions.checkState(this.position != startPosition);
            return token;
        }

        /* access modifiers changed from: package-private */
        public char consumeCharacter(CharMatcher matcher) {
            Preconditions.checkState(hasMore());
            char c = previewChar();
            Preconditions.checkState(matcher.matches(c));
            this.position++;
            return c;
        }

        /* access modifiers changed from: package-private */
        public char consumeCharacter(char c) {
            Preconditions.checkState(hasMore());
            Preconditions.checkState(previewChar() == c);
            this.position++;
            return c;
        }

        /* access modifiers changed from: package-private */
        public char previewChar() {
            Preconditions.checkState(hasMore());
            return this.input.charAt(this.position);
        }

        /* access modifiers changed from: package-private */
        public boolean hasMore() {
            return this.position >= 0 && this.position < this.input.length();
        }
    }

    public boolean equals(@Nullable Object obj) {
        if (obj == this) {
            return true;
        }
        if (!(obj instanceof MediaType)) {
            return false;
        }
        MediaType that = (MediaType) obj;
        if (!this.type.equals(that.type) || !this.subtype.equals(that.subtype) || !parametersAsMap().equals(that.parametersAsMap())) {
            return false;
        }
        return true;
    }

    public int hashCode() {
        return Objects.hashCode(this.type, this.subtype, parametersAsMap());
    }

    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append(this.type);
        sb.append(IOUtils.DIR_SEPARATOR_UNIX);
        StringBuilder builder = sb.append(this.subtype);
        if (!this.parameters.isEmpty()) {
            builder.append("; ");
            PARAMETER_JOINER.appendTo(builder, (Iterable<? extends Map.Entry<?, ?>>) Multimaps.transformValues(this.parameters, new Function<String, String>() {
                public String apply(String value) {
                    return MediaType.TOKEN_MATCHER.matchesAllOf(value) ? value : MediaType.escapeAndQuote(value);
                }
            }).entries());
        }
        return builder.toString();
    }

    /* access modifiers changed from: private */
    public static String escapeAndQuote(String value) {
        StringBuilder escaped = new StringBuilder(value.length() + 16).append('\"');
        for (char ch : value.toCharArray()) {
            if (ch == 13 || ch == '\\' || ch == '\"') {
                escaped.append(IOUtils.DIR_SEPARATOR_WINDOWS);
            }
            escaped.append(ch);
        }
        escaped.append('\"');
        return escaped.toString();
    }
}
