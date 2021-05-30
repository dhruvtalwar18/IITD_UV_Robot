package android.support.v4.content;

import android.support.annotation.NonNull;
import android.support.annotation.Nullable;
import java.util.ArrayList;
import org.apache.commons.httpclient.cookie.CookieSpec;

public final class MimeTypeFilter {
    private MimeTypeFilter() {
    }

    private static boolean mimeTypeAgainstFilter(@NonNull String[] mimeTypeParts, @NonNull String[] filterParts) {
        if (filterParts.length != 2) {
            throw new IllegalArgumentException("Ill-formatted MIME type filter. Must be type/subtype.");
        } else if (filterParts[0].isEmpty() || filterParts[1].isEmpty()) {
            throw new IllegalArgumentException("Ill-formatted MIME type filter. Type or subtype empty.");
        } else if (mimeTypeParts.length != 2) {
            return false;
        } else {
            if ("*".equals(filterParts[0]) || filterParts[0].equals(mimeTypeParts[0])) {
                return "*".equals(filterParts[1]) || filterParts[1].equals(mimeTypeParts[1]);
            }
            return false;
        }
    }

    public static boolean matches(@Nullable String mimeType, @NonNull String filter) {
        if (mimeType == null) {
            return false;
        }
        return mimeTypeAgainstFilter(mimeType.split(CookieSpec.PATH_DELIM), filter.split(CookieSpec.PATH_DELIM));
    }

    @Nullable
    public static String matches(@Nullable String mimeType, @NonNull String[] filters) {
        if (mimeType == null) {
            return null;
        }
        String[] mimeTypeParts = mimeType.split(CookieSpec.PATH_DELIM);
        for (String filter : filters) {
            if (mimeTypeAgainstFilter(mimeTypeParts, filter.split(CookieSpec.PATH_DELIM))) {
                return filter;
            }
        }
        return null;
    }

    @Nullable
    public static String matches(@Nullable String[] mimeTypes, @NonNull String filter) {
        if (mimeTypes == null) {
            return null;
        }
        String[] filterParts = filter.split(CookieSpec.PATH_DELIM);
        for (String mimeType : mimeTypes) {
            if (mimeTypeAgainstFilter(mimeType.split(CookieSpec.PATH_DELIM), filterParts)) {
                return mimeType;
            }
        }
        return null;
    }

    @NonNull
    public static String[] matchesMany(@Nullable String[] mimeTypes, @NonNull String filter) {
        if (mimeTypes == null) {
            return new String[0];
        }
        ArrayList<String> list = new ArrayList<>();
        String[] filterParts = filter.split(CookieSpec.PATH_DELIM);
        for (String mimeType : mimeTypes) {
            if (mimeTypeAgainstFilter(mimeType.split(CookieSpec.PATH_DELIM), filterParts)) {
                list.add(mimeType);
            }
        }
        return (String[]) list.toArray(new String[list.size()]);
    }
}