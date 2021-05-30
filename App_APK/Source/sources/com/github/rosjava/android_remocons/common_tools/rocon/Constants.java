package com.github.rosjava.android_remocons.common_tools.rocon;

import org.ros.internal.message.DefaultMessageFactory;
import org.ros.internal.message.definition.MessageDefinitionReflectionProvider;
import rocon_std_msgs.PlatformInfo;
import rocon_std_msgs.Strings;

public class Constants {
    public static final String ACTIVITY_ROCON_REMOCON = "com.github.rosjava.android_remocons.rocon_remocon.Remocon";
    public static final String ACTIVITY_SWITCHER_ID = "com.github.rosjava.android_remocons.common_tools.rocon.Constants";
    public static final PlatformInfo ANDROID_PLATFORM_INFO = makePlatformInfo();
    public static final int NFC_APP_HASH_FIELD_LENGTH = 4;
    public static final int NFC_APP_RECORD_FIELD_LENGTH = 56;
    public static final int NFC_EXTRA_DATA_FIELD_LENGTH = 2;
    public static final int NFC_MASTER_HOST_FIELD_LENGTH = 16;
    public static final int NFC_MASTER_PORT_FIELD_LENGTH = 2;
    public static final int NFC_PASSWORD_FIELD_LENGTH = 16;
    public static final int NFC_PAYLOAD_LENGTH = 56;
    public static final int NFC_SSID_FIELD_LENGTH = 16;
    public static final int NFC_ULTRALIGHT_C_MAX_LENGTH = 137;

    private static PlatformInfo makePlatformInfo() {
        PlatformInfo platformInfo = (PlatformInfo) new DefaultMessageFactory(new MessageDefinitionReflectionProvider()).newFromType(PlatformInfo._TYPE);
        platformInfo.setUri("rocon:/*/*/indigo/ice_cream_sandwich|jellybean|chrome");
        platformInfo.setVersion(Strings.ROCON_VERSION);
        return platformInfo;
    }
}
