package rocon_interaction_msgs;

import java.util.List;
import org.ros.internal.message.Message;
import rocon_std_msgs.Icon;
import rocon_std_msgs.Remapping;

public interface Interaction extends Message {
    public static final int UNLIMITED_INTERACTIONS = -1;
    public static final String _DEFINITION = "###############################################################\n# Non-modifiable Specifications\n###############################################################\n# These should be stored in a meta-repository somewhere.\n#\n# Name of the interaction. This should be the string\n# required to install and execute the application on the remocon's\n# end. e.g. for android app this will be an intent action name such as\n# com.github.rosjava.android_apps.listener.Listener, while for\n# a web app it will be its URL.\nstring name\n\n# A rocon uri string denoting the platforms this interaction\n# is compatible with\nstring compatibility\n\n###############################################################\n# Variable Specifications\n###############################################################\n\n# User friendly version of the name. Useful to be customised\n# differently from the name for either 1) branding, or 2) because\n# some names are complicated visually (e.g. android names).\nstring display_name\n \n# Should be a default for the interaction, but sometimes useful to\n# override it to provide more human friendly information.\nstring description\n\n# The namespace that this interaction will be associated\n# with. Interfaces from the interaction will be automatically pushed\n# into this namespace (for concerts it will be typically\n# used by the services to push interfaces into /services/_service_name_).\n# It is up to the user to make sure this is unique to avoid\n# potential conflicts.\nstring namespace\n\n# Again, should exist a default, but may want to override it.\nrocon_std_msgs/Icon icon\n\n# Any remappings that need to be applied\nrocon_std_msgs/Remapping[] remappings\n\n# Yaml string representing parameters for the interaction\nstring parameters\n\n# Maximum number of permitted connections (-1 = any)\nint32 UNLIMITED_INTERACTIONS = -1\nint32 max\n\n# The configuration for a pairing if this interaction is to be paired\n# with a rapp. If not, it should be left unfilled.\nPairing pairing\n\n###############################################################\n# Parameters finalised by the interactions manager\n###############################################################\n\n# This is a crc32 hash code for the service-role-name\n# unique string that identifies this interaction.\n# crc32 gets a few collisions, so we should be careful of that.\n# It is used by the nfc android auto-launching program so we can\n# compactify the request in the url sent by the nfc to the autolauncher.\nint32 hash\n\n# The role this solution has configured the interaction for. This is in\n# some use cases redundant, and in other use cases essential (e.g.\n# headless launcher)\nstring role";
    public static final String _TYPE = "rocon_interaction_msgs/Interaction";

    String getCompatibility();

    String getDescription();

    String getDisplayName();

    int getHash();

    Icon getIcon();

    int getMax();

    String getName();

    String getNamespace();

    Pairing getPairing();

    String getParameters();

    List<Remapping> getRemappings();

    String getRole();

    void setCompatibility(String str);

    void setDescription(String str);

    void setDisplayName(String str);

    void setHash(int i);

    void setIcon(Icon icon);

    void setMax(int i);

    void setName(String str);

    void setNamespace(String str);

    void setPairing(Pairing pairing);

    void setParameters(String str);

    void setRemappings(List<Remapping> list);

    void setRole(String str);
}
