package rosgraph_msgs;

import java.util.List;
import org.ros.internal.message.Message;
import std_msgs.Header;

public interface Log extends Message {
    public static final byte DEBUG = 1;
    public static final byte ERROR = 8;
    public static final byte FATAL = 16;
    public static final byte INFO = 2;
    public static final byte WARN = 4;
    public static final String _DEFINITION = "##\n## Severity level constants\n##\nbyte DEBUG=1 #debug level\nbyte INFO=2  #general level\nbyte WARN=4  #warning level\nbyte ERROR=8 #error level\nbyte FATAL=16 #fatal/critical level\n##\n## Fields\n##\nHeader header\nbyte level\nstring name # name of the node\nstring msg # message \nstring file # file the message came from\nstring function # function the message came from\nuint32 line # line the message came from\nstring[] topics # topic names that the node publishes\n";
    public static final String _TYPE = "rosgraph_msgs/Log";

    String getFile();

    String getFunction();

    Header getHeader();

    byte getLevel();

    int getLine();

    String getMsg();

    String getName();

    List<String> getTopics();

    void setFile(String str);

    void setFunction(String str);

    void setHeader(Header header);

    void setLevel(byte b);

    void setLine(int i);

    void setMsg(String str);

    void setName(String str);

    void setTopics(List<String> list);
}
