package sensor_msgs;

import org.ros.internal.message.Message;
import std_msgs.Header;

public interface BatteryState extends Message {
    public static final byte POWER_SUPPLY_HEALTH_COLD = 6;
    public static final byte POWER_SUPPLY_HEALTH_DEAD = 3;
    public static final byte POWER_SUPPLY_HEALTH_GOOD = 1;
    public static final byte POWER_SUPPLY_HEALTH_OVERHEAT = 2;
    public static final byte POWER_SUPPLY_HEALTH_OVERVOLTAGE = 4;
    public static final byte POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE = 8;
    public static final byte POWER_SUPPLY_HEALTH_UNKNOWN = 0;
    public static final byte POWER_SUPPLY_HEALTH_UNSPEC_FAILURE = 5;
    public static final byte POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE = 7;
    public static final byte POWER_SUPPLY_STATUS_CHARGING = 1;
    public static final byte POWER_SUPPLY_STATUS_DISCHARGING = 2;
    public static final byte POWER_SUPPLY_STATUS_FULL = 4;
    public static final byte POWER_SUPPLY_STATUS_NOT_CHARGING = 3;
    public static final byte POWER_SUPPLY_STATUS_UNKNOWN = 0;
    public static final byte POWER_SUPPLY_TECHNOLOGY_LIFE = 4;
    public static final byte POWER_SUPPLY_TECHNOLOGY_LIMN = 6;
    public static final byte POWER_SUPPLY_TECHNOLOGY_LION = 2;
    public static final byte POWER_SUPPLY_TECHNOLOGY_LIPO = 3;
    public static final byte POWER_SUPPLY_TECHNOLOGY_NICD = 5;
    public static final byte POWER_SUPPLY_TECHNOLOGY_NIMH = 1;
    public static final byte POWER_SUPPLY_TECHNOLOGY_UNKNOWN = 0;
    public static final String _DEFINITION = "\n# Constants are chosen to match the enums in the linux kernel\n# defined in include/linux/power_supply.h as of version 3.7\n# The one difference is for style reasons the constants are\n# all uppercase not mixed case.\n\n# Power supply status constants\nuint8 POWER_SUPPLY_STATUS_UNKNOWN = 0\nuint8 POWER_SUPPLY_STATUS_CHARGING = 1\nuint8 POWER_SUPPLY_STATUS_DISCHARGING = 2\nuint8 POWER_SUPPLY_STATUS_NOT_CHARGING = 3\nuint8 POWER_SUPPLY_STATUS_FULL = 4\n\n# Power supply health constants\nuint8 POWER_SUPPLY_HEALTH_UNKNOWN = 0\nuint8 POWER_SUPPLY_HEALTH_GOOD = 1\nuint8 POWER_SUPPLY_HEALTH_OVERHEAT = 2\nuint8 POWER_SUPPLY_HEALTH_DEAD = 3\nuint8 POWER_SUPPLY_HEALTH_OVERVOLTAGE = 4\nuint8 POWER_SUPPLY_HEALTH_UNSPEC_FAILURE = 5\nuint8 POWER_SUPPLY_HEALTH_COLD = 6\nuint8 POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE = 7\nuint8 POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE = 8\n\n# Power supply technology (chemistry) constants\nuint8 POWER_SUPPLY_TECHNOLOGY_UNKNOWN = 0\nuint8 POWER_SUPPLY_TECHNOLOGY_NIMH = 1\nuint8 POWER_SUPPLY_TECHNOLOGY_LION = 2\nuint8 POWER_SUPPLY_TECHNOLOGY_LIPO = 3\nuint8 POWER_SUPPLY_TECHNOLOGY_LIFE = 4\nuint8 POWER_SUPPLY_TECHNOLOGY_NICD = 5\nuint8 POWER_SUPPLY_TECHNOLOGY_LIMN = 6\n\nHeader  header\nfloat32 voltage          # Voltage in Volts (Mandatory)\nfloat32 current          # Negative when discharging (A)  (If unmeasured NaN)\nfloat32 charge           # Current charge in Ah  (If unmeasured NaN)\nfloat32 capacity         # Capacity in Ah (last full capacity)  (If unmeasured NaN)\nfloat32 design_capacity  # Capacity in Ah (design capacity)  (If unmeasured NaN)\nfloat32 percentage       # Charge percentage on 0 to 1 range  (If unmeasured NaN)\nuint8   power_supply_status     # The charging status as reported. Values defined above\nuint8   power_supply_health     # The battery health metric. Values defined above\nuint8   power_supply_technology # The battery chemistry. Values defined above\nbool    present          # True if the battery is present\n\nfloat32[] cell_voltage   # An array of individual cell voltages for each cell in the pack\n                         # If individual voltages unknown but number of cells known set each to NaN\nstring location          # The location into which the battery is inserted. (slot number or plug)\nstring serial_number     # The best approximation of the battery serial number\n";
    public static final String _TYPE = "sensor_msgs/BatteryState";

    float getCapacity();

    float[] getCellVoltage();

    float getCharge();

    float getCurrent();

    float getDesignCapacity();

    Header getHeader();

    String getLocation();

    float getPercentage();

    byte getPowerSupplyHealth();

    byte getPowerSupplyStatus();

    byte getPowerSupplyTechnology();

    boolean getPresent();

    String getSerialNumber();

    float getVoltage();

    void setCapacity(float f);

    void setCellVoltage(float[] fArr);

    void setCharge(float f);

    void setCurrent(float f);

    void setDesignCapacity(float f);

    void setHeader(Header header);

    void setLocation(String str);

    void setPercentage(float f);

    void setPowerSupplyHealth(byte b);

    void setPowerSupplyStatus(byte b);

    void setPowerSupplyTechnology(byte b);

    void setPresent(boolean z);

    void setSerialNumber(String str);

    void setVoltage(float f);
}
