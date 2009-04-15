/* Cable environment */

/* 0000_2 */
BEGIN_REGISTER(PHY_0000, 0, 0xff)
    FIELD(PHY, Physical_ID, 0xfc, 2)
    FLAG (PHY, R,           1)
    FLAG (PHY, PS,          0)
END_REGISTER

/* 0001_2 */
BEGIN_REGISTER(PHY_0001, 1, 0xff)
    FLAG (PHY, RHB,         7)
    FLAG (PHY, IBR,         6)
    FIELD(PHY, Gap_count,   0x3f, 0)
END_REGISTER

/* 0010_2 */
BEGIN_REGISTER(PHY_0010, 2, 0xff)
    FIELD(PHY, Extended,    0xe0, 5)
    FIELD(PHY, Total_ports, 0x1f, 0)
END_REGISTER

/* 0011_2 */
BEGIN_REGISTER(PHY_0011, 3, 0xef)
    FIELD(PHY, Max_speed,   0xe0, 5)
    FIELD(PHY, Delay,       0x0f, 0)
END_REGISTER

/* 0100_2 */
BEGIN_REGISTER(PHY_0100, 4, 0xff)
    FLAG (PHY, LCtrl,       7)
    FLAG (PHY, Contender,   6)
    FIELD(PHY, Jitter,      0x31, 3)
    FIELD(PHY, Pwr_class,   0x07, 0)
END_REGISTER

/* 0101_2 */
BEGIN_REGISTER(PHY_0101, 5, 0xff)
    FLAG (PHY, Watchdog,   7)
    FLAG (PHY, ISBR,       6)
    FLAG (PHY, Loop,       5)
    FLAG (PHY, Pwr_fail,   4)
    FLAG (PHY, Timeout,    3)
    FLAG (PHY, Port_event, 2)
    FLAG (PHY, Enab_accel, 1)
    FLAG (PHY, Enab_multi, 0)
END_REGISTER

/* 0111_2 */
BEGIN_REGISTER(PHY_0111, 7, 0xef)
    FIELD(PHY, Page_select, 0xe0, 5)
    FIELD(PHY, Port_select, 0x0f, 0)
END_REGISTER

