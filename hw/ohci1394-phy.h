/* Cable environment */

/* 0000_2 */
BEGIN_REGISTER(PHY_0000, 0, ~0)
    FIELD(PHY, Physical_ID, 0x3f, 0)
    FLAG (PHY, R,           6)
    FLAG (PHY, PS,          7)
END_REGISTER

/* 0001_2 */
BEGIN_REGISTER(PHY_0001, 1, ~0)
    FLAG (PHY, RHB,         0)
    FLAG (PHY, IBR,         1)
    FIELD(PHY, Gap_count,   0xfc, 2)
END_REGISTER

/* 0010_2 */
BEGIN_REGISTER(PHY_0010, 2, ~0)
    FIELD(PHY, Extended,    0x07, 0)
    FIELD(PHY, Total_ports, 0x08, 3)
END_REGISTER

/* 0011_2 */
BEGIN_REGISTER(PHY_0011, 3, ~0)
    FIELD(PHY, Max_speed,   0x07, 0)
    FIELD(PHY, Delay,       0xf0, 4)
END_REGISTER

/* 0100_2 */
BEGIN_REGISTER(PHY_0100, 4, ~0)
    FLAG (PHY, LCtrl,       0)
    FLAG (PHY, Contender,   1)
    FIELD(PHY, Jitter,      0x1c, 2)
    FIELD(PHY, Pwr_class,   0xe0, 5)
END_REGISTER

/* 0101_2 */
BEGIN_REGISTER(PHY_0101, 5, ~0)
    FLAG (PHY, Watchdog, 0)
    FLAG (PHY, ISBR,       1)
    FLAG (PHY, Loop,       2)
    FLAG (PHY, Pwr_fail,   3)
    FLAG (PHY, Timeout,    4)
    FLAG (PHY, Port_event, 5)
    FLAG (PHY, Enab_accel, 6)
    FLAG (PHY, Enab_multi, 7)
END_REGISTER

/* 0111_2 */
BEGIN_REGISTER(PHY_0111, 7, ~0)
    FIELD(PHY, Page_select, 0x07, 0)
    FIELD(PHY, Port_select, 0xf0, 4)
END_REGISTER

