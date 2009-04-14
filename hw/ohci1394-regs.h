/* 5.2  Version */
BEGIN_REGISTER(Version, 0x000, 0x01ff00ff)
    FLAG (Version, GUID_ROM,    24)
    FIELD(Version, version,     0x00ff0000, 16)
    FIELD(Version, revision,    0x000000ff, 0)
END_REGISTER

/* 5.3  GUID_ROM */
BEGIN_REGISTER(GUID_ROM, 0x004, 0x82ff00ff)
    FLAG (GUID_ROM, addrReset,  31)
    FLAG (GUID_ROM, rdStart,    25)
    FIELD(GUID_ROM, rdData,     0x00ff0000, 16)
    FIELD(GUID_ROM, miniROM,    0x000000ff, 0)
END_REGISTER

/* 5.4  ATRetries */
BEGIN_REGISTER(ATRetries, 0x008, 0xffff0fff)
    FIELD(ATRetries, secondLimit,   0xe0000000, 29)
    FIELD(ATRetries, cycleLimit,    0x1fff0000, 16)
    FIELD(ATRetries, maxPhysRespRetries,    0x00000f00, 8)
    FIELD(ATRetries, maxATRespRetries, 0x000000f0, 4)
    FIELD(ATRetries, maxATReqRetries, 0x0000000f, 0)
END_REGISTER

/* 5.5  p56 */

BEGIN_REGISTER(CSRReadData, 0x00c, 0xffffffff)
END_REGISTER

BEGIN_REGISTER(CSRCompareDate, 0x010, 0xffffffff)
END_REGISTER

BEGIN_REGISTER(CSRControl, 0x014, 0xffffffff)
END_REGISTER

BEGIN_REGISTER(ConfigROMhdr, 0x018, 0xffffffff)
END_REGISTER

BEGIN_REGISTER(BusID, 0x01c, 0xffffffff)
END_REGISTER

BEGIN_REGISTER(BusOptions, 0x020, 0xffffffff)
END_REGISTER

BEGIN_REGISTER(GUIDHi, 0x024, 0xffffffff)
END_REGISTER

BEGIN_REGISTER(GUIDLo, 0x028, 0xffffffff)
END_REGISTER

BEGIN_REGISTER(ConfigROMmap, 0x034, 0xffffffff)
END_REGISTER

BEGIN_REGISTER(PostedWriteAddrLo, 0x038, 0xffffffff)
END_REGISTER

BEGIN_REGISTER(PostedWriteAddrHi, 0x03c, 0xffffffff)
END_REGISTER


/* 5.6  VendorID */
BEGIN_REGISTER(VendorID, 0x040, 0xffffffff)
    FIELD(VendorID, vendorUnique, 0xff000000, 24)
    FIELD(VendorID, vendorCompanyID, 0x00ffffff, 0)
END_REGISTER

/* 5.7  HCControl */
BEGIN_REGISTER_SET_CLEAR(HCControl, 0x050, 0xe0cf0000)
    FLAG(HCControl, BIBimageValid,      31)
    FLAG(HCControl, noByteSwapData,     30)
    FLAG(HCControl, ackTardyEnable,     29)
    FLAG(HCControl, programPhyEnable,   23)
    FLAG(HCControl, aPhyEnhanceEnable,  22)
    FLAG(HCControl, LPS,                19)
    FLAG(HCControl, postedWriteEnable,  18)
    FLAG(HCControl, linkEnable,         17)
    FLAG(HCControl, softReset,          16)
END_REGISTER_SET_CLEAR


/* 11.1  SelfIDBuffer */
BEGIN_REGISTER(SelfIDBuffer, 0x064, 0xfffff800)
    FIELD(SelfIDBuffer, selfIDBufferPtr, 0xfffff800, 11)
END_REGISTER

/* 11.2  SelfIDCount */
BEGIN_REGISTER(SelfIDCount, 0x068, 0x80ff07fc)
    FIELD(SelfIDCount, selfIDError,         0x80000000, 31)
    FIELD(SelfIDCount, selfIDGeneration,    0x00ff0000, 16)
    FIELD(SelfIDCount, selfIDSize,          0x000007fc, 2)
END_REGISTER


/* ?.??  IRMultiChanMaskHi/Lo */
BEGIN_REGISTER_SET_CLEAR(IRMultiChanMaskHi, 0x070, 0xffffffff)
END_REGISTER_SET_CLEAR

BEGIN_REGISTER_SET_CLEAR(IRMultiChanMaskLo, 0x078, 0xffffffff)
END_REGISTER_SET_CLEAR


/* 6.1  IntEvent */
BEGIN_REGISTER_SET_CLEAR(IntEvent, 0x080, 0x6fff83ff)
    FLAG(IntEvent, vendorSpecific,      30)
    FLAG(IntEvent, softInterrupt,       29)
    FLAG(IntEvent, ack_tardy,           27)
    FLAG(IntEvent, phyRegRcvd,          26)
    FLAG(IntEvent, cycleTooLong,        25)
    FLAG(IntEvent, unrecoverableError,  24)
    FLAG(IntEvent, cycleInconsistent,   23)
    FLAG(IntEvent, cycleLost,           22)
    FLAG(IntEvent, cycle64Seconds,      21)
    FLAG(IntEvent, cycleSynch,          20)
    FLAG(IntEvent, phy,                 19)
    FLAG(IntEvent, regAccessFail,       18)
    FLAG(IntEvent, busReset,            17)
    FLAG(IntEvent, selfIDComplete,      16)
    FLAG(IntEvent, selfIDComplete2,     15)
    FLAG(IntEvent, lockRespErr,         9)
    FLAG(IntEvent, postedWriteErr,      8)
    FLAG(IntEvent, isochRx,             7)
    FLAG(IntEvent, isochTx,             6)
    FLAG(IntEvent, RSPkt,               5)
    FLAG(IntEvent, RQPkt,               4)
    FLAG(IntEvent, ARRS,                3)
    FLAG(IntEvent, ARRQ,                2)
    FLAG(IntEvent, respTxComplete,      1)
    FLAG(IntEvent, reqTxComplete,       0)
END_REGISTER_SET_CLEAR

/* 6.2  IntMask */
BEGIN_REGISTER_SET_CLEAR(IntMask, 0x088, 0xefff83ff)
    FLAG(IntMask, masterIntEnable,      31)
END_REGISTER_SET_CLEAR

/* 6.3.1  IsoXmitIntEvent */
BEGIN_REGISTER_SET_CLEAR(IsoXmitIntEvent, 0x090, 0xffffffff)
END_REGISTER_SET_CLEAR

/* 6.3.2  IsoXmitIntMask */
BEGIN_REGISTER_SET_CLEAR(IsoXmitIntMask, 0x098, 0xffffffff)
END_REGISTER_SET_CLEAR

/* 6.4.1  IsoRecvIntEvent */
BEGIN_REGISTER_SET_CLEAR(IsoRecvIntEvent, 0x0a0, 0xffffffff)
END_REGISTER_SET_CLEAR

/* 6.4.2  IsoRecvIntMask */
BEGIN_REGISTER_SET_CLEAR(IsoRecvIntMask, 0x0a8, 0xffffffff)
END_REGISTER_SET_CLEAR


BEGIN_REGISTER(InitialBandwidthAvailable, 0x0b0, 0xffffffff)
END_REGISTER

BEGIN_REGISTER(InitialChannelsAvailableHi, 0x0b4, 0xffffffff)
END_REGISTER

BEGIN_REGISTER(InitialChannelsAvailableLo, 0x0b8, 0xffffffff)
END_REGISTER

/* 5.9  FairnessControl */
BEGIN_REGISTER(FairnessControl, 0x0dc, 0x000000ff)
    FIELD(FairnessControl, pri_req, 0x000000ff, 0)
END_REGISTER

/* 5.10  LinkControl */
BEGIN_REGISTER_SET_CLEAR(LinkControl, 0x0e0, 0x00700640)
    FLAG(LinkControl, cycleSource,          22)
    FLAG(LinkControl, cycleMaster,          21)
    FLAG(LinkControl, cycleTimerEnable,     20)
    FLAG(LinkControl, rcvPhyPkt,            10)
    FLAG(LinkControl, rcvSelfID,            9)
    FLAG(LinkControl, tag1SyncFilterLock,   6)
END_REGISTER_SET_CLEAR

/* 5.11  Node ID */
BEGIN_REGISTER(NodeID, 0x0e8, 0xc800ffff)
    FLAG (NodeID, iDValid,      31)
    FLAG (NodeID, root,         30)
    FLAG (NodeID, CPS,          27)
    FIELD(NodeID, busNumber,    0x0000ffc0, 6)
    FIELD(NodeID, nodeNumber,   0x0000003f, 0)
END_REGISTER

/* 5.12  PhyControl */
BEGIN_REGISTER(PhyControl, 0x0ec, 0xffffffff)
    FLAG (PhyControl, rdDone,   31)
    FIELD(PhyControl, rdAddr,   0x0f000000, 24)
    FIELD(PhyControl, rdData,   0x00ff0000, 16)
    FLAG (PhyControl, rdReg,    15)
    FLAG (PhyControl, wrReg,    14)
    FIELD(PhyControl, regAddr,  0x00000f00, 8)
    FIELD(PhyControl, wrData,   0x000000ff, 0)
END_REGISTER

/* 5.13  IsoCycleTimer */
BEGIN_REGISTER(IsoCycleTimer, 0x0f0, 0xffffffff)
    FIELD(IsoCycleTimer, cycleSeconds,  0xfe000000, 25)
    FIELD(IsoCycleTimer, cycleCount,    0x01fff000, 12)
    FIELD(IsoCycleTimer, cycleOffest,   0x00000fff, 0)
END_REGISTER

/* 5.14.1  AsyncReqFilter */
BEGIN_REGISTER_SET_CLEAR(AsyncReqFilterHi, 0x100, 0xffffffff)
END_REGISTER_SET_CLEAR

BEGIN_REGISTER_SET_CLEAR(AsyncReqFilterLo, 0x108, 0xffffffff)
END_REGISTER_SET_CLEAR

/* 5.14.2  PhyReqFilter */
BEGIN_REGISTER_SET_CLEAR(PhyReqFilterHi, 0x110, 0xffffffff)
END_REGISTER_SET_CLEAR

BEGIN_REGISTER_SET_CLEAR(PhyReqFilterLo, 0x118, 0xffffffff)
END_REGISTER_SET_CLEAR


BEGIN_REGISTER(PhysicalUpperBound, 0x120, 0xffffffff)
END_REGISTER


/* Async Request Transmit */
BEGIN_REGISTER_SET_CLEAR(AsyncReqXmitContextControl, 0x180, 0xffffffff)
END_REGISTER_SET_CLEAR

BEGIN_REGISTER(AsyncReqXmitCommandPtr, 0x18c, 0xffffffff)
END_REGISTER

/* Async Response Transmit */
BEGIN_REGISTER_SET_CLEAR(AsyncRespXmitContextControl, 0x1a0, 0xffffffff)
END_REGISTER_SET_CLEAR

BEGIN_REGISTER(AsyncRespXmitCommandPtr, 0x1ac, 0xffffffff)
END_REGISTER

/* Async Request Receive */
BEGIN_REGISTER_SET_CLEAR(AsyncReqRecvContextControl, 0x1c0, 0xffffffff)
END_REGISTER_SET_CLEAR

BEGIN_REGISTER(AsyncReqRecvCommandPtr, 0x1cc, 0xffffffff)
END_REGISTER

/* Async Response Receive */
BEGIN_REGISTER_SET_CLEAR(AsyncRespRecvContextControl, 0x1e0, 0xffffffff)
END_REGISTER_SET_CLEAR

BEGIN_REGISTER(AsyncResqRecvCommandPtr, 0x1ec, 0xffffffff)
END_REGISTER

/* Iso Transmit */
BEGIN_REGISTER_SET_CLEAR(IsoXmitContextControl, 0x200, 0xffffffff)
END_REGISTER_SET_CLEAR

BEGIN_REGISTER(IsoXmitCommandPtr, 0x20c, 0xffffffff)
END_REGISTER

/* Iso Receive */
BEGIN_REGISTER_SET_CLEAR(IsoRecvContextControl, 0x400, 0xffffffff)
END_REGISTER_SET_CLEAR

BEGIN_REGISTER(IsoRecvCommandPtr, 0x40c, 0xffffffff)
END_REGISTER

BEGIN_REGISTER(IsoRecvContextMatch, 0x410, 0xffffffff)
END_REGISTER

