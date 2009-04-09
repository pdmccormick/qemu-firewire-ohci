LABEL(Version, 0x000)
LABEL(GUID_ROM, 0x004)
LABEL(ATRetries, 0x008)

LABEL(CSRReadData, 0x00c)
LABEL(CSRCompareDate, 0x010)
LABEL(CSRControl, 0x014)

LABEL(ConfigROMhdr, 0x018)
LABEL(BusID, 0x01c)
LABEL(BusOptions, 0x020)
LABEL(GUIDHi, 0x024)
LABEL(GUIDLo, 0x028)

LABEL(ConfigROMmap, 0x034)
LABEL(PostedWriteAddrLo, 0x038)
LABEL(PostedWriteAddrHi, 0x03c)
LABEL(VendorID, 0x040)

LABEL(HCControl_Set, 0x050)
LABEL(HCControl_Clear, 0x054)

LABEL(SelfIDBuffer, 0x064)
LABEL(SelfIDCount, 0x068)

LABEL(IRMultiChanMaskHi_Set, 0x070)
LABEL(IRMultiChanMaskHi_Clear, 0x074)

LABEL(IRMultiChanMaskLo_Set, 0x078)
LABEL(IRMultiChanMaskLo_Clear, 0x07c)

LABEL(IntEvent_Set, 0x080)
LABEL(IntEvent_Clear, 0x084)

LABEL(IntMask_Set, 0x088)
LABEL(IntMask_Clear, 0x08c)

LABEL(IsoXmitIntEvent_Set, 0x090)
LABEL(IsoXmitIntEvent_Clear, 0x094)
LABEL(IsoXmitIntMask_Set, 0x098)
LABEL(IsoXmitIntMask_Clear, 0x09c)

LABEL(IsoRecvIntEvent_Set, 0x0a0)
LABEL(IsoRecvIntEvent_Clear, 0x0a4)
LABEL(IsoRecvIntMask_Set, 0x0a8)
LABEL(IsoRecvIntMask_Clear, 0x0ac)

LABEL(InitialBandwidthAvailable, 0x0b0)
LABEL(InitialChannelsAvailableHi, 0x0b4)
LABEL(InitialChannelsAvailableLo, 0x0b8)

LABEL(FairnessControl, 0x0dc)
LABEL(LinkControl_Set, 0x0e0)
LABEL(LinkControl_Clear, 0x0e4)
LABEL(NodeID, 0x0e8)
LABEL(PhyControl, 0x0ec)
LABEL(IsoCycleTimer, 0x0f0)

LABEL(AsyncReqFilterHi_Set, 0x100)
LABEL(AsyncReqFilterHi_Clear, 0x104)
LABEL(AsyncReqFilterLo_Set, 0x108)
LABEL(AsyncReqFilterLo_Clear, 0x10c)

LABEL(PhyReqFilterHi_Set, 0x110)
LABEL(PhyReqFilterHi_Clear, 0x114)
LABEL(PhyReqFilterLo_Set, 0x118)
LABEL(PhyReqFilterLo_Clear, 0x11c)

LABEL(PhysicalUpperBound, 0x120)

/* Async Request Transmit */
LABEL(AsyncReqXmitContextControl_Set, 0x180)
LABEL(AsyncReqXmitContextControl_Clear, 0x184)
LABEL(AsyncReqXmitCommandPtr, 0x18c)

/* Async Response Transmit */
LABEL(AsyncRespXmitContextControl_Set, 0x1a0)
LABEL(AsyncRespXmitContextControl_Clear, 0x1a4)
LABEL(AsyncRespXmitCommandPtr, 0x1ac)

/* Async Request Receive */
LABEL(AsyncReqRecvContextControl_Set, 0x1c0)
LABEL(AsyncReqRecvContextControl_Clear, 0x1c4)
LABEL(AsyncReqRecvCommandPtr, 0x1cc)

/* Async Response Receive */
LABEL(AsyncRespRecvContextControl_Set, 0x1e0)
LABEL(AsyncRespRecvContextControl_Clear, 0x1e4)
LABEL(AsyncRepsRecvCommandPtr, 0x1ec)

/* Iso Transmit */
LABEL(IsoXmitContextControl_Set, 0x200)     // + 16*n
LABEL(IsoXmitContextControl_Clear, 0x204)   // + 16*n
LABEL(IsoXmitCommandPtr, 0x20C)             // + 16*n

/* Iso Receive */
LABEL(IsoRecvContextControl_Set, 0x400)     // + 32*n
LABEL(IsoRecvContextControl_Clear, 0x404)   // + 32*n
LABEL(IsoRecvCommandPtr, 0x40C)             // + 32*n
LABEL(IsoRecvContextMatch, 0x410)           // + 32*n

