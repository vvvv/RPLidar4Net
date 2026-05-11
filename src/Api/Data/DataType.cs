namespace RPLidar4Net.Api.Data
{
    public enum DataType : byte
    {
        // SCAN (p. 14)
        Scan = 0x81,
        LegacyExpressScan = 0x82, //mentioned in LR001 2021-02-25 rev.2.2 (omitted in later versions of the pdf)
        ExpressScan = 0x84, 
        DenseCapsule = 0x85,
        // GET_INFO (p. 33)
        GetInfo = 0x04,
        // GET_HEALTH (p.35)
        GetHealth = 0x06,
        // GET_LIDAR_CONF (p.38)
        LidarConfig = 0x20,
        //s_lidar_cmd.h:162
        AccBoardFlag = 0xFF
    }
}