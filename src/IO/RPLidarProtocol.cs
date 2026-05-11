using System;
using System.Collections.Generic;

namespace RPLidar4Net.IO
{
    /// <summary>
    /// Low-level RPLIDAR serial protocol constants, packet building, and parsing.
    /// Covers the A/S/C series protocol documented in Slamtec's LR001/LR002 spec sheets.
    /// </summary>
    internal static class Protocol
    {
        // Express scan — A2/A3/C/S series (FW ≥ 1.17). Returns 84-byte capsules.
        public const byte CMD_EXPRESS_SCAN     = 0x82;
        public const byte RESP_EXPRESS_SCAN    = 0x82;   // Standard Express Capsule (A2/A3)
        public const byte RESP_DENSE_CAPSULE   = 0x85;   // Dense Capsule (C1/S-series)
        public const int  EXPRESS_CAPSULE_SIZE = 84;     // bytes per capsule (both formats)
        public const byte EXP_SYNC_1          = 0xA;    // upper nibble of capsule byte 0
        public const byte EXP_SYNC_2          = 0x5;    // upper nibble of capsule byte 1

        // ── Response framing ─────────────────────────────────────────────────────
        public const byte RESP_SYNC1 = 0xA5;
        public const byte RESP_SYNC2 = 0x5A;
        public const int  DESC_SIZE  = 7;   // response descriptor length in bytes

        // ── Standard scan measurement ─────────────────────────────────────────────

        public readonly struct Measurement
        {
            /// <summary>Bearing in degrees, 0–360, 0 = forward.</summary>
            public readonly float Angle;
            /// <summary>Radial distance in millimetres. 0 = no object detected.</summary>
            public readonly float Distance;
            /// <summary>Measurement quality 0–63 (higher = better).</summary>
            public readonly byte Quality;
            /// <summary>True on the first point of each new 360° scan.</summary>
            public readonly bool IsNewScan;

            internal Measurement(float a, float d, byte q, bool s)
            { Angle = a; Distance = d; Quality = q; IsNewScan = s; }
        }

        // ── Express scan capsule ─────────────────────────────────────────────────
        //
        // Capsule layout (84 bytes, sl_lidar_response_capsule_measurement_nodes_t):
        //   byte[0]    s_checksum_1  — upper nibble = EXP_SYNC_1 (0xA)
        //   byte[1]    s_checksum_2  — upper nibble = EXP_SYNC_2 (0x5)
        //   byte[2..3] start_angle_sync_q6 (uint16 LE):
        //                bit15  = new-scan flag
        //                [14:0] = startAngle × 64  (÷64 → degrees)
        //   bytes[4..83] 16 × Cabin (5 bytes each):
        //     [0..1] distance_angle_1 (uint16 LE): bits[15:2]=distQ2, bits[1:0]=angleLo
        //     [2..3] distance_angle_2 (uint16 LE): same for second measurement
        //     [4]    offset_angles_q3: upper nibble = angleHi for meas-A, lower = angleHi for meas-B

        internal readonly struct LegacyExpressCabin
        {
            public readonly ushort DistAngle1;    // raw uint16 as described above
            public readonly ushort DistAngle2;
            public readonly byte   OffsetAnglesQ3;

            internal LegacyExpressCabin(ushort da1, ushort da2, byte oa)
            { DistAngle1 = da1; DistAngle2 = da2; OffsetAnglesQ3 = oa; }
        }

        internal readonly struct LegacyExpressCapsule
        {
            public readonly bool          IsNewScan;
            public readonly float         StartAngleDeg;   // 0–360
            public readonly LegacyExpressCabin[] Cabins;          // length 16
            public readonly bool          Valid;

            internal LegacyExpressCapsule(bool isNew, float angle, LegacyExpressCabin[] cabins)
            { IsNewScan = isNew; StartAngleDeg = angle; Cabins = cabins; Valid = true; }
        }

        /// <summary>
        /// Parse an 84-byte legacy express-scan capsule buffer.
        /// Returns an invalid capsule if sync bytes are missing.
        /// </summary>
        public static LegacyExpressCapsule ParseExpressCapsule(ReadOnlySpan<byte> buf)
        {
            if ((buf[0] >> 4) != EXP_SYNC_1 || (buf[1] >> 4) != EXP_SYNC_2)
                return default;

            ushort startWord  = (ushort)(buf[2] | (buf[3] << 8));
            bool   isNewScan  = (startWord >> 15) != 0;
            float  startAngle = (startWord & 0x7FFF) / 64.0f;

            var cabins = new LegacyExpressCabin[16];
            int offset = 4;
            for (int i = 0; i < 16; i++, offset += 5)
            {
                ushort da1 = (ushort)(buf[offset]     | (buf[offset + 1] << 8));
                ushort da2 = (ushort)(buf[offset + 2] | (buf[offset + 3] << 8));
                byte   oa  = buf[offset + 4];
                cabins[i]  = new LegacyExpressCabin(da1, da2, oa);
            }
            return new LegacyExpressCapsule(isNewScan, startAngle, cabins);
        }

        /// <summary>
        /// Decode the 32 measurements carried by <paramref name="prev"/> using the
        /// start-angle of <paramref name="curr"/> for linear angle interpolation.
        /// Appends decoded <see cref="Measurement"/> values to <paramref name="output"/>.
        /// Zero-distance measurements are skipped.
        /// </summary>
        public static void DecodeLegacyExpressCapsulePair(
            in LegacyExpressCapsule prev, in LegacyExpressCapsule curr,
            List<Measurement> output)
        {
            float a0 = prev.StartAngleDeg;
            float a1 = curr.StartAngleDeg;
            // Handle wrap-around at 360°
            float dA = a1 - a0;
            if (dA < 0) dA += 360f;

            int idx = 0;
            for (int c = 0; c < 16; c++)
            {
                var cabin = prev.Cabins[c];
                // Per Slamtec SDK handler_capsules.cpp:
                //   angle_offset1_q3 = (offset_angles_q3 & 0xF)  | ((distance_angle_1 & 0x3) << 4)
                //   angle_offset2_q3 = (offset_angles_q3 >> 4)   | ((distance_angle_2 & 0x3) << 4)
                // DA1 uses the LOWER nibble of offset_angles_q3; DA2 uses the UPPER nibble.
                ProcessHalf(cabin.DistAngle1, (byte)(cabin.OffsetAnglesQ3 & 0xF), idx,     a0, dA, output);
                ProcessHalf(cabin.DistAngle2, (byte)(cabin.OffsetAnglesQ3 >> 4),  idx + 1, a0, dA, output);
                idx += 2;
            }
        }

        private static void ProcessHalf(
            ushort distAngle, byte angleLo4, int idx,
            float a0, float dA, List<Measurement> output)
        {
            // Distance: clear the 2 angle bits then interpret as Q2 (mm × 4).
            // SDK: dist_q2 = distance_angle & 0xFFFC; hqNode.dist_mm_q2 = dist_q2
            // → dist_mm = (distAngle & 0xFFFC) / 4.0
            float dist = (distAngle & 0xFFFC) / 4.0f;
            if (dist < 0.01f) return;   // no-object reading

            // 6-bit Q3 angle delta:
            //   bits[3:0] = lower nibble of offset_angles_q3 (passed as angleLo4)
            //   bits[5:4] = lower 2 bits of distance_angle (the angle-offset bits)
            // SDK: angle_offset_q3 = angleLo4 | ((distAngle & 0x3) << 4)
            int   dAngleQ3  = angleLo4 | ((distAngle & 0x3) << 4);
            float dAngleDeg = dAngleQ3 / 8.0f;

            // Interpolate base angle then subtract the cabin delta
            float angle = a0 + dA * (idx / 32.0f) - dAngleDeg;
            angle = ((angle % 360f) + 360f) % 360f;   // normalise to [0, 360)

            output.Add(new Measurement(angle, dist, 15, false));
        }

        // ── Dense Capsule (C1 / S-series express scan, DataType 0x85) ────────────
        //
        // Capsule layout (84 bytes, sl_lidar_response_dense_capsule_measurement_nodes_t):
        //   byte[0]    s_checksum_1  — upper nibble = EXP_SYNC_1 (0xA)
        //   byte[1]    s_checksum_2  — upper nibble = EXP_SYNC_2 (0x5)
        //   byte[2..3] start_angle_sync_q6 (uint16 LE):
        //                bit15  = new-scan flag
        //                [14:0] = startAngle × 64  (÷64 → degrees)
        //   bytes[4..83] 40 × Cabin (2 bytes each):
        //     [0..1] distance (uint16 LE): raw distance in mm (no Q encoding)
        //
        // Angles are evenly distributed across the capsule's angular span —
        // no per-measurement angle offsets unlike the standard Express Capsule.

        internal readonly struct DenseCapsule
        {
            public readonly bool      IsNewScan;
            public readonly float     StartAngleDeg;  // 0–360
            public readonly ushort[]  Distances;       // length 40, raw mm
            public readonly bool      Valid;

            internal DenseCapsule(bool isNew, float angle, ushort[] distances)
            { IsNewScan = isNew; StartAngleDeg = angle; Distances = distances; Valid = true; }
        }

        /// <summary>
        /// Parse an 84-byte dense-capsule buffer (C1/S-series express scan response 0x85).
        /// Returns an invalid capsule if sync nibbles are missing.
        /// </summary>
        public static DenseCapsule ParseDenseCapsule(ReadOnlySpan<byte> buf)
        {
            if ((buf[0] >> 4) != EXP_SYNC_1 || (buf[1] >> 4) != EXP_SYNC_2)
                return default;

            ushort startWord  = (ushort)(buf[2] | (buf[3] << 8));
            bool   isNewScan  = (startWord >> 15) != 0;
            float  startAngle = (startWord & 0x7FFF) / 64.0f;

            var distances = new ushort[40];
            int offset = 4;
            for (int i = 0; i < 40; i++, offset += 2)
                distances[i] = (ushort)(buf[offset] | (buf[offset + 1] << 8));

            return new DenseCapsule(isNewScan, startAngle, distances);
        }

        /// <summary>
        /// Decode the 40 measurements carried by <paramref name="prev"/> using the
        /// start-angle of <paramref name="curr"/> for linear angle interpolation.
        /// Angles are evenly distributed — no per-measurement offsets.
        /// Zero-distance measurements are skipped.
        /// </summary>
        public static void DecodeDenseCapsulePair(
            in DenseCapsule prev, in DenseCapsule curr,
            List<Measurement> output)
        {
            float a0 = prev.StartAngleDeg;
            float a1 = curr.StartAngleDeg;
            float dA = a1 - a0;
            if (dA < 0) dA += 360f;

            const int cabins = 40;
            for (int i = 0; i < cabins; i++)
            {
                float dist = prev.Distances[i];   // raw mm directly (SDK: dist_q2 = dist << 2)
                if (dist < 0.01f) continue;        // no-object reading

                float angle = a0 + dA * (i / (float)cabins);
                angle = ((angle % 360f) + 360f) % 360f;
                output.Add(new Measurement(angle, dist, 15, false));
            }
        }
    }
}