// csharp
using System;
using System.IO;
using System.Runtime.InteropServices;
using System.Threading;
namespace PSVR2HandTracking;
public class SharedMemoryData
{
    // Pointer to the base of the shared memory buffer
    public IntPtr pBuf;
}
// csharp
public class CameraParameters
{
    public float[] Coeffs = new float[20];
}
// csharp
public class CameraIntrinsics
{
    public float fx;
    public float fy;
    public float cx;
    public float cy;
}

public static class CalibrationReader
{
    private const string CalibMutexName = "SHARE_VRT2_WIN_CALIB_MTX";
    private const int CameraConfigBaseOffset = 0x524;
    private const int CameraConfigStride = 0x70; // same stride used in the C++ code
    private const int CoeffCount = 20;

    public static bool GetDistortionConfig(SharedMemoryData data, int cameraId, ref CameraParameters _params, ref CameraIntrinsics _intrinsics)
    {
        Mutex mutex;
        try
        {
            mutex = Mutex.OpenExisting(CalibMutexName);
        }
        catch (WaitHandleCannotBeOpenedException)
        {
            Console.Error.WriteLine("Failed to open calibration mutex.");
            return false;
        }

        bool acquired = false;
        try
        {
            acquired = mutex.WaitOne();
            if (!acquired)
            {
                Console.Error.WriteLine("Failed to acquire calibration mutex.");
                return false;
            }

            IntPtr basePtr = IntPtr.Add(data.pBuf, CameraConfigBaseOffset);
            bool found = false;

            for (int i = 0; i < 4; i++)
            {
                IntPtr cfgPtr = IntPtr.Add(basePtr, i * CameraConfigStride);

                int camIdAtEntry = Marshal.ReadInt32(cfgPtr);
                if (camIdAtEntry == cameraId)
                {
                    // Read 20 float coefficients starting at offset 4
                    int coeffsOffset = 4;
                    for (int j = 0; j < CoeffCount; j++)
                    {
                        IntPtr valPtr = IntPtr.Add(cfgPtr, coeffsOffset + j * 4);
                        _params.Coeffs[j] = ReadFloat(valPtr);
                    }

                    // pxMat assumed right after coefficients (offset = 4 + 20*4 = 84)
                    int pxMatOffset = coeffsOffset + CoeffCount * 4;

                    _intrinsics.fx = ReadFloat(IntPtr.Add(cfgPtr, pxMatOffset + 0 * 4));
                    _intrinsics.cx = ReadFloat(IntPtr.Add(cfgPtr, pxMatOffset + 2 * 4));
                    _intrinsics.fy = ReadFloat(IntPtr.Add(cfgPtr, pxMatOffset + 4 * 4));
                    _intrinsics.cy = ReadFloat(IntPtr.Add(cfgPtr, pxMatOffset + 5 * 4));

                    found = true;
                    break;
                }
            }

            if (!found)
            {
                Console.Error.WriteLine($"Camera ID {cameraId} not found in shared memory.");
                return false;
            }

            return true;
        }
        finally
        {
            if (acquired)
            {
                try { mutex.ReleaseMutex(); } catch { /* ignore release errors */ }
            }
            mutex.Dispose();
        }
    }

    private static float ReadFloat(IntPtr ptr)
    {
        // Safe generic read of a 4-byte float from unmanaged memory
        byte[] tmp = new byte[4];
        Marshal.Copy(ptr, tmp, 0, 4);
        return BitConverter.ToSingle(tmp, 0);
    }
}
