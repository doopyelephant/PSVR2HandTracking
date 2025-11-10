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
    public double[] Coeffs = new double[20];
}
// csharp
public class CameraIntrinsics
{
    public float Fx;
    public float Fy;
    public float Cx;
    public float Cy;
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
                    Console.WriteLine(ReadInt16(IntPtr.Add(cfgPtr, 4)));
                    Console.WriteLine(ReadInt16(IntPtr.Add(cfgPtr, 6)));
                    int coeffsOffset = 44;
                    for (int j = 0; j < CoeffCount; j++)
                    {
                        IntPtr valPtr = IntPtr.Add(cfgPtr, coeffsOffset + j * 8);
                        _params.Coeffs[j] = ReadDouble(valPtr);
                    }
                    
                    int pxMatOffset = 8;

                    _intrinsics.Fx = ReadFloat(IntPtr.Add(cfgPtr, pxMatOffset + 0 * 4));
                    _intrinsics.Cx = ReadFloat(IntPtr.Add(cfgPtr, pxMatOffset + 2 * 4));
                    _intrinsics.Fy = ReadFloat(IntPtr.Add(cfgPtr, pxMatOffset + 4 * 4));
                    _intrinsics.Cy = ReadFloat(IntPtr.Add(cfgPtr, pxMatOffset + 5 * 4));
                    Console.WriteLine($"FXY: {(_intrinsics.Fx, _intrinsics.Fy)} CXY: {(_intrinsics.Cx, _intrinsics.Cy)}");
                    // Given your GetDistortedPoint implementation and CameraIntrinsics/CameraParameters
                    double xCenterNorm = 0.0, yCenterNorm = 0.0;
                    var centerDist = DistortionMeshes.GetDistortedPoint(xCenterNorm, yCenterNorm, _params);
                    double srcCx = centerDist.X * _intrinsics.Fx + _intrinsics.Cx;
                    double srcCy = centerDist.Y * _intrinsics.Fy + _intrinsics.Cy;
                    Console.WriteLine($"centerDist normalized: ({centerDist.X}, {centerDist.Y}), srcPixel: ({srcCx}, {srcCy})");
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

    private static double ReadDouble(IntPtr ptr)
    {
        // Safe generic read of an 8-byte double from unmanaged memory
        byte[] tmp = new byte[8];
        Marshal.Copy(ptr, tmp, 0, 8);
        return BitConverter.ToDouble(tmp, 0);
    }
    private static double ReadInt16(IntPtr ptr)
    {
        // Safe generic read of a 2-byte int from unmanaged memory
        byte[] tmp = new byte[2];
        Marshal.Copy(ptr, tmp, 0, 2);
        return BitConverter.ToInt16(tmp, 0);
    }
}
