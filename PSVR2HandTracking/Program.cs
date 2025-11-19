using System;
using System.IO.MemoryMappedFiles;
using System.Numerics;
using System.Runtime.InteropServices;
using BCnEncoder.Decoder;
using BCnEncoder.ImageSharp;
using BCnEncoder.Shared;
using SixLabors.ImageSharp;
using SixLabors.ImageSharp.PixelFormats;
using OpenCvSharp;
using Mediapipe.Net;
using Mediapipe.Net.Framework.Protobuf;
using PSVR2HandTracking;
using Size = OpenCvSharp.Size;

using Microsoft.ML.OnnxRuntime;
using Microsoft.ML.OnnxRuntime.Tensors;
using Point = OpenCvSharp.Point;
using Rect = OpenCvSharp.Rect;
using DirectN;

public static class Program
{
    private static IComObject<IMFVirtualCamera>? _camera;
    //public const string CLSID_VCamNet = "A8C3D28A-238A-421A-8919-6AA97A44B634";
    private static long[] _offsets = new long[]
    {
        0x10BA00,
        0x30BB00,
        0x50BC00,
        0x70BD00,
        0x90BE00,
        0xB0BF00,
        0xD0C000,
        0xF0C100
    };
    private static MemoryMappedViewAccessor accessor;

    private static MemoryMappedFile _shareFile;
    private static EventWaitHandle _imageEvent;
    private static MemoryMappedViewStream _imageViewStream;
    private static int _index = 0;
    private static Mat undistortionX = new Mat();
    private static Mat undistortionY = new Mat();

    public static void Main()
    {
        _shareFile = MemoryMappedFile.OpenExisting("SHARE_VRT2_WIN");
        _imageEvent = new EventWaitHandle(false, EventResetMode.AutoReset, "SHARE_VRT2_WIN_IMAGE_EVT");

        var data = new SharedMemoryData();
        data.pBuf = SharedMemoryNative.MapRead(_shareFile);
        var cameraParams = new CameraParameters();
        var intrinsics = new CameraIntrinsics();
        CalibrationReader.GetDistortionConfig(data, 0, ref cameraParams, ref intrinsics);
        foreach (var VARIABLE in cameraParams.Coeffs)
        {
            Console.WriteLine(VARIABLE);
        }
        DistortionMeshes.CreateUndistortionMaps(1024,1016,1f,intrinsics,cameraParams,out var mapX, out var mapY);
        undistortionX = mapX;
        undistortionY = mapY;
      /*  MFFunctions.MFStartup();
        var hr = Functions.MFCreateVirtualCamera(
            __MIDL___MIDL_itf_mfvirtualcamera_0000_0000_0001.MFVirtualCameraType_SoftwareCameraSource,
            __MIDL___MIDL_itf_mfvirtualcamera_0000_0000_0002.MFVirtualCameraLifetime_Session,
            __MIDL___MIDL_itf_mfvirtualcamera_0000_0000_0003.MFVirtualCameraAccess_CurrentUser,
            "PSVR2CAM",
            "{" + CLSID_VCamNet + "}",
            null,
            0,
            out var camera);
        if (hr.IsSuccess)
        {
            Console.WriteLine("Virtual camera created successfully.");
            _camera = new ComObject<IMFVirtualCamera>(camera);
            hr = _camera.Object.Start(null);
        }
        else
        {
            Console.WriteLine("Failed to create virtual camera.");
        }

        Console.WriteLine("Code: " + hr.Value);*/
        long capacity = 1920 * 960;
        using var frameReadySemaphore = new Semaphore(0, 1, "PSVR2_FRAME_READY");
        using var mmf = MemoryMappedFile.CreateOrOpen("PSVR2_SHARED_MEM", capacity);
        using var accessor = mmf.CreateViewAccessor();

        while (true)
        {
            byte[] frameBytes = Update(); // Your method

            // Write frame to shared memory
            accessor.WriteArray(0, frameBytes, 0, frameBytes.Length);

            // Signal the DLL (which is inside Teams/OBS)
            try { frameReadySemaphore.Release(1); } catch (SemaphoreFullException) { }

            Thread.Sleep(33); // ~30fps
        }


        Exit();
    }

    private static void Exit()
    {
        _shareFile.Dispose();
        MFFunctions.MFShutdown();
    }

    private static byte[] Update()
    {
        _imageEvent.WaitOne();
            _imageViewStream = _shareFile.CreateViewStream(_offsets[_index], 0xFE100);
            byte[] buffer = new byte[0xFE100];
            _imageViewStream.Read(buffer, 0, 0xFE100);
            var dataleft = new byte[0x7F000];
            Array.Copy(buffer, 0x100, dataleft, 0, 0x7F000);
            var dataright = new byte[0x7F000];
            Array.Copy(buffer, 0x100 + 0x7F000, dataright, 0, 0x7F000);
            _imageViewStream.Dispose();
// Create a decoder
            var decoder = new BcDecoder();

            // Decode the image
            // This example assumes you know the width and height of the original image
            Image<Rgba32> decodedImage = decoder.DecodeRawToImageRgba32(dataleft,1024,1016,CompressionFormat.Bc4);
            Console.WriteLine("after decode");

            Image<Rgba32> decodedImageright = decoder.DecodeRawToImageRgba32(dataright,1024,1016,CompressionFormat.Bc4);
            Console.WriteLine("after decode");

            Mat _left = ConvertImageSharpToGrayscaleMat(decodedImage);
            Mat _right = ConvertImageSharpToGrayscaleMat(decodedImageright);
            Mat left = new Mat(new Size(1024, 1016),MatType.CV_8UC1);
            Mat right = new Mat(new Size(1024, 1016),MatType.CV_8UC1);
          Cv2.Remap(_left,left,undistortionX,undistortionY,InterpolationFlags.Linear,BorderTypes.Constant,Scalar.AliceBlue);
          Cv2.Remap(_right,right,undistortionX,undistortionY,InterpolationFlags.Linear,BorderTypes.Constant,Scalar.AliceBlue);
        CropMat(ref left,0,0,1016,1016);
        CropMat(ref right,0,0,1016,1016);
        Cv2.Resize(left,_left,new Size(960,960));
        Cv2.Resize(right,_right,new Size(960,960));
        left = _left;
        right = _right;
        Mat combined = new Mat();
        Mat[] list = {left,right};
        Cv2.HConcat(list,combined);
        return combined.ToBytes();

    }
    public static void CropMat(ref Mat _mat,int x_start,int y_start,int width,int height)
    {
        // 2. Define the Region of Interest (ROI)
        Rect roi = new Rect(x_start, y_start, width, height);

        // 3. Perform the crop using SubMat (creates a header for the sub-rectangle)
        // Note: 'cropped_img' is a view, use .Clone() if you want an independent copy
        Mat cropped_img = _mat.SubMat(roi);
        _mat = cropped_img;
    }
    public static Mat ConvertImageSharpToGrayscaleMat(Image<Rgba32> image)
    {
        int byteCount = image.Width * image.Height * 4;
        byte[] pixelBytes = new byte[byteCount];
        image.CopyPixelDataTo(pixelBytes);

        GCHandle handle = GCHandle.Alloc(pixelBytes, GCHandleType.Pinned);
        try
        {
            IntPtr ptr = handle.AddrOfPinnedObject();

            // 1. Create a 4-channel Mat from the RGBA data (ImageSharp memory layout is RGBA)
            using (Mat rgbaMat = Mat.FromPixelData(image.Height, image.Width, MatType.CV_8UC4, ptr))
            {
                // 2. Create a destination Mat for the single channel
                Mat grayMat = new Mat();

                // 3. Extract the first channel (Channel 0: Red in RGBA) which contains the BC4 data.
                Cv2.ExtractChannel(rgbaMat, grayMat, 0);

                return grayMat;
            }
        }
        finally
        {
            if (handle.IsAllocated)
            {
                handle.Free();
            }
        }
    }
}