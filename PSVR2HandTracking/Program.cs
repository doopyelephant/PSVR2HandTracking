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
using PSVR2HandTracking;
using Size = OpenCvSharp.Size;

public static class Program
{
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

    private static MemoryMappedFile _shareFile;
    private static EventWaitHandle _imageEvent;
    private static MemoryMappedViewStream _imageViewStream;
    private static int _index = 0;

    public static void Main()
    {
        _shareFile = MemoryMappedFile.OpenExisting("SHARE_VRT2_WIN");
        _imageEvent = new EventWaitHandle(false, EventResetMode.AutoReset, "SHARE_VRT2_WIN_IMAGE_EVT");

        var data = new SharedMemoryData();
        var cameraParams = new CameraParameters();
        var intrinsics = new CameraIntrinsics();
        CalibrationReader.GetDistortionConfig(data, 0, ref cameraParams, ref intrinsics);

        //Update();

        Exit();
    }

    private static void Exit()
    {
        _shareFile.Dispose();
    }

    private static void Update()
    {
        _imageEvent.WaitOne();

        Console.WriteLine(_index);
        Console.WriteLine(_offsets[_index]);
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
            Image<Rgba32> decodedImage = decoder.DecodeRawToImageRgba32(dataleft,1024,1000,CompressionFormat.Bc4);
            Console.WriteLine("after decode");

            Image<Rgba32> decodedImageright = decoder.DecodeRawToImageRgba32(dataright,1024,1000,CompressionFormat.Bc4);
            Console.WriteLine("after decode");

            Mat _left = ConvertImageSharpToGrayscaleMat(decodedImage);
            Mat _right = ConvertImageSharpToGrayscaleMat(decodedImageright);
            Mat left = new Mat(new Size(1024, 1000),MatType.CV_8UC1);
            Mat right = new Mat(new Size(1024, 1000),MatType.CV_8UC1);



           if (File.Exists("./calibration.cal"))
           {
               Mat cameraMatrix2 = Mat.Eye(3, 3, MatType.CV_64F);
               double[,] matrix2 = new double[3,3];
               Mat distCoeffs2 = Mat.Zeros(1, 5, (MatType)MatType.CV_64F);
               double[] distortion2 = new double[5];
               string[] contents = File.ReadAllLines("./calibration.cal");
               int i = 1;
               for (; i < distortion2.Length; i++)
               {
                   distortion2[i - 1] = double.Parse(contents[i]);
               }
               for (int j = 0; j < 3; j++)
               {
                   for (int k = 0; k < 3; k++)
                   {
                       matrix2[j, k] = double.Parse(contents[i]);
                       i++;
                   }
               }
               Console.WriteLine("Distortion: ");
               foreach (var item in distortion2)
                   Console.WriteLine(item);
               Console.WriteLine("Matrix: ");
               foreach (var item in matrix2)
               {
                   Console.WriteLine(item);
               }

               Cv2.Undistort(_left, left, cameraMatrix2, distCoeffs2);
           }
           using (new Window("left image", left))

           using (new Window("right image", _left))
           {
               Cv2.WaitKey();
           }
           var stereo = StereoBM.Create(16,15);
           Mat depth = new Mat();
           stereo.Compute(left, right, depth);
           Mat normalized = new Mat();
           Cv2.Normalize(depth, normalized, 0, 255, NormTypes.MinMax);
           using (new Window("depth image", depth))
           {
               Cv2.WaitKey();
           }

           List<Point3f[]> objpoints;
           objpoints = new List<Point3f[]>();
           List<Point2f[]> imagepoints;
           imagepoints = new List<Point2f[]>();
           for (int i = 0; i < 25; i++)
           {
               bool found = Cv2.FindChessboardCorners(left, new Size(10, 7), out var corners);
               Console.WriteLine(found);
               Cv2.DrawChessboardCorners(left, new Size(10, 7), corners, found);
               using (new Window("chess image", left))
               {
                   Cv2.WaitKey();
               }
                Thread.Sleep(1500);

               if (found)
               {
                   Console.WriteLine("Chessboard found!");
                   imagepoints.Add(corners.ToArray());
                   objpoints.Add(CreateChessboardObjectPoints());
               }

               _imageEvent.WaitOne();

               Console.WriteLine(_index);
               Console.WriteLine(_offsets[_index]);
               _imageViewStream = _shareFile.CreateViewStream(_offsets[_index], 0xFE100);
               buffer = new byte[0xFE100];
               _imageViewStream.Read(buffer, 0, 0xFE100);
               dataleft = new byte[0x7F000];
               Array.Copy(buffer, 0x100, dataleft, 0, 0x7F000);
               dataright = new byte[0x7F000];
               Array.Copy(buffer, 0x100 + 0x7F000, dataright, 0, 0x7F000);
               _imageViewStream.Dispose();
               decodedImage = decoder.DecodeRawToImageRgba32(dataleft,1024,1000,CompressionFormat.Bc4);
               Console.WriteLine("after decode");
               decodedImageright = decoder.DecodeRawToImageRgba32(dataright,1024,1000,CompressionFormat.Bc4);
               Console.WriteLine("after decode");
               left = ConvertImageSharpToGrayscaleMat(decodedImage);
               right = ConvertImageSharpToGrayscaleMat(decodedImageright);
           }
           Mat cameraMatrix = Mat.Eye(3, 3, MatType.CV_64F);
           double[,] matrix = new double[3,3];
           Mat distCoeffs = Mat.Zeros(1, 5, (MatType)MatType.CV_64F);
         double[] distortion = new double[5];

          Cv2.CalibrateCamera(objpoints, imagepoints, left.Size(), matrix,distortion,out Vec3d[] rotation,out Vec3d[] translation,CalibrationFlags.None);
Console.WriteLine("calibrated");
Console.WriteLine("Rotation: ");
cameraMatrix.SetRectangularArray(matrix);
distCoeffs.SetArray(distortion);
       foreach (var item in rotation)
           Console.WriteLine(item);
       Console.WriteLine("Translation: ");
       foreach (var item in translation)
           Console.WriteLine(item);
       Console.WriteLine("Distortion: ");
       foreach (var item in distortion)
           Console.WriteLine(item);
       Console.WriteLine("Matrix: ");
       foreach (var item in matrix)
       {
           Console.WriteLine(item);
       }
       Mat dst = new Mat();
       Cv2.Undistort(left,dst,cameraMatrix,distCoeffs);
       using (new Window("undistorted image", dst))
       {
           Cv2.WaitKey();
       }
       left.SaveImage("distorted.png");
       dst.SaveImage("undistorted.png");
       string file = "";

       foreach (var item in distortion)
       {
           file += item + "\n";
       }

       foreach (var item in matrix)
       {
           file += item + "\n";
       }
       File.WriteAllText("./calibration.cal",file);
           StereoRectifyFisheye.FishEyeUndistort.Undistort(ref left,ref right,"C:\\Users\\nolan\\Downloads\\psvr2_threshold.calibration.json");

    }
    public static Mat ConvertImageSharpToMat(Image<Rgba32> image)
    {
        // 1. Prepare the raw pixel data array
        // Rgba32 has 4 bytes per pixel (R, G, B, A)
        int byteCount = image.Width * image.Height * 4;
        byte[] pixelBytes = new byte[byteCount];

        // Copy data from ImageSharp into the C# array
        image.CopyPixelDataTo(pixelBytes);

        GCHandle handle = GCHandle.Alloc(pixelBytes, GCHandleType.Pinned);

        try
        {
            // 2. Safely get the memory address and create the Mat
            IntPtr ptr = handle.AddrOfPinnedObject();

            // The Mat(int rows, int cols, MatType type, IntPtr data) constructor
            // is the correct candidate for wrapping unmanaged data.
            using (Mat rgbaMat = Mat.FromPixelData(image.Height, image.Width, MatType.CV_8UC4, ptr))
            {
                // 3. Convert from RGBA (ImageSharp order) to BGRA (OpenCV order)
                // The Mat created in step 2 is an RGBA Mat.
                Mat bgraMat = new Mat();
                Cv2.CvtColor(rgbaMat, bgraMat, ColorConversionCodes.RGBA2BGRA);

                // bgraMat is a new Mat with a copy of the data, now in the correct format.
                return bgraMat;
            }
        }
        finally
        {
            // 4. Release the pinned handle
            if (handle.IsAllocated)
            {
                handle.Free();
            }
        }
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
    public static Point3f[] CreateChessboardObjectPoints(int cols = 10, int rows = 7, float squareSize = 1.0f)
    {
        var objp = new Point3f[cols * rows];
        int idx = 0;
        for (int y = 0; y < rows; y++)
        {
            for (int x = 0; x < cols; x++)
            {
                objp[idx++] = new Point3f(x * squareSize, y * squareSize, 0f);
            }
        }
        return objp;
    }
}