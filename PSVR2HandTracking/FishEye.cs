using System;
using System.IO;
using System.Text.Json;
using OpenCvSharp;

namespace StereoRectifyFisheye
{
    // --- 1. Define C# classes to match the JSON structure ---

    public class Resolution
    {
        public int width { get; set; }
        public int height { get; set; }
    }

    public class Intrinsics
    {
        public double fx { get; set; }
        public double fy { get; set; }
        public double cx { get; set; }
        public double cy { get; set; }
    }

    public class Distortion
    {
        public double k1 { get; set; }
        public double k2 { get; set; }
        public double k3 { get; set; }
        public double k4 { get; set; }
    }

    public class Camera
    {
        public string model { get; set; }
        public Intrinsics intrinsics { get; set; }
        public Distortion distortion { get; set; }
        public Resolution resolution { get; set; }
    }

    // New class to hold the opencv_stereo_calibrate data
    public class OpenCvStereoCalibrate
    {
        public double[] rotation { get; set; }
        public double[] translation { get; set; }
        public double[] essential { get; set; }
        public double[] fundamental { get; set; }
    }

    // Updated CalibrationData class
    public class CalibrationData
    {
        public Camera[] cameras { get; set; }
        public OpenCvStereoCalibrate opencv_stereo_calibrate { get; set; }
    }

    class FishEyeUndistort
    {
        public static void Undistort(ref Mat leftImage, ref Mat rightImage, string JsonPath)
        {
            // --- 2. Load and Parse JSON Data ---

            string jsonString = File.ReadAllText(JsonPath);
            // Use JsonSerializerOptions to handle snake_case from JSON
            var options = new JsonSerializerOptions
            {
                PropertyNameCaseInsensitive = true,
                PropertyNamingPolicy = JsonNamingPolicy.SnakeCaseLower
            };
            var calibData = JsonSerializer.Deserialize<CalibrationData>(jsonString, options);

            // Assuming the JSON has two cameras (0=Left, 1=Right)
            var camL = calibData.cameras[0];
            var camR = calibData.cameras[1];

            Size imageSize = new Size(camL.resolution.width, camL.resolution.height);

            // Build Camera Matrices (K)
            Mat K1 = BuildCameraMatrix(camL.intrinsics);
            Mat K2 = BuildCameraMatrix(camR.intrinsics);

            // Build Distortion Coefficients (D)
            Mat D1 = BuildDistortionCoefficients(camL.distortion);
            Mat D2 = BuildDistortionCoefficients(camR.distortion);

            // --- 3. Get Stereo Extrinsics (R and T) from parsed JSON data ---
            // This replaces the old hardcoded values

            // The rotation matrix is stored as a 9-element array (row-major)
            Mat R = new Mat(3, 3, MatType.CV_64FC1);
            R.Set<double>(0, 0, calibData.opencv_stereo_calibrate.rotation[0]);
            R.Set<double>(0, 1, calibData.opencv_stereo_calibrate.rotation[1]);
            R.Set<double>(0, 2, calibData.opencv_stereo_calibrate.rotation[2]);
            R.Set<double>(1, 0, calibData.opencv_stereo_calibrate.rotation[3]);
            R.Set<double>(1, 1, calibData.opencv_stereo_calibrate.rotation[4]);
            R.Set<double>(1, 2, calibData.opencv_stereo_calibrate.rotation[5]);
            R.Set<double>(2, 0, calibData.opencv_stereo_calibrate.rotation[6]);
            R.Set<double>(2, 1, calibData.opencv_stereo_calibrate.rotation[7]);
            R.Set<double>(2, 2, calibData.opencv_stereo_calibrate.rotation[8]);

            // The translation vector is stored as a 3-element array
            Mat T = new Mat(3, 1, MatType.CV_64FC1);
            T.Set<double>(0, 0, calibData.opencv_stereo_calibrate.translation[0]);
            T.Set<double>(1, 0, calibData.opencv_stereo_calibrate.translation[1]);
            T.Set<double>(2, 0, calibData.opencv_stereo_calibrate.translation[2]);


            // --- 4. Stereo Rectification ---

            Mat R1 = new Mat(), R2 = new Mat();
            Mat P1 = new Mat(), P2 = new Mat();
            Mat Q = new Mat();

            // Balance can be adjusted. 0.0 zooms in to remove all black areas, 1.0 shows all pixels.
            double balance = 0.0; // Start with 0.0 for maximum zoom/crop, then adjust if needed.

            Cv2.FishEye.StereoRectify(
                K1, D1, K2, D2,
                imageSize,
                R, T,
                R1, R2, P1, P2, Q,
                FishEyeCalibrationFlags.None,
                newImageSize: imageSize, // Explicitly set newSize
                balance: balance
            );

            // --- 5. Compute Undistortion Maps ---

            Mat map1L = new Mat(), map2L = new Mat();
            Cv2.FishEye.InitUndistortRectifyMap(K1, D1, R1, P1, imageSize, MatType.CV_16SC2.Value, map1L, map2L);

            Mat map1R = new Mat(), map2R = new Mat();
            Cv2.FishEye.InitUndistortRectifyMap(K2, D2, R2, P2, imageSize, MatType.CV_16SC2.Value, map1R, map2R);

            // --- 6. Remap Images ---

            // Important: Resize input images to match calibration resolution if they aren't already
            if (leftImage.Size() != imageSize)
            {
                Cv2.Resize(leftImage, leftImage, imageSize);
            }
            if (rightImage.Size() != imageSize)
            {
                Cv2.Resize(rightImage, rightImage, imageSize);
            }

            Mat leftRectified = new Mat();
            Mat rightRectified = new Mat();

            Cv2.Remap(leftImage, leftRectified, map1L, map2L, InterpolationFlags.Linear);
            Cv2.Remap(rightImage, rightRectified, map1R, map2R, InterpolationFlags.Linear);

            // --- 7. Display Results ---
            Mat combined = new Mat();
            Cv2.HConcat(leftRectified, rightRectified, combined);

            // Draw horizontal lines to visualize alignment
            for (int i = 1; i < 4; i++)
            {
                int y = i * imageSize.Height / 4;
                Cv2.Line(combined, new Point(0, y), new Point(imageSize.Width * 2, y), new Scalar(0, 255, 0), 1);
            }
            combined.SaveImage("Rectified Pair.png");
            Cv2.ImShow("Rectified Pair", combined);

            Console.WriteLine("Stereo rectification complete. Press any key to close images.");
            Cv2.WaitKey();
            Cv2.DestroyAllWindows();


            Cv2.DestroyAllWindows();
        }

        static Mat BuildCameraMatrix(Intrinsics intrinsics)
        {
            Mat K = Mat.Eye(3, 3, MatType.CV_64FC1);
            K.Set<double>(0, 0, intrinsics.fx);
            K.Set<double>(0, 2, intrinsics.cx);
            K.Set<double>(1, 1, intrinsics.fy);
            K.Set<double>(1, 2, intrinsics.cy);
            return K;
        }

        static Mat BuildDistortionCoefficients(Distortion distortion)
        {
            Mat D = new Mat(4, 1, MatType.CV_64FC1);
            D.Set<double>(0, 0, distortion.k1);
            D.Set<double>(1, 0, distortion.k2);
            D.Set<double>(2, 0, distortion.k3);
            D.Set<double>(3, 0, distortion.k4);
            return D;
        }
    }
}