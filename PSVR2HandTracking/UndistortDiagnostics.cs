using System;
using System.Linq;
using OpenCvSharp;

public static class UndistortDiagnostics
{
    // Inspect a CV_32FC2 map (or two CV_32FC1 mats converted to CV_32FC2)
    public static void AnalyzeMap(Mat map, string prefix)
    {
        if (map.Empty()) { Console.WriteLine("Map is empty"); return; }
        Console.WriteLine($"Map Type: {map.Type()}  Channels: {map.Channels()}  Size: {map.Size()}");

        // If CV_32FC2, split channels to get min/max
        Mat[] channels = Cv2.Split(map);
        if (channels.Length >= 2)
        {
            Cv2.MinMaxLoc(channels[0], out double minX, out double maxX);
            Cv2.MinMaxLoc(channels[1], out double minY, out double maxY);
            Console.WriteLine($"srcX range: {minX:F3} .. {maxX:F3}");
            Console.WriteLine($"srcY range: {minY:F3} .. {maxY:F3}");

            // Save visualizations (normalized to 0..255) to detect banding in map values
            Mat visX = new Mat();
            Mat visY = new Mat();
            channels[0].ConvertTo(visX, MatType.CV_8UC1, 255.0 / Math.Max(1.0, maxX - minX), -minX * 255.0 / Math.Max(1.0, maxX - minX));
            channels[1].ConvertTo(visY, MatType.CV_8UC1, 255.0 / Math.Max(1.0, maxY - minY), -minY * 255.0 / Math.Max(1.0, maxY - minY));
            Cv2.ApplyColorMap(visX, visX, ColormapTypes.Hot);
            Cv2.ApplyColorMap(visY, visY, ColormapTypes.Hot);
            Cv2.ImWrite(prefix + "_mapX_vis.png", visX);
            Cv2.ImWrite(prefix + "_mapY_vis.png", visY);

            // Sample radial line (center -> right) to show continuity
            int cx = map.Cols / 2, cy = map.Rows / 2;
            int steps = Math.Min(map.Cols, 200);
            Console.WriteLine("Radial sample (center -> right) first 30 entries:");
            for (int i = 0; i < Math.Min(30, steps); i++)
            {
                int x = cx + i * (map.Cols - cx) / steps;
                Vec2f v = map.At<Vec2f>(cy, x);
                Console.WriteLine($"{i,2}: out({x},{cy}) -> src({v.Item0:F3},{v.Item1:F3})");
            }
        }
        else
        {
            Console.WriteLine("Map does not have two channels; expected CV_32FC2.");
        }
    }

    // Alternate: build CV_32FC2 from two CV_32FC1 mats and analyze
    public static Mat BuildMapFromXY(Mat mapX, Mat mapY)
    {
        if (mapX.Empty() || mapY.Empty()) throw new ArgumentException("Empty mapX/mapY");
        if (mapX.Size() != mapY.Size()) throw new ArgumentException("mapX/mapY size mismatch");
        Mat map = new Mat(mapX.Rows, mapX.Cols, MatType.CV_32FC2);
        for (int y = 0; y < map.Rows; y++)
        {
            for (int x = 0; x < map.Cols; x++)
            {
                float sx = mapX.At<float>(y, x);
                float sy = mapY.At<float>(y, x);
                map.Set(y, x, new Vec2f(sx, sy));
            }
        }
        return map;
    }

    // Quick helper to verify remap call and interpolation mode
    public static void RemapAndSave(Mat src, Mat map, string outPath)
    {
        Mat dst = new Mat();
        // use linear interpolation
        Cv2.Remap(src, dst, map, new Mat(), InterpolationFlags.Linear, BorderTypes.Constant, Scalar.Black);
        Cv2.ImWrite(outPath, dst);
    }
}