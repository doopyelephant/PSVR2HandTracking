using System;
using System.Collections.Generic;
using System.Linq;
using OpenCvSharp;

public static class RemapRingDiagnostics
{
    // map: CV_32FC2 mapping out->src, src: source image
    // saves outputs to disk and prints diagnostics
    public static void RunAllDiagnostics(Mat src, Mat map, string outPrefix)
    {
        if (map.Empty()) throw new ArgumentException("map empty");
        if (map.Type() != MatType.CV_32FC2) throw new ArgumentException("map must be CV_32FC2");

        // 1) Compare interpolation modes
        Console.WriteLine("Remapping with different interpolations...");
        SaveRemaps(src, map, outPrefix);

        // 2) Remap on float source to reduce quantization during interpolation
        Console.WriteLine("Remapping using float32 source (remap in float)...");
        Mat srcF = new Mat();
        src.ConvertTo(srcF, MatType.CV_32FC3, 1.0); // keep same dynamic range in float
        Mat dstF = new Mat();
        Cv2.Remap(srcF, dstF, map, new Mat(), InterpolationFlags.Linear, BorderTypes.Constant, Scalar.All(0));
        Mat dstF8 = new Mat();
        dstF.ConvertTo(dstF8, MatType.CV_8UC3, 1.0);
        Cv2.ImWrite($"{outPrefix}_remap_float_linear.png", dstF8);

        // 3) Slight blur & remap (checks source quantization)
        Mat srcBlur = new Mat();
        Cv2.GaussianBlur(src, srcBlur, new Size(3,3), 0.5);
        Mat dstBlur = new Mat();
        Cv2.Remap(srcBlur, dstBlur, map, new Mat(), InterpolationFlags.Linear, BorderTypes.Constant, Scalar.All(0));
        Cv2.ImWrite($"{outPrefix}_remap_blur_linear.png", dstBlur);

        // 4) Sample radial line and print mapped coordinates + fractional parts + integer neighbors
        Console.WriteLine("Sampling radial line from center to right:");
        AnalyzeRadialLine(map, src, outPrefix);

        // 5) Analyze whole map quantisation: unique integer neighbor count heatmap and fractional statistics
        Console.WriteLine("Analyzing map integer neighbor usage and fractional-part distributions...");
        AnalyzeMapQuantization(map, outPrefix);
    }

    static void SaveRemaps(Mat src, Mat map, string prefix)
    {
        var modes = new (InterpolationFlags flag, string name)[] {
            (InterpolationFlags.Nearest, "nearest"),
            (InterpolationFlags.Linear, "linear"),
            (InterpolationFlags.Cubic, "cubic"),
            (InterpolationFlags.Lanczos4, "lanczos4")
        };

        foreach (var (flag, name) in modes)
        {
            Mat dst = new Mat();
            Cv2.Remap(src, dst, map, new Mat(), flag, BorderTypes.Constant, Scalar.All(0));
            Cv2.ImWrite($"{prefix}_remap_{name}.png", dst);
            Console.WriteLine($"Saved {prefix}_remap_{name}.png");
        }
    }

    static void AnalyzeRadialLine(Mat map, Mat src, string prefix)
    {
        int cx = map.Cols / 2, cy = map.Rows / 2;
        int steps = Math.Min(map.Cols - cx - 1, 400);
        var rows = new List<string>();
        int sampleCount = Math.Min(200, steps);
        for (int i = 0; i < sampleCount; i++)
        {
            int x = cx + i * (map.Cols - cx - 1) / (sampleCount - 1);
            Vec2f v = map.At<Vec2f>(cy, x);
            float sx = v.Item0, sy = v.Item1;
            float fx = sx - (float)Math.Floor(sx);
            float fy = sy - (float)Math.Floor(sy);
            int ix = (int)Math.Floor(sx), iy = (int)Math.Floor(sy);

            // read source pixel(s) around location for quick check (if inside)
            Vec3b p00 = new Vec3b(0,0,0), p10 = p00, p01 = p00, p11 = p00;
            bool inside = ix >= 0 && iy >= 0 && ix+1 < src.Cols && iy+1 < src.Rows;
            if (inside && src.Type() == MatType.CV_8UC3)
            {
                p00 = src.At<Vec3b>(iy, ix);
                p10 = src.At<Vec3b>(iy, Math.Min(ix+1, src.Cols-1));
                p01 = src.At<Vec3b>(Math.Min(iy+1, src.Rows-1), ix);
                p11 = src.At<Vec3b>(Math.Min(iy+1, src.Rows-1), Math.Min(ix+1, src.Cols-1));
            }

            rows.Add($"{i,3}: out({x},{cy}) -> src({sx:F3},{sy:F3}) floor({ix},{iy}) frac({fx:F3},{fy:F3}) inside={inside} p00={p00}");
            if (i < 30) Console.WriteLine(rows.Last());
        }
        System.IO.File.WriteAllLines($"{prefix}_radial_sample.txt", rows);
        Console.WriteLine($"Wrote radial sample to {prefix}_radial_sample.txt (first lines printed above).");
    }

    static void AnalyzeMapQuantization(Mat map, string prefix)
    {
        int h = map.Rows, w = map.Cols;
        // count unique integer neighbour pairs used by the map (floor(sx), floor(sy))
        var used = new HashSet<(int, int)>();
        // fractional part histograms
        int bins = 256;
        double[] fracHistX = new double[bins];
        double[] fracHistY = new double[bins];

        for (int y = 0; y < h; y++)
        {
            for (int x = 0; x < w; x++)
            {
                Vec2f v = map.At<Vec2f>(y, x);
                float sx = v.Item0, sy = v.Item1;
                if (float.IsNaN(sx) || float.IsNaN(sy) || float.IsInfinity(sx) || float.IsInfinity(sy)) continue;
                int ix = (int)Math.Floor(sx), iy = (int)Math.Floor(sy);
                used.Add((ix, iy));
                float fx = sx - (float)Math.Floor(sx);
                float fy = sy - (float)Math.Floor(sy);
                int bx = (int)Math.Floor(fx * (bins - 1));
                int by = (int)Math.Floor(fy * (bins - 1));
                fracHistX[bx]++;
                fracHistY[by]++;
            }
        }

        Console.WriteLine($"Unique integer neighbor coords used: {used.Count} (map size {w*h})");
        // Save heatmap of counts per integer neighbor by bounding box of used coords
        int minX = used.Min(p => p.Item1), maxX = used.Max(p => p.Item1); // careful: stored (ix,iy) as (x,y)? original used (ix,iy)
        int minY = used.Min(p => p.Item2), maxY = used.Max(p => p.Item2);

        // create histogram visualization (fractional part)
        Mat fracX = new Mat(1, bins, MatType.CV_32F, Scalar.All(0));
        Mat fracY = new Mat(1, bins, MatType.CV_32F, Scalar.All(0));
        for (int i = 0; i < bins; i++) { fracX.Set(0, i, (float)fracHistX[i]); fracY.Set(0, i, (float)fracHistY[i]); }
        Cv2.Normalize(fracX, fracX, 0, 255, NormTypes.MinMax);
        Cv2.Normalize(fracY, fracY, 0, 255, NormTypes.MinMax);
        Mat fracX8 = new Mat();
        Mat fracY8 = new Mat();
        fracX.ConvertTo(fracX8, MatType.CV_8UC1);
        fracY.ConvertTo(fracY8, MatType.CV_8UC1);
        Cv2.Resize(fracX8, fracX8, new Size(bins, 64), 0, 0, InterpolationFlags.Nearest);
        Cv2.Resize(fracY8, fracY8, new Size(bins, 64), 0, 0, InterpolationFlags.Nearest);
        Cv2.ApplyColorMap(fracX8, fracX8, ColormapTypes.Plasma);
        Cv2.ApplyColorMap(fracY8, fracY8, ColormapTypes.Plasma);
        Cv2.ImWrite($"{prefix}_fracHistX.png", fracX8);
        Cv2.ImWrite($"{prefix}_fracHistY.png", fracY8);

        // Print some stats about fractional parts distribution
        double sumFx = fracHistX.Sum(), sumFy = fracHistY.Sum();
        Console.WriteLine($"Fractional parts - total samples: X={sumFx}, Y={sumFy}");
        Console.WriteLine("Fractional hist peaks (X): top 5 bins");
        var topX = fracHistX.Select((v,i)=> (v,i)).OrderByDescending(t=>t.v).Take(5);
        foreach (var t in topX) Console.WriteLine($" bin {t.i} count {t.v}");
        var topY = fracHistY.Select((v,i)=> (v,i)).OrderByDescending(t=>t.v).Take(5);
        Console.WriteLine("Fractional hist peaks (Y): top 5 bins");
        foreach (var t in topY) Console.WriteLine($" bin {t.i} count {t.v}");

        Console.WriteLine($"Saved fractional histograms: {prefix}_fracHistX.png, {prefix}_fracHistY.png");

        // If many values concentrated in a few fractional bins -> quantisation causing rings
        // If used.Count is small relative to w*h, there are large regions mapping to same integer neighbor -> cause of banding
    }
}