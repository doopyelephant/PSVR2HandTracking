using System;
using OpenCvSharp;
using PSVR2HandTracking;

public static class DistortionMeshes
{

// Ported from get_distorted_point: computes distorted normalized coords from undistorted ones
    public static Point2d GetDistortedPoint(double x, double y, CameraParameters parameters)
    {
        double[] p = parameters.Coeffs;
        double xSq = x * x;
        double ySq = y * y;
        double rSq = xSq + ySq;
        double twoXY = 2.0 * x * y;
        double rSqP4 = rSq * rSq * rSq * rSq; // r^8? rSqP4 = r^4, etc.
        double rSqP5 = rSqP4 * rSq; // r^5
        double rSqP6 = rSqP5 * rSq; // r^6

        // Numerator polynomial
        double numPoly = 1.0 +
                         (((p[4] * rSq + p[1]) * rSq + p[0]) * rSq) +
                         (p[16] * rSqP6) +
                         (p[15] * rSqP5) +
                         (p[14] * rSqP4);

        // Denominator polynomial
        double denPoly = 1.0 +
                         (((p[7] * rSq + p[6]) * rSq + p[5]) * rSq) +
                         (p[19] * rSqP6) +
                         (p[18] * rSqP5) +
                         (p[17] * rSqP4);

        double radialScale = (Math.Abs(denPoly) > 1e-9) ? (numPoly / denPoly) : 1.0;

        // Distorted terms
        double distortedXTerm = (((p[9] * rSq + p[8]) * rSq)) +
                                ((xSq + xSq + rSq) * p[3]) +
                                (radialScale * x) +
                                (twoXY * p[2]);

        double distortedYTerm = (((p[11] * rSq + p[10]) * rSq)) +
                                (twoXY * p[3]) +
                                ((ySq + ySq + rSq) * p[2]) +
                                (radialScale * y);

        // Rotations: Manual 3x3 matrix (Rx * Ry)
        double cosRx = Math.Cos(p[12]);
        double sinRx = Math.Sin(p[12]);
        double cosRy = Math.Cos(p[13]);
        double sinRy = Math.Sin(p[13]);

        double R00 = cosRy;
        double R01 = sinRx * sinRy;
        double R02 = cosRx * sinRy;
        double R10 = 0.0;
        double R11 = cosRx;
        double R12 = -sinRx;
        double R20 = -sinRy;
        double R21 = sinRx * cosRy;
        double R22 = cosRx * cosRy;

        // Apply to vector
        double v0 = distortedXTerm;
        double v1 = distortedYTerm;
        double v2 = 1.0;
        double t0 = R00 * v0 + R01 * v1 + R02 * v2;
        double t1 = R10 * v0 + R11 * v1 + R12 * v2;
        double t2 = R20 * v0 + R21 * v1 + R22 * v2;

        double w = t2;
        if (Math.Abs(w) < 1e-9)
        {
            return new Point2d(-10.0, -10.0);
        }

        return new Point2d(t0 / w, t1 / w);
    }

// Creates undistortion maps for Cv2.Remap
    public static void CreateUndistortionMaps(
        int imageWidth, int imageHeight,
        float zoomFactor,
        CameraIntrinsics intrinsics,
        CameraParameters parameters,
        out Mat mapX, out Mat mapY)
    {
        mapX = new Mat(imageHeight, imageWidth, MatType.CV_32FC1);
        mapY = new Mat(imageHeight, imageWidth, MatType.CV_32FC1);

        for (int i = 0; i < imageHeight; ++i)
        {
            for (int j = 0; j < imageWidth; ++j)
            {
                double pxOut = j;
                double pyOut = i;

                double xNorm = (pxOut - intrinsics.Cx) / intrinsics.Fx;
                double yNorm = (pyOut - intrinsics.Cy) / intrinsics.Fy;

                Point2d distortedNorm = GetDistortedPoint(xNorm * zoomFactor, yNorm * zoomFactor, parameters);

                double srcPx = distortedNorm.X * intrinsics.Fx + intrinsics.Cx;
                double srcPy = distortedNorm.Y * intrinsics.Fy + intrinsics.Cy;

                mapX.Set<float>(i, j, (float)srcPx);
                mapY.Set<float>(i, j, (float)srcPy);
            }
        }
    }
}