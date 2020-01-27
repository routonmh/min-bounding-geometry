using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;

namespace MinBoundingGeo.Algorithms
{
    /// <summary>
    /// Adapted/Stolen from: https://stackoverflow.com/questions/25190164/graham-scan-issue-at-high-amount-of-points/25204997#25204997
    /// </summary>
    public class GrahamScan
    {
        public List<Vector3> PointsRaw;

        public List<Vector3> EdgePoints { get; }

        private static readonly double RELATIVE_TOLERANCE = 1e-10;

        private enum RemovalFlag
        {
            None,
            MidPoint,
            EndPoint
        };

        public GrahamScan(List<Vector3> pointsRaw)
        {
            PointsRaw = pointsRaw;
            EdgePoints = grahamScanCompute(pointsRaw).ToList();
        }

        /// <summary>
        ///
        /// </summary>
        /// <param name="initialPoints"></param>
        /// <returns></returns>
        /// <exception cref="Exception"></exception>
        private static IList<Vector3> grahamScanCompute(IList<Vector3> initialPoints)
        {
            if (initialPoints.Count < 2)
                return initialPoints.ToList();

            // Find point with minimum y; if more than one, minimize x also.
            int iMin = Enumerable.Range(0, initialPoints.Count).Aggregate((jMin, jCur) =>
            {
                if (initialPoints[jCur].Y < initialPoints[jMin].Y)
                    return jCur;
                if (initialPoints[jCur].Y > initialPoints[jMin].Y)
                    return jMin;
                if (initialPoints[jCur].X < initialPoints[jMin].X)
                    return jCur;
                return jMin;
            });

            // Sort them by polar angle from iMin,
            var sortQuery = Enumerable.Range(0, initialPoints.Count)
                .Where((i) => (i != iMin)) // Skip the min point
                .Select((i) =>
                    new KeyValuePair<double, Vector3>(
                        Math.Atan2(initialPoints[i].Y - initialPoints[iMin].Y,
                            initialPoints[i].X - initialPoints[iMin].X), initialPoints[i]))
                .OrderBy((pair) => pair.Key)
                .Select((pair) => pair.Value);
            List<Vector3> points = new List<Vector3>(initialPoints.Count);
            points.Add(initialPoints[iMin]); // Add minimum point
            points.AddRange(sortQuery); // Add the sorted points.

            int m = 0;
            for (int i = 1, n = points.Count; i < n; i++)
            {
                bool keepNewPoint = true;
                if (m == 0)
                {
                    // Find at least one point not coincident with points[0]
                    keepNewPoint = !nearlyEqual(points[0], points[i]);
                }
                else
                {
                    while (true)
                    {
                        var flag = whichToRemoveFromBoundary(points[m - 1], points[m], points[i]);
                        if (flag == RemovalFlag.None)
                            break;
                        else if (flag == RemovalFlag.MidPoint)
                        {
                            if (m > 0)
                                m--;
                            if (m == 0)
                                break;
                        }
                        else if (flag == RemovalFlag.EndPoint)
                        {
                            keepNewPoint = false;
                            break;
                        }
                        else
                            throw new Exception("Unknown RemovalFlag");
                    }
                }

                if (keepNewPoint)
                {
                    m++;
                    swap(points, m, i);
                }
            }

            // points[M] is now the last point in the boundary.  Remove the remainder.
            points.RemoveRange(m + 1, points.Count - m - 1);
            return points;
        }

        /// <summary>
        ///
        /// </summary>
        /// <param name="list"></param>
        /// <param name="i"></param>
        /// <param name="j"></param>
        /// <typeparam name="T"></typeparam>
        private static void swap<T>(IList<T> list, int i, int j)
        {
            if (i != j)
            {
                T temp = list[i];
                list[i] = list[j];
                list[j] = temp;
            }
        }

        /// <summary>
        ///
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        private static bool nearlyEqual(Vector3 a, Vector3 b)
        {
            return nearlyEqual(a.X, b.X) && nearlyEqual(a.Y, b.Y);
        }

        /// <summary>
        ///
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        private static bool nearlyEqual(double a, double b)
        {
            return nearlyEqual(a, b, RELATIVE_TOLERANCE);
        }

        /// <summary>
        ///
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <param name="epsilon"></param>
        /// <returns></returns>
        private static bool nearlyEqual(double a, double b, double epsilon)
        {
            // See here: http://floating-point-gui.de/errors/comparison/
            if (a == b)
            {
                // shortcut, handles infinities
                return true;
            }

            double absA = Math.Abs(a);
            double absB = Math.Abs(b);
            double diff = Math.Abs(a - b);
            double sum = absA + absB;
            if (diff < 4 * double.Epsilon || sum < 4 * double.Epsilon)
                // a or b is zero or both are extremely close to it
                // relative error is less meaningful here
                return true;

            // use relative error
            return diff / (absA + absB) < epsilon;
        }

        private static double ccw(Vector3 p1, Vector3 p2, Vector3 p3)
        {
            // Compute (p2 - p1) X (p3 - p1)
            double cross1 = (p2.X - p1.X) * (p3.Y - p1.Y);
            double cross2 = (p2.Y - p1.Y) * (p3.X - p1.X);
            if (nearlyEqual(cross1, cross2))
                return 0;
            return cross1 - cross2;
        }

        /// <summary>
        ///
        /// </summary>
        /// <param name="p1"></param>
        /// <param name="p2"></param>
        /// <param name="p3"></param>
        /// <returns></returns>
        private static RemovalFlag whichToRemoveFromBoundary(Vector3 p1, Vector3 p2, Vector3 p3)
        {
            var cross = ccw(p1, p2, p3);
            if (cross < 0)
                // Remove p2
                return RemovalFlag.MidPoint;
            if (cross > 0)
                // Remove none.
                return RemovalFlag.None;
            // Check for being reversed using the dot product off the difference vectors.
            var dotp = (p3.X - p2.X) * (p2.X - p1.X) + (p3.Y - p2.Y) * (p2.Y - p1.Y);
            if (nearlyEqual(dotp, 0.0))
                // Remove p2
                return RemovalFlag.MidPoint;
            if (dotp < 0)
                // Remove p3
                return RemovalFlag.EndPoint;
            else
                // Remove p2
                return RemovalFlag.MidPoint;
        }
    }
}