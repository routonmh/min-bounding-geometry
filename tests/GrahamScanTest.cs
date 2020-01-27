using System;
using System.Collections.Generic;
using System.Numerics;
using MinBoundingGeo.Algorithms;
using NUnit.Framework;

namespace Tests
{
    public class Tests
    {
        [SetUp]
        public void Setup()
        {
        }

        [Test]
        public void TestQuad()
        {
            List<Vector3> points = new List<Vector3>();
            points.Add(new Vector3(1,3,0));
            points.Add(new Vector3(-1, 3, 0));
            points.Add(new Vector3(1, 1, 0));
            points.Add(new Vector3(-1, 1, 0));
            points.Add(new Vector3(2,4,0));
            points.Add(new Vector3(-2, 4, 0));
            points.Add(new Vector3(2, 0, 0));
            points.Add(new Vector3(-2, 0,0));

            GrahamScan scan = new GrahamScan(points);

            Console.WriteLine(scan.EdgePoints.Count + " points.");

            scan.EdgePoints.ForEach(it => Console.WriteLine(it));
        }
    }
}