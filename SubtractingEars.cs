using CGUtilities;
using System;
using System.Collections.Generic;
using System.Linq;

namespace CGAlgorithms.Algorithms.PolygonTriangulation
{
    class SubtractingEars : Algorithm
    {
        public override void Run(
            List<Point> points, List<Line> lines, List<Polygon> polygons,
            ref List<Point> outPoints, ref List<Line> outLines, ref List<Polygon> outPolygons)
        {

            Polygon polygon = new Polygon(lines);
            //polygon = MakePolygonCounterClockwise(polygon);

            // Extract points from the polygon's lines
            List<Point> extractedPoints = new List<Point>();
            foreach (var line in polygon.lines)
            {
                if (extractedPoints.Count == 0 || !line.Start.Equals(extractedPoints.Last()))
                    extractedPoints.Add(line.Start);
                if (!line.End.Equals(extractedPoints.First()))
                    extractedPoints.Add(line.End);
            }


            // Ensure polygon has at least 3 vertices
            if (extractedPoints.Count < 3)
                throw new ArgumentException("A polygon must have at least 3 vertices.");

            while (extractedPoints.Count > 3)
            {
                bool earClipped = false;
                for (int i = 0; i < extractedPoints.Count; i++)
                {
                    Point current = extractedPoints[i];
                    Point prev = getitem(extractedPoints, i - 1);
                    Point next = getitem(extractedPoints, i + 1);

                    // Check if the diagonal from prev to next is inside the polygon
                    if (IsDiagonalValid(prev, next, extractedPoints))
                    {
                        
                        bool isEar = true;
                        for (int j = 0; j < extractedPoints.Count; j++)
                        {
                            if (j == i || j == (i - 1 + extractedPoints.Count) % extractedPoints.Count || j == (i + 1) % extractedPoints.Count)
                                continue;

                            if (HelperMethods.PointInTriangle(extractedPoints[j], prev, current, next) == Enums.PointInPolygon.Inside)
                            {
                                isEar = false;
                                break;
                            }
                        }

                        if (isEar)
                        {
                            // Clip the ear
                            outLines.Add(new Line(prev, current));
                            outLines.Add(new Line(current, next));
                            outLines.Add(new Line(next, prev));

                            Polygon triangle = new Polygon();
                            triangle.lines.Add(new Line(prev, current));
                            triangle.lines.Add(new Line(current, next));
                            triangle.lines.Add(new Line(next, prev));
                            outPolygons.Add(triangle);

                                extractedPoints.RemoveAt(i);
                                
                            
                            earClipped = true;

                        }
                    }
                }

                if (!earClipped)
                {
                    throw new InvalidOperationException("Failed to clip an ear. Input might not be a simple polygon.");
                }
            }

            // Add the last remaining triangle
            if (extractedPoints.Count == 3)
            {
                Point a = extractedPoints[0];
                Point b = extractedPoints[1];
                Point c = extractedPoints[2];

                outLines.Add(new Line(a, b));
                outLines.Add(new Line(b, c));
                outLines.Add(new Line(c, a));

                Polygon finalTriangle = new Polygon();
                finalTriangle.lines.Add(new Line(a, b));
                finalTriangle.lines.Add(new Line(b, c));
                finalTriangle.lines.Add(new Line(c, a));
                outPolygons.Add(finalTriangle);
            }
        }

        public override string ToString() => "Subtracting Ears";

        public static T getitem<T>(List<T> list, int index)
        {
            return list[(index + list.Count) % list.Count];
        }
    private bool IsDiagonalValid(Point prev, Point next, List<Point> polygon)
        {
            Line diagonal = new Line(prev, next);

            for (int i = 0; i < polygon.Count; i++)
            {
                int j = (i + 1) % polygon.Count;
                Point pi = polygon[i];
                Point pj = polygon[j];

  
                if (pi.Equals(prev) || pi.Equals(next) || pj.Equals(prev) || pj.Equals(next))
                    continue;

                Line edge = new Line(pi, pj);
                if (LinesIntersect(diagonal, edge))
                    return false;
            }

            Point midpoint = new Point(
                (prev.X + next.X) / 2,
                (prev.Y + next.Y) / 2
            );

            return IsPointInPolygon(midpoint, polygon) == Enums.PointInPolygon.Inside;
        }

        private bool LinesIntersect(Line l1, Line l2)
        {

            Point v1 = new Point(l1.End.X - l1.Start.X, l1.End.Y - l1.Start.Y);
            Point v2 = new Point(l2.End.X - l2.Start.X, l2.End.Y - l2.Start.Y);

     
            double cross1 = HelperMethods.CrossProduct(v1, new Point(l2.Start.X - l1.Start.X, l2.Start.Y - l1.Start.Y));
            double cross2 = HelperMethods.CrossProduct(v1, new Point(l2.End.X - l1.Start.X, l2.End.Y - l1.Start.Y));
            double cross3 = HelperMethods.CrossProduct(v2, new Point(l1.Start.X - l2.Start.X, l1.Start.Y - l2.Start.Y));
            double cross4 = HelperMethods.CrossProduct(v2, new Point(l1.End.X - l2.Start.X, l1.End.Y - l2.Start.Y));


            return (cross1 * cross2 < 0) && (cross3 * cross4 < 0);
        }

        private Enums.PointInPolygon IsPointInPolygon(Point point, List<Point> polygon)
        {
            bool inside = false;
            for (int i = 0, j = polygon.Count - 1; i < polygon.Count; j = i++)
            {
                Point pi = polygon[i];
                Point pj = polygon[j];

                // Check if point is on an edge
                if (HelperMethods.PointOnSegment(point, pi, pj))
                    return Enums.PointInPolygon.OnEdge;

                // Ray casting algorithm
                if (((pi.Y > point.Y) != (pj.Y > point.Y)) &&
                    (point.X < (pj.X - pi.X) * (point.Y - pi.Y) / (pj.Y - pi.Y) + pi.X))
                {
                    inside = !inside;
                }
            }

            return inside ? Enums.PointInPolygon.Inside : Enums.PointInPolygon.Outside;
        }
    }
}
