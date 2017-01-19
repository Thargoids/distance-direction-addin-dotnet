// Copyright 2016 Esri 
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// System
using System;

// Esri
using ESRI.ArcGIS.Geometry;
using ESRI.ArcGIS.Display;
using DistanceAndDirectionLibrary;
using ESRI.ArcGIS.ArcMapUI;
using ESRI.ArcGIS.Carto;
using System.Collections.Generic;

namespace ArcMapAddinDistanceAndDirection.ViewModels
{
    public class CircleViewModel : TabBaseViewModel
    {
        /// <summary>
        /// CTOR
        /// </summary>
        public CircleViewModel()
        {
            //properties
            CircleType = CircleFromTypes.Radius;
        }

        #region Properties

        CircleFromTypes circleType = CircleFromTypes.Radius;
        /// <summary>
        /// Type of circle property
        /// </summary>
        public CircleFromTypes CircleType
        {
            get { return circleType; }
            set
            {
                if (circleType == value)
                    return;

                circleType = value;

                // reset distance
                RaisePropertyChanged(() => Distance);
                RaisePropertyChanged(() => DistanceString);
            }
        }

        TimeUnits timeUnit = TimeUnits.Minutes;
        /// <summary>
        /// Type of time units
        /// </summary>
        public TimeUnits TimeUnit
        {
            get
            {
                return timeUnit;
            }
            set
            {
                if (timeUnit == value)
                {
                    return;
                }
                timeUnit = value;

                UpdateDistance(TravelTimeInSeconds * TravelRateInSeconds, RateUnit);

                RaisePropertyChanged(() => TimeUnit);
            }
        }

        /// <summary>
        /// Property for travel time in seconds
        /// </summary>
        private double TravelTimeInSeconds
        {
            get
            {
                switch (TimeUnit)
                {
                    case TimeUnits.Seconds:
                        {
                            return travelTime;
                        }
                    case TimeUnits.Minutes:
                        {
                            return travelTime * 60;
                        }
                    case TimeUnits.Hours:
                        {
                            return travelTime * 3600;
                        }
                    default:
                        return travelTime;
                }
            }
        }

        /// <summary>
        /// Property for travel rate in seconds
        /// </summary>
        private double TravelRateInSeconds
        {
            get
            {
                switch (RateTimeUnit)
                {
                    case RateTimeTypes.FeetHour:
                    case RateTimeTypes.KilometersHour:
                    case RateTimeTypes.MetersHour:
                    case RateTimeTypes.MilesHour:
                    case RateTimeTypes.NauticalMilesHour:
                        return TravelRate / 3600;
                    default:
                        return TravelRate;
                }
            }
        }

        double travelTime = 0.0;
        /// <summary>
        /// Property for time display
        /// </summary>
        public double TravelTime
        {
            get
            {
                return travelTime;
            }
            set
            {
                if (value < 0.0)
                    throw new ArgumentException(DistanceAndDirectionLibrary.Properties.Resources.AEMustBePositive);

                travelTime = value;

                // we need to make sure we are in the same units as the Distance property before setting
                UpdateDistance(TravelRateInSeconds * TravelTimeInSeconds, RateUnit);

                RaisePropertyChanged(() => TravelTime);
            }
        }

        private void UpdateDistance(double distance, DistanceTypes fromDistanceType)
        {
            Distance = ConvertFromTo(fromDistanceType, LineDistanceType, distance);
            UpdateFeedbackWithGeoCircle();
        }

        double travelRate = 0.0;
        /// <summary>
        /// Property of rate display
        /// </summary>
        public double TravelRate
        {
            get
            {
                return travelRate;
            }
            set
            {
                if (value < 0.0)
                    throw new ArgumentException(DistanceAndDirectionLibrary.Properties.Resources.AEMustBePositive);

                travelRate = value;

                UpdateDistance(TravelRateInSeconds * TravelTimeInSeconds, RateUnit);

                RaisePropertyChanged(() => TravelRate);
            }
        }

        DistanceTypes rateUnit = DistanceTypes.Meters;
        public DistanceTypes RateUnit
        {
            get
            {
                switch (RateTimeUnit)
                {
                    case RateTimeTypes.FeetHour:
                    case RateTimeTypes.FeetSec:
                        return DistanceTypes.Feet;
                    case RateTimeTypes.KilometersHour:
                    case RateTimeTypes.KilometersSec:
                        return DistanceTypes.Kilometers;
                    case RateTimeTypes.MetersHour:
                    case RateTimeTypes.MetersSec:
                        return DistanceTypes.Meters;
                    case RateTimeTypes.MilesHour:
                    case RateTimeTypes.MilesSec:
                        return DistanceTypes.Miles;
                    case RateTimeTypes.NauticalMilesHour:
                    case RateTimeTypes.NauticalMilesSec:
                        return DistanceTypes.NauticalMile;
                    default:
                        return DistanceTypes.Meters;
                }
            }
            set
            {
                if (rateUnit == value)
                {
                    return;
                }

                rateUnit = value;

                UpdateDistance(TravelTimeInSeconds * TravelRateInSeconds, RateUnit);

                RaisePropertyChanged(() => RateUnit);
            }
        }

        RateTimeTypes rateTimeUnit = RateTimeTypes.MilesHour;
        public RateTimeTypes RateTimeUnit
        {
            get
            {
                return rateTimeUnit;
            }
            set
            {
                if (rateTimeUnit == value)
                {
                    return;
                }
                rateTimeUnit = value;

                UpdateDistance(TravelTimeInSeconds * TravelRateInSeconds, RateUnit);

                RaisePropertyChanged(() => RateTimeUnit);
            }
        }

        bool isDistanceCalcExpanded = false;
        public bool IsDistanceCalcExpanded
        {
            get { return isDistanceCalcExpanded; }
            set
            {
                isDistanceCalcExpanded = value;
                if (value == true)
                {
                    TravelRate = 0;
                    TravelTime = 0;
                    Distance = 0.0;
                    ResetFeedback();
                }
                else
                {
                    Reset(false);
                }

                ClearTempGraphics();
                if (HasPoint1)
                    AddGraphicToMap(Point1, new RgbColor() { Green = 255 } as IColor, true);

                RaisePropertyChanged(() => IsDistanceCalcExpanded);
            }
        }
        /// <summary>
        /// Distance is always the radius
        /// Update DistanceString for user
        /// Do nothing for Radius mode, double the radius for Diameter mode
        /// </summary>
        public override string DistanceString
        {
            get
            {
                if (CircleType == CircleFromTypes.Diameter)
                {
                    return (Distance * 2.0).ToString("G");
                }

                return base.DistanceString;
            }
            set
            {
                // lets avoid an infinite loop here
                if (string.Equals(base.DistanceString, value))
                    return;

                // divide the manual input by 2
                double d = 0.0;
                if (double.TryParse(value, out d))
                { 
                    if (CircleType == CircleFromTypes.Diameter)
                        d /= 2.0;

                    // Prevent creation of circle that is larger than the world
                    // Convert into kilometers taking units into account
                    if (d > 20075)
                    {
                        throw new ArgumentException(DistanceAndDirectionLibrary.Properties.Resources.AEInvalidInput);
                    }

                    Distance = d;

                    UpdateFeedbackWithGeoCircle();
                }
                else
                {
                    throw new ArgumentException(DistanceAndDirectionLibrary.Properties.Resources.AEInvalidInput);
                }
            }
        }

        #endregion

        #region Commands

        // when someone hits the enter key, create geodetic graphic
        internal override void OnEnterKeyCommand(object obj)
        {
            if (Distance == 0 || Point1 == null)
            {
                return;
            }
            base.OnEnterKeyCommand(obj);
        }

        #endregion

        #region override events

        public override DistanceTypes LineDistanceType
        {
            get
            {
                return base.LineDistanceType;
            }
            set
            {
                if (IsDistanceCalcExpanded)
                {
                    var before = base.LineDistanceType;
                    var temp = ConvertFromTo(before, value, Distance);
                    if (CircleType == CircleFromTypes.Diameter)
                        Distance = temp * 2.0;
                    else
                        Distance = temp;
                }

                base.LineDistanceType = value;

                UpdateFeedbackWithGeoCircle();
            }
        }

        internal override void OnNewMapPointEvent(object obj)
        {
            var point = obj as IPoint;
            if (point == null)
                return;

            if (IsDistanceCalcExpanded)
            {
                HasPoint1 = false;
            }

            base.OnNewMapPointEvent(obj);

            if (IsDistanceCalcExpanded)
            {
                UpdateDistance(TravelRateInSeconds * TravelTimeInSeconds, RateUnit);
            }
        }

        internal override void OnMouseMoveEvent(object obj)
        {
            if (!IsActiveTab)
                return;

            var point = obj as IPoint;

            if (point == null)
                return;

            // dynamically update start point if not set yet
            if (!HasPoint1)
            {
                Point1 = point;
            }
            else if (HasPoint1 && !HasPoint2 && !IsDistanceCalcExpanded)
            {
                Point2Formatted = string.Empty;
                // get distance from feedback
                var polyline = GetGeoPolylineFromPoints(Point1, point);
                UpdateDistance(polyline);
            }

            // update feedback
            if (HasPoint1 && !HasPoint2 && !IsDistanceCalcExpanded)
            {
                UpdateFeedbackWithGeoCircle();
            }
        }

        // Where a circle crosses dateline on one side, split and return as two separate parts, otherwise return original geometry
        private List<IGeometry> GetDatelineAwareGeometry(IGeometry construct)
        {
            IGeometry constructIGeom = construct as IGeometry;

            // Determine what case we are looking at
            Boolean beyondRightEdge = false;
            Boolean beyondLeftEdge = false;
            int pts = (constructIGeom as IPointCollection).PointCount;
            for (int i = 0; i < pts; i++)
            {
                double ptX = (constructIGeom as IPointCollection).get_Point(i).X;
                if (ptX > 180)
                {
                    beyondRightEdge = true;
                }
                if (ptX < -180)
                {
                    beyondLeftEdge = true;
                }
            }

            Point2 = (construct as IPolyline).ToPoint;
            var color = new RgbColorClass() as IColor;

            // Handle case
            if (!beyondLeftEdge && !beyondRightEdge)
            {
                // No overlap, or overlaps both, return original geometry
                return new List<IGeometry>() { construct as IGeometry };
            }
            else
            {
                // Either circle overlaps on the right hand side of the world, or on the left hand side of the world
                // shift points as appropriate
                IGeometry easternEdge = new Polyline() as IGeometry;
                IGeometry westernEdge = new Polyline() as IGeometry;

                if (beyondRightEdge)
                {
                    for (int i = 0; i < pts; i++)
                    {
                        IPoint pt = (constructIGeom as IPointCollection).get_Point(i);
                        {
                            if (pt.X > 180)
                            {
                                IPoint ptShifted = new Point();
                                ptShifted.Y = pt.Y;
                                // The actual shift of the point happens here
                                ptShifted.X = pt.X - 360;
                                (westernEdge as IPointCollection).AddPoint(ptShifted);
                            }
                            else
                            {
                                (easternEdge as IPointCollection).AddPoint(pt);
                            }
                        }
                    }
                }
                else
                {
                    for (int i = 0; i < pts; i++)
                    {
                        IPoint pt = (constructIGeom as IPointCollection).get_Point(i);
                        {
                            if (pt.X < -180)
                            {
                                IPoint ptShifted = new Point();
                                ptShifted.Y = pt.Y;
                                // The actual shift of the point happens here
                                ptShifted.X = pt.X + 360;
                                (easternEdge as IPointCollection).AddPoint(ptShifted);
                            }
                            else
                            {
                                (westernEdge as IPointCollection).AddPoint(pt);
                            }
                        }
                    }
                }

                // Ensure both portions of the divided circle meet the dateline, as they should
                int xVal = -180;
                foreach (var edge in new List<IPointCollection>(){westernEdge as IPointCollection, easternEdge as IPointCollection})
                {
                    double minY = 0, maxY = 0;
                    List<int> maxYIndices = new List<int>(), minYIndices = new List<int>();

                    // Find the min and max Y values
                    for (int k = 0; k < (edge).PointCount; k++)
                    {
                        double ptY = (edge).get_Point(k).Y;

                        if (ptY < minY)
                        {
                            minY = ptY;
                        }
                        if (ptY > maxY)
                        {
                            maxY = ptY;
                        }
                    }

                    // Just in case more than one point shares the same min y, or more than one point shares the same max y
                    // find the indices of the points with min and max y
                    for (int k = 0; k < (edge).PointCount; k++)
                    {
                        double ptY = (edge).get_Point(k).Y;

                        if (ptY == minY)
                        {
                            minYIndices.Add(k);
                        }
                        if (ptY == maxY)
                        {
                            maxYIndices.Add(k);
                        }
                    }

                    // Loop over the points with min and max y setting these points' x to 180
                    // We always want the highest and lowest points to be on the dateline,
                    // thus a forceful approach is valid
                    foreach (var yLimitList in new List<List<int>>(){minYIndices, maxYIndices})
                    {
                        foreach (int j in yLimitList)
                        {
                            IPoint updatedPoint = new Point();
                            updatedPoint.X = xVal;
                            updatedPoint.Y = (edge).get_Point(j).Y;
                            (edge).UpdatePoint(j, updatedPoint);
                        }
                    }
                    
                    // Update for second iteration which will use easternEdge
                    xVal = 180;
                }
 
                // Ensure our lines' start and end points are the same in order to end up with a closed polygon
                IPoint firstPointE = (easternEdge as IPointCollection).get_Point(0);
                IPoint lastPointE = (easternEdge as IPointCollection).get_Point((easternEdge as IPointCollection).PointCount - 1);
                if (firstPointE.X != lastPointE.X || firstPointE.Y != lastPointE.X)
                {
                    (easternEdge as IPointCollection).AddPoint(firstPointE);
                }
                IPoint firstPointW = (westernEdge as IPointCollection).get_Point(0);
                IPoint lastPointW = (westernEdge as IPointCollection).get_Point((westernEdge as IPointCollection).PointCount - 1);
                if (firstPointW.X != lastPointW.X || firstPointW.Y != lastPointW.X)
                {
                    (westernEdge as IPointCollection).AddPoint(firstPointW);
                }
                
                return new List<IGeometry>() { easternEdge as IGeometry, westernEdge as IGeometry };
            }
        }

        private void UpdateFeedbackWithGeoCircle()
        {
            if (Point1 == null || Distance <= 0.0)
                return;

            var construct = new Polyline() as IConstructGeodetic;
            if (construct != null)
            {
                ClearTempGraphics();
                AddGraphicToMap(Point1, new RgbColor() { Green = 255 } as IColor, true);

                try
                {
                    construct.ConstructGeodesicCircle(Point1, GetLinearUnit(), Distance, esriCurveDensifyMethod.esriCurveDensifyByAngle, 0.45);
                }
                catch(Exception ex)
                { 
                    // We get an exception here for some reason
                }

                List<IGeometry> constructGeom = GetDatelineAwareGeometry(construct as IGeometry);

                Point2 = (construct as IPolyline).ToPoint;
                var color = new RgbColorClass() as IColor;

                foreach(var geom in constructGeom)
                {
                    this.AddGraphicToMap(geom as IGeometry, color, true, rasterOpCode: esriRasterOpCode.esriROPNotXOrPen);
                }

            }
        }

        #endregion

        #region Private Functions

        /// <summary>
        /// Overrides TabBaseViewModel CreateMapElement
        /// </summary>
        internal override IGeometry CreateMapElement()
        {
            base.CreateMapElement();
            var geom = CreateCircle();
            Reset(false);

            return geom;
        }

        public override bool CanCreateElement
        {
            get
            {
                return (HasPoint1 && Distance != 0.0);
            }
        }

        internal override void Reset(bool toolReset)
        {
            base.Reset(toolReset);
            TravelTime = 0;
            TravelRate = 0;
        }

        /// <summary>
        /// Create a geodetic circle
        /// </summary>
        private IGeometry CreateCircle()
        {
            if (Point1 == null && Point2 == null)
            {
                return null;
            }

            var polyLine = new Polyline() as IPolyline;
            polyLine.SpatialReference = Point1.SpatialReference;
            var ptCol = polyLine as IPointCollection;
            ptCol.AddPoint(Point1);
            ptCol.AddPoint(Point2);

            UpdateDistance(polyLine as IGeometry);

            try
            {
                var construct = new Polyline() as IConstructGeodetic;
                if (construct != null)
                {
                    construct.ConstructGeodesicCircle(Point1, GetLinearUnit(), Distance, esriCurveDensifyMethod.esriCurveDensifyByAngle, 0.01);

                    List<IGeometry> constructGeom = GetDatelineAwareGeometry(construct as IGeometry);

                    foreach (var geom in constructGeom)
                    {
                        this.AddGraphicToMap(geom as IGeometry);

                        //Construct a polygon from geodesic polyline
                        var newPoly = this.PolylineToPolygon((IPolyline)geom);
                        if (newPoly != null)
                        {
                            //Get centroid of polygon
                            var area = newPoly as IArea;

                            // Ensure we use the correct distance, dependent on whether we are in Radius or Diameter mode
                            string distanceLabel;
                            if (circleType == CircleFromTypes.Radius)
                            {
                                distanceLabel = Math.Round(Distance, 2).ToString("N2");
                            }
                            else
                            {
                                distanceLabel = Math.Round((Distance * 2), 2).ToString("N2");
                            }

                            //Add text using centroid point
                            //Use circleType to ensure our label contains either Radius or Diameter dependent on mode
                            DistanceTypes dtVal = (DistanceTypes)LineDistanceType; //Get line distance type
                            this.AddTextToMap(area.Centroid, string.Format("{0}:{1} {2}",
                                circleType,
                                distanceLabel,
                                dtVal.ToString()));
                        }
                    }

                    Point2 = null;
                    HasPoint2 = false;
                    ResetFeedback();
                }

                return construct as IGeometry;
            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.Message);
                return null;
            }
        }
        #endregion
    }
}