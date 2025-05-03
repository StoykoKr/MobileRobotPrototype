using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Drawing.Text;
using System.IO;
using System.Linq;
using System.Reflection.Metadata;
using System.Text;
using System.Threading.Tasks;
using static System.Formats.Asn1.AsnWriter;
using static System.Windows.Forms.VisualStyles.VisualStyleElement.Rebar;

namespace RobotAppControl
{
    public class MonteCarloLocal
    {

        public bool isStarted { private set; get; }
        public double currentEstimateWeight;
        public Random rand = new Random();
        private Grid MapMap = null;
        public int allParticleCount { private set; get; }
        public int numberOfTasksToRunOn { private set; get; }
        private double resampleNoiseFactor = 10;
        private double weightScale = 1;
        public double totalWeightPublic = 666;
        private (double, double, double) lastEstimatedPos = (700, 700, 120);
        public List<Particle> Particles { private set; get; }
        private readonly object _particlesLock = new object();
        private readonly object _estimatedPosLock = new object();
        public List<String> comparing = new List<string>();
        public MonteCarloLocal(int particleCount, int CenterX, int CenterY, int range, int tastksCount, Grid map)
        {
            MapMap = map;
            isStarted = true;
            allParticleCount = particleCount;
            numberOfTasksToRunOn = tastksCount;
            Particles = InitializeParticles(particleCount, CenterX - range, CenterX + range, CenterY - range, CenterY + range);
            currentEstimateWeight = 0;
        }
        private List<Particle> InitializeParticles(int particleCount, int xMin, int xMax, int yMin, int yMax)  // Init the particles in the specified area
        {
            var particles = new List<Particle>();
            Random rand = new Random();

            for (int i = 0; i < particleCount; i++)
            {
                double x = rand.NextDouble() * (xMax - xMin) + xMin;
                double y = rand.NextDouble() * (yMax - yMin) + yMin;
                double theta = rand.NextDouble() * 360;
                particles.Add(new Particle(x, y, theta));
            }

            return particles;
        }
        private double RecalcDegree(double angle, double range) // A bad way to add some randomness to the direction of the particle
        {
            Random rand = new Random();
            double newAngle = angle;
            if (rand.NextDouble() > 0.5)
            {
                newAngle += range * rand.NextDouble();
                if (newAngle > 360)
                {
                    newAngle -= 360;
                }
            }
            else if (rand.NextDouble() < 0.5)
            {
                newAngle -= range * rand.NextDouble();
                if (newAngle < 0)
                {
                    newAngle += 360;
                }
            }
            else
            {
               
            }
            return newAngle;
        }

        private void MoveParticles(double forwardMove, double Angle, int startingIndex, int finalIndex) // Hmm the problem could be here? No idea if the particle.theta is the correct one or should be modified
        {
            for (int i = startingIndex; i <= finalIndex; i++)
            {
                Particles[i].Theta = RecalcDegree(Angle, 3);
                Particles[i].X += (forwardMove) * Math.Cos(Particles[i].Theta * Math.PI / 180);
                Particles[i].Y += (forwardMove) * Math.Sin(Particles[i].Theta * Math.PI / 180);
            }
        }
        public async Task StartTasksToMoveParticles(float move, float dir)
        {
            int remainingIndexes = allParticleCount;
            List<Task> tasks = new List<Task>();
            for (int j = 1; j <= numberOfTasksToRunOn; j++)
            {
                int starIndex = 0;
                int endIndex = 0;
                endIndex = remainingIndexes - 1;
                remainingIndexes = remainingIndexes - allParticleCount / numberOfTasksToRunOn;
                if (j == numberOfTasksToRunOn)
                {
                    remainingIndexes -= allParticleCount % numberOfTasksToRunOn;
                }
                starIndex = remainingIndexes;
                Task task = new Task(() => MoveParticles(move, dir, starIndex, endIndex));
                task.Start();
                tasks.Add(task);
            }
            await Task.WhenAll(tasks);
        }
        public void UpdateWeights(double[] observedData, double sigma, int startingIndex, int finalIndex, ConcurrentQueue<double> doubles)
        {
            double total = 0;
            for (int i = startingIndex; i <= finalIndex; i++)
            {
                double likelihood = CalculateLikelihood(Particles[i], observedData, sigma);
                if (likelihood <= 0)
                {
                    likelihood = double.Epsilon;
                }
                if (double.IsNaN(likelihood) || likelihood < 1e-12) // honestly don't even remember what madness drove me to these values.
                { 
                    likelihood = 1e-12;
                }
                Particles[i].Weight = likelihood;
                total += Particles[i].Weight;
            }
            if (total == 0)
            {
                total = double.Epsilon;
            }
            doubles.Enqueue(total);
        }
        public async Task StartTasksToUpdateWeights(double[] observedData, double sigma) // Spread the work on multiple tasks. After all are done then recalc weight so the total is 1
        {
            ConcurrentQueue<double> weights = new ConcurrentQueue<double>();
            int remainingIndexes = allParticleCount;
            List<Task> tasks = new List<Task>();

            for (int j = 1; j <= numberOfTasksToRunOn; j++)
            {
                int endIndex = remainingIndexes - 1;
                remainingIndexes = remainingIndexes - allParticleCount / numberOfTasksToRunOn;
                if (j == numberOfTasksToRunOn)
                {
                    remainingIndexes -= allParticleCount % numberOfTasksToRunOn;
                }
                int starIndex = remainingIndexes;
                Task task = new Task(() => UpdateWeights(observedData, sigma, starIndex, endIndex, weights));
                tasks.Add(task);
                task.Start();
            }
            await Task.WhenAll(tasks);

            double totalWeight = 0;
            while (weights.TryDequeue(out double result))
            {
                totalWeight += result;
            }
            if (totalWeight <= 0)
            {
                throw new Exception("wtf");
            }
            double normalizationFactor = 1.0 / totalWeight;
            for (int i = 0; i < Particles.Count; i++)
            {
                Particles[i].Weight *= normalizationFactor;


            }
            lock (_estimatedPosLock)
            {
                lastEstimatedPos = EstimatePosition();
            }

            totalWeightPublic = totalWeight;

        }

        public void DrawStartingPosForSensors(CustomBitmap bitmap) // The idea here was to visualize the starting points of the particle sensors and their line of sight. Though due to my poor coding we are calculating the line of sight as if it is a laser and not the cone which it is.
        {

            foreach (var item in Particles)
            {
                double thetaused = item.Theta - 90;
                if (thetaused > 360) { thetaused -= 360; }
                if (thetaused < 0) { thetaused += 360; }
                double headingRadians = thetaused * Math.PI / 180.0;


                double offsetX = GlobalConstants.MidSensorOffsets.Item1 * Math.Cos(headingRadians) - GlobalConstants.MidSensorOffsets.Item2 * Math.Sin(headingRadians);
                double offsetY = GlobalConstants.MidSensorOffsets.Item1 * Math.Sin(headingRadians) + GlobalConstants.MidSensorOffsets.Item2 * Math.Cos(headingRadians);

                double shiftedX = item.X + offsetX;
                double shiftedY = item.Y + offsetY;
                double angl = item.Theta + GlobalConstants.DegreeOffsetMid;

                if (angl < 0)
                    angl += 360;
                if (angl >= 360)
                    angl -= 360;

                   DrawRaycast(shiftedX, shiftedY, angl, 335, MapMap, bitmap, Color.MediumBlue);
                // bitmap.SetPixel((int)shiftedX,(int) shiftedY, Color.MediumBlue);

                offsetX = GlobalConstants.LeftSensorOffsets.Item1 * Math.Cos(headingRadians) - GlobalConstants.LeftSensorOffsets.Item2 * Math.Sin(headingRadians);
                offsetY = GlobalConstants.LeftSensorOffsets.Item1 * Math.Sin(headingRadians) + GlobalConstants.LeftSensorOffsets.Item2 * Math.Cos(headingRadians);

                shiftedX = item.X + offsetX;
                shiftedY = item.Y + offsetY;
                angl = item.Theta + GlobalConstants.DegreeOffsetLeft;

                if (angl < 0)
                    angl += 360;
                if (angl >= 360)
                    angl -= 360;


                 DrawRaycast(shiftedX, shiftedY, angl, 335, MapMap, bitmap, Color.Lavender);
                //   bitmap.SetPixel((int)shiftedX, (int)shiftedY, Color.Lavender);

                offsetX = GlobalConstants.RightSensorOffsets.Item1 * Math.Cos(headingRadians) - GlobalConstants.RightSensorOffsets.Item2 * Math.Sin(headingRadians);
                offsetY = GlobalConstants.RightSensorOffsets.Item1 * Math.Sin(headingRadians) + GlobalConstants.RightSensorOffsets.Item2 * Math.Cos(headingRadians);

                shiftedX = item.X + offsetX;
                shiftedY = item.Y + offsetY;
                angl = item.Theta + GlobalConstants.DegreeOffsetRight;

                if (angl < 0)
                    angl += 360;
                if (angl >= 360)
                    angl -= 360;

                  DrawRaycast(shiftedX, shiftedY, angl, 335, MapMap, bitmap, Color.OrangeRed);
                //  bitmap.SetPixel((int)shiftedX, (int)shiftedY, Color.OrangeRed);

            }
        }

        public double GetDynamicSigma(double expectedValue, double baseFactor = 0.01)  // Still not sure if this is even helpfull, but likely it isn't. Not sure exactly how important the sigma is for Gaussian
        {
            return Math.Max(baseFactor * expectedValue, 5);
        }
        public double CalculateLikelihood(Particle particle, double[] observedData, double sigma)
        {
            double usedTheta = /*360 -*/particle.Theta;// + 90;  // No idea how exactly it should be here.. likely one of the major errors
            if (usedTheta < 0)
            {
                usedTheta += 360;
            }
            if (usedTheta > 360)
            {
                usedTheta -= 360;
            }

            double predictedFront = GetPredictedDistance(particle.X, particle.Y, usedTheta, GlobalConstants.DegreeOffsetMid, GlobalConstants.MidDegrees, GlobalConstants.MidSensorOffsets, MapMap);
            double predictedLeft = GetPredictedDistance(particle.X, particle.Y, usedTheta, GlobalConstants.DegreeOffsetLeft, GlobalConstants.LeftDegrees, GlobalConstants.LeftSensorOffsets, MapMap);
            double predictedRight = GetPredictedDistance(particle.X, particle.Y, usedTheta, GlobalConstants.DegreeOffsetRight, GlobalConstants.RightDegrees, GlobalConstants.RightSensorOffsets, MapMap);


            comparing.Add($"predicteed: {predictedLeft} | {predictedFront} | {predictedRight}"); // some logging


            double frontDiff = observedData[0] - predictedFront;
            double leftDiff = observedData[1] - predictedLeft;
            double rightDiff = observedData[2] - predictedRight;

            double threshold = 320;
            double minProbability = 1e-5;

            double totalLogLikelihood = 0;

            (double predicted, double actual)[] sensors = {
        (predictedFront, observedData[0]),
        (predictedLeft, observedData[1]),
        (predictedRight, observedData[2])
            };

            foreach (var (predicted, actual) in sensors)
            {
                if (predicted > threshold && actual > threshold)
                {
                    // Ignore this sensor
                    continue;
                }
                else if (predicted > threshold || actual > threshold)
                {
                    // One is above threshold → very unlikely
                    totalLogLikelihood += Math.Log(minProbability);
                }
                else
                {
                    double diff = actual - predicted;
                    double sigmaDynamic = GetDynamicSigma(diff);
                    double likelihood = GaussianProbability(diff, sigmaDynamic);
                    totalLogLikelihood += Math.Log(likelihood);
                }
            }
            double frontSigma = GetDynamicSigma(frontDiff);
            double leftSigma = GetDynamicSigma(leftDiff);
            double rightSigma = GetDynamicSigma(rightDiff);

            double maxLikelihood = GaussianProbability(0, frontSigma)
                    * GaussianProbability(0, leftSigma)
                    * GaussianProbability(0, rightSigma);


            return Math.Exp(totalLogLikelihood) / maxLikelihood;

        }
        public double GaussianProbability(double difference, double sigma)
        {
            double exponent = -Math.Pow(difference, 2) / (2 * Math.Pow(sigma, 2));
            double coefficient = 1.0 / (sigma * Math.Sqrt(2 * Math.PI));
            double probability = coefficient * Math.Exp(exponent);

            return Math.Max(probability, 1e-10);
        }
        public double GetPredictedDistance(double x, double y, double theta, double sensorDirectionLookingAt, double startingPointDegreeOffsets, (double, double) startingPointOffsets, Grid map)
        {

            double thetaused = theta - 90;   // it was somewhat shifted in a bad direction visually so I tried to shift it back
            if (thetaused > 360) { thetaused -= 360; }
            if (thetaused < 0) { thetaused += 360; }
            double headingRadians = thetaused * Math.PI / 180.0;

            double offsetX = startingPointOffsets.Item1 * Math.Cos(headingRadians) - startingPointOffsets.Item2 * Math.Sin(headingRadians);
            double offsetY = startingPointOffsets.Item1 * Math.Sin(headingRadians) + startingPointOffsets.Item2 * Math.Cos(headingRadians);

            double shiftedX = x + offsetX;
            double shiftedY = y + offsetY;

            double angl = thetaused + sensorDirectionLookingAt;

            if (angl < 0)
                angl += 360;
            if (angl >= 360)
                angl -= 360;

           // return Raycast(shiftedX, shiftedY, angl, 335, map);
            return RaycastCone(shiftedX, shiftedY, angl, 335,GlobalConstants.SensorDispersion, map);
        }

        public static double RaycastCone(double startX, double startY, double angle, double maxRange, double dispersion, Grid map)  // tried something. not good yet
        {
            double stepSize = 7;
            double distance = 0;

            double tempAngle;

            double radians = 0;
            int targetX =0;
            int targetY =0;
            while (distance < maxRange)
            {
                for (int i = 0; i <= dispersion; i+=6)
                {

                    tempAngle = angle - dispersion / 2 + i;
                    if (tempAngle < 0)
                    {
                        tempAngle += 360;
                    }
                    if (tempAngle > 360)
                    {
                        tempAngle -= 360;
                    }
                    radians = tempAngle * Math.PI / 180.0;
                     targetX = (int)Math.Round(startX + Math.Abs(distance) * Math.Cos(radians));
                     targetY = (int)Math.Round(startY + Math.Abs(distance) * Math.Sin(radians));

                    if (!map.IsWalkable(targetX, targetY) == true)
                    {
                        return distance;
                    }

                }
                distance += stepSize;
            }
            return maxRange;
        }
        public void Resample(bool forceTheta, double thetaToBeForced)
        {
            List<Particle> newParticles = new List<Particle>();
            double[] cumulativeWeights = new double[allParticleCount];


            var snapshot = Particles.ToList();
            double weightSum = snapshot.Sum(p => p.Weight);
            int numberToSample = snapshot.Count();
            double step = weightSum / numberToSample;
            double start = rand.NextDouble() * step;

            double cumulative = 0.0;
            int index = 0;

            for (int i = 0; i < numberToSample; i++)
            {
                double threshold = start + i * step;

                while (cumulative < threshold && index < snapshot.Count)
                {
                    cumulative += snapshot[index].Weight;
                    index++;
                }

                int selectedIndex = Math.Min(index - 1, snapshot.Count - 1);

                Particle resampledParticle = snapshot[selectedIndex].Clone();



                // Used to use another way of making less likely particles change more aggresively but after changing some code the way was not longer usable
                weightScale = 0.22;


                resampledParticle.X += (rand.NextDouble() * 2 - 1) * resampleNoiseFactor * weightScale;
                resampledParticle.Y += (rand.NextDouble() * 2 - 1) * resampleNoiseFactor * weightScale;
                if (forceTheta)
                {
                    resampledParticle.Theta = thetaToBeForced;

                }
                resampledParticle.Theta += (rand.NextDouble() * 2 - 1) * (resampleNoiseFactor / 2) * weightScale;

                newParticles.Add(resampledParticle);
            }
            Particles = newParticles;
        }

        public (double X, double Y, double Theta) GetEstimatedPos()
        {
            lock (_estimatedPosLock)
            {
                return lastEstimatedPos;
            }
        }
        public (double X, double Y, double Theta) EstimatePosition()  //Selects a third of the points and averages them for an estimation
        {
            List<Particle> snapshot;

            lock (_particlesLock)  //Think there was some bad things with threading thus the lock
            {
                snapshot = Particles.ToList();
            }

            double weightSum = snapshot.Sum(p => p.Weight);
            int numberToSample = snapshot.Count() / 3;
            double step = weightSum / numberToSample;
            double start = rand.NextDouble() * step;

            double cumulative = 0.0;
            int index = 0;

            double x = 0.0, y = 0.0;
            double sumSin = 0.0, sumCos = 0.0;
            for (int i = 0; i < numberToSample; i++)
            {
                double threshold = start + i * step;

                while (cumulative < threshold && index < snapshot.Count)
                {
                    cumulative += snapshot[index].Weight;
                    index++;
                }

                int selectedIndex = Math.Min(index - 1, snapshot.Count - 1);

                x += snapshot[i].X;
                y += snapshot[i].Y;

                sumCos += Math.Cos(snapshot[i].Theta * Math.PI / 180.0);
                sumSin += Math.Sin(snapshot[i].Theta * Math.PI / 180.0);

            }

            double avgX = x / numberToSample;
            double avgY = y / numberToSample;
            double avgTheta = Math.Atan2(sumSin, sumCos) * 180.0 / Math.PI;

            return (avgX, avgY, avgTheta);
        }

        public static double Raycast(double startX, double startY, double angle, double maxRange, Grid map) // This is a simple but stupid way but it was working so far so it was not replaced.
        {

            double stepSize = 2;
            double distance = 0;


            double dirX = Math.Cos(angle * Math.PI / 180);
            double dirY = Math.Sin(angle * Math.PI / 180);

            double currentX = startX;
            double currentY = startY;


            while (distance < maxRange)
            {
                currentX += dirX * stepSize;
                currentY -= dirY * stepSize;
                distance += stepSize;

                if (!map.IsWalkable((int)currentX, (int)currentY) == true)
                {
                    return distance;
                }
            }
            return maxRange;
        }
        public static double DrawRaycast(double startX, double startY, double angle, double maxRange, Grid map, CustomBitmap bitmap, Color color) //Same as above but also draws the dots
        {

            double stepSize = 2;
            double distance = 0;


            double dirX = Math.Cos(angle * Math.PI / 180);
            double dirY = Math.Sin(angle * Math.PI / 180);

            double currentX = startX;
            double currentY = startY;


            while (distance < maxRange)
            {
                currentX += dirX * stepSize;
                currentY += dirY * stepSize;
                distance += stepSize;
                bitmap.SetPixel((int)currentX, (int)currentY, color);
                if (!map.IsWalkable((int)currentX, (int)currentY) == true)
                {
                    return distance;
                }
            }
            return maxRange;
        }

    }
}
