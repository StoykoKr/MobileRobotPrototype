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
        private double resampleNoiseFactor = 12;  // here
        private double weightScale = 0.4;
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

        public List<string> logOfParticleChanges = new List<string>();
        private void MoveParticles(double forwardMove, double Angle, double theta, int startingIndex, int finalIndex) // Hmm the problem could be here? No idea if the particle.theta is the correct one or should be modified
        {

            if (startingIndex == 0)
            {

                logOfParticleChanges.Add($"Old way: theta = {RecalcDegree(Angle, 3)} | x = {Particles[0].X + (forwardMove) * Math.Cos(Particles[0].Theta * Math.PI / 180)} | y = {Particles[0].Y + (forwardMove) * Math.Sin(Particles[0].Theta * Math.PI / 180)}");


                double newTheta = NormalizeAngleDeg(NormalizeAngleDeg(Particles[0].Theta) + theta);
                //double thetaMid = NormalizeAngleDeg(360 - (Particles[i].Theta + Angle / 2.0));
                double thetaMid = NormalizeAngleDeg(NormalizeAngleDeg(Particles[0].Theta) + theta / 2.0);
                double thetaMidRad = (360 - thetaMid) * Math.PI / 180.0;

                logOfParticleChanges.Add($"Correct? way: theta = {newTheta} | x = {Particles[0].X + forwardMove * Math.Cos(thetaMidRad)} | y = {Particles[0].Y - forwardMove * Math.Sin(thetaMidRad)}");

            }

            // changes at 7/5/2025  angle is now deltaTheta
            for (int i = startingIndex; i <= finalIndex; i++)
            {
                //Particles[i].X += (forwardMove) * Math.Cos((Particles[i].Theta + absoluteAngle / 2) * Math.PI / 180);
                //Particles[i].Y -= (forwardMove) * Math.Sin((Particles[i].Theta + absoluteAngle / 2) * Math.PI / 180);
                //Particles[i].Theta += absoluteAngle;
                Particles[i].Theta = RecalcDegree(Angle, 3);
                Particles[i].X += (forwardMove) * Math.Cos(Particles[i].Theta * Math.PI / 180);
                Particles[i].Y += (forwardMove) * Math.Sin(Particles[i].Theta * Math.PI / 180);

                //double newTheta = NormalizeAngleDeg(NormalizeAngleDeg(Particles[i].Theta) + Angle);
                ////double thetaMid = NormalizeAngleDeg(360 - (Particles[i].Theta + Angle / 2.0));
                //double thetaMid = NormalizeAngleDeg(NormalizeAngleDeg(Particles[i].Theta) + Angle / 2.0);
                //double thetaMidRad = (360 - thetaMid) * Math.PI / 180.0;

                //// Update position (adjust Y for inverted axis)
                //Particles[i].X += forwardMove * Math.Cos(thetaMidRad);
                //Particles[i].Y -= forwardMove * Math.Sin(thetaMidRad);

                //Particles[i].Theta = newTheta;
            }
        }
        private double NormalizeAngleDeg(double angle)
        {
            while (angle < 0) angle += 360;
            while (angle >= 360) angle -= 360;
            return angle;
        }
        public async Task StartTasksToMoveParticles(double move, double dir, double theta)
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
                var usedDir = RecalcDegree(dir, 2);
                Task task = new Task(() => MoveParticles(move, dir, theta, starIndex, endIndex));
                task.Start();
                tasks.Add(task);
            }
            await Task.WhenAll(tasks);
        }

        public ConcurrentQueue<string> listOfbigWeights = new ConcurrentQueue<string>();

        public ConcurrentQueue<string> listOfSmallbigWeights = new ConcurrentQueue<string>();
        public void UpdateWeights(double[] observedData, int startingIndex, int finalIndex, ConcurrentQueue<double> doubles)
        {
            double total = 0;

            if(startingIndex == 0)
            {
                logOfParticleChanges.Add($"Likelihood of particle is {CalculateLikelihood(Particles[0], observedData)}");

            }


            for (int i = startingIndex; i <= finalIndex; i++)
            {
                double likelihood = CalculateLikelihood(Particles[i], observedData);
                if( likelihood > 10 )
                {
                    listOfbigWeights.Enqueue($"Particle with values {Particles[i].X}|{Particles[i].Y}|{Particles[i].Theta} has likeHood of {likelihood}| with observed data of {observedData[0]}|{observedData[1]}|{observedData[2]}");
                }
                if( likelihood > 1 && likelihood< 10)
                {
                    listOfSmallbigWeights.Enqueue($"Particle with values {Particles[i].X}|{Particles[i].Y}|{Particles[i].Theta} has likeHood of {likelihood}| with observed data of {observedData[0]}|{observedData[1]}|{observedData[2]}");
                }

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
        public async Task StartTasksToUpdateWeightsGPT(double[] observedData)
        {
            double[] tempWeights = new double[Particles.Count];

            // Parallelize likelihood computation with a thread-safe pattern
            await Task.Run(() =>
            {
                Parallel.For(0, Particles.Count, i =>
                {
                    double likelihood = CalculateLikelihood(Particles[i], observedData);

                    if (double.IsNaN(likelihood) || likelihood <= 0 || likelihood < 1e-12)
                    {
                        likelihood = 1e-12;
                    }

                    tempWeights[i] = likelihood;
                });
            });

            // Normalize weights
            double totalWeight = tempWeights.Sum();
            if (totalWeight <= 0)
            {
                throw new Exception("Total weight is non-positive after likelihood calculation.");
            }

            double normalizationFactor = 1.0 / totalWeight;

            for (int i = 0; i < Particles.Count; i++)
            {
                Particles[i].Weight = tempWeights[i] * normalizationFactor;
            }

            // Optionally sanity check normalization
            double sanitySum = Particles.Sum(p => p.Weight);
            if (Math.Abs(sanitySum - 1.0) > 1e-3)
            {
                throw new Exception($"Normalized weights don't sum to 1.0 (got {sanitySum})");
            }

            // Update estimated position safely
            lock (_estimatedPosLock)
            {
                lastEstimatedPos = EstimatePosition();
            }

            totalWeightPublic = totalWeight;
        }

        public async Task StartTasksToUpdateWeights(double[] observedData) // Spread the work on multiple tasks. After all are done then recalc weight so the total is 1
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
                Task task = new Task(() => UpdateWeights(observedData, starIndex, endIndex, weights));
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
                if (Particles[i].Weight > 10)
                {
                    listOfbigWeights.Enqueue($"Particle with values {Particles[i].X}|{Particles[i].Y}|{Particles[i].Theta} has WEIGHT of {Particles[i].Weight}| with observed data of {observedData[0]}|{observedData[1]}|{observedData[2]} and NormalizationFactor of {normalizationFactor} caused by totalWeight of 1/ {totalWeight}");
                }
                if (Particles[i].Weight > 1 && Particles[i].Weight < 10)
                {
                    listOfSmallbigWeights.Enqueue($"Particle with values {Particles[i].X}|{Particles[i].Y}|{Particles[i].Theta} has WEIGHT of {Particles[i].Weight}| with observed data of {observedData[0]}|{observedData[1]}|{observedData[2]} and NormalizationFactor of {normalizationFactor} caused by totalWeight of 1/ {totalWeight}");
                }

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

        public double GetDynamicSigma(double expectedValue, double baseFactor = 0.15)  // Still not sure if this is even helpfull, but likely it isn't. Not sure exactly how important the sigma is for Gaussian
        {
            return Math.Clamp(baseFactor * expectedValue, 4, 100);//Math.Max(baseFactor * expectedValue, 4);

        }
        public double CalculateLikelihood(Particle particle, double[] observedData)
        {
            double usedTheta = /*360 -*/particle.Theta;// + 90;  // No idea how exactly it should be here.. likely one of the major errors
                                                       //   double usedTheta = NormalizeAngleDeg(particle.Theta);
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




            double frontDiff = observedData[0] > 0 ? Math.Abs(observedData[0] - predictedFront) : -1;
            double leftDiff = observedData[1] > 0 ? Math.Abs( observedData[1] - predictedLeft) : -1;
            double rightDiff = observedData[2] > 0 ? Math.Abs( observedData[2] - predictedRight) : -1;





            double threshold = 310;
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
                    continue;
                }
                else if (predicted > threshold || actual > threshold)
                {
                    totalLogLikelihood += Math.Log(minProbability);
                }
                else
                {
                    if (actual != 0)
                    {
                        double diff = Math.Abs(actual - predicted);
                            double sigmaDynamic = GetDynamicSigma(diff);
                            double likelihood = GaussianProbability(diff, sigmaDynamic);
                            totalLogLikelihood += Math.Log(likelihood);
                        
                    }
                }
            }
            double frontSigma = GetDynamicSigma(frontDiff);
            double leftSigma = GetDynamicSigma(leftDiff);
            double rightSigma = GetDynamicSigma(rightDiff);

            double maxLikelihood = (frontDiff >= 0 ? GaussianProbability(0, frontSigma) : 1)
                    * (leftDiff >= 0 ? GaussianProbability(0, leftSigma) : 1)
                    * (rightDiff >= 0 ? GaussianProbability(0, rightSigma) : 1);

            var toReturn =  Math.Exp(totalLogLikelihood) / maxLikelihood;

            comparing.Add($"predicteed: {predictedLeft} | {predictedFront} | {predictedRight} with theta {usedTheta} but it has -90 right now. This gives differences of {leftDiff} | {frontDiff} | {rightDiff}  And final likelihood of {toReturn}"); // some logging
            return toReturn;
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
            return RaycastCone(shiftedX, shiftedY, angl, 335, GlobalConstants.SensorDispersion, map);
        }

        public static double RaycastCone(double startX, double startY, double angle, double maxRange, double dispersion, Grid map)  // tried something. not good yet
        {
            double stepSize = 7;
            double distance = 0;

            double tempAngle;

            double radians = 0;
            int targetX = 0;
            int targetY = 0;
            while (distance < maxRange)
            {
                for (int i = 0; i <= dispersion; i += 3)
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
        
        public List<(double,double, double, double,double,double)> logOfResamplingNoiseAndImportanceAndCurrentWeightAndMaxWeightForThisCalcAndParticleXChangeAsWellAsYChange = new List<(double,double, double, double,double,double)>();

        public void Resample(bool forceTheta, double thetaToBeForced)
        {
            List<Particle> newParticles = new List<Particle>();
            double[] cumulativeWeights = new double[allParticleCount];


            var snapshot = Particles.ToList();
            double weightSum = snapshot.Sum(p => p.Weight);
            int numberToSample = snapshot.Count();
            double step = weightSum / numberToSample;
            double start = rand.NextDouble() * step;

                double maxWeight = snapshot.Max(p => p.Weight);
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

                double relativeImportance = snapshot[selectedIndex].Weight / maxWeight;
                double adaptiveNoise = resampleNoiseFactor * (1.1 - relativeImportance);

                var xChange = (rand.NextDouble() * 2 - 1) * adaptiveNoise;
                var yChange = (rand.NextDouble() * 2 - 1) * adaptiveNoise;
                resampledParticle.X += xChange;
                resampledParticle.Y += yChange;
                logOfResamplingNoiseAndImportanceAndCurrentWeightAndMaxWeightForThisCalcAndParticleXChangeAsWellAsYChange.Add(new (adaptiveNoise,relativeImportance, snapshot[selectedIndex].Weight, maxWeight, xChange, yChange));

                if (forceTheta)
                {
                    resampledParticle.Theta = thetaToBeForced;

                }
                resampledParticle.Theta += (rand.NextDouble() * 2 - 1) * adaptiveNoise * weightScale;

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
