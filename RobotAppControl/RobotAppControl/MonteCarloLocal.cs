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
        public double currentEstimateWeight;

        private readonly Grid MapMap = null;
        public int allParticleCount { private set; get; }
        public int numberOfTasksToRunOn { private set; get; }
        private double resampleNoiseFactor = 12;  // here
        private double weightScale = 0.4;
        public double totalWeightPublic = 666;
        private (double, double, double) lastEstimatedPos = (700, 700, 120);
        public List<Particle> Particles { private set; get; }
        private readonly object _particlesLock = new object();
        private readonly object _estimatedPosLock = new object();

        public bool firstMove = true;
        public ConcurrentQueue<string> comparing = new ConcurrentQueue<string>();
        public List<string> logOfTheBS = new List<string>();
        public List<string> logOfParticleChanges = new List<string>();
        public ConcurrentQueue<string> listOfbigWeights = new ConcurrentQueue<string>();
        public ConcurrentQueue<string> listOfSmallbigWeights = new ConcurrentQueue<string>();

        public List<(double, double, double, double, double, double)> logOfResamplingNoiseAndImportanceAndCurrentWeightAndMaxWeightForThisCalcAndParticleXChangeAsWellAsYChange = new List<(double, double, double, double, double, double)>();
        private double baseJitterXY = 2.5;
        private double baseJitterTheta = 3.5;
        public List<(double, double)> logOfEss = new List<(double, double)>();
        public MonteCarloLocal(int particleCount, int CenterX, int CenterY, int range, int tastksCount, Grid map)
        {
            MapMap = map;
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

            double newAngle = angle;
            if (Random.Shared.NextDouble() > 0.5)
            {
                newAngle += range * Random.Shared.NextDouble();
                if (newAngle > 360)
                {
                    newAngle -= 360;
                }
            }
            else if (Random.Shared.NextDouble() < 0.5)
            {
                newAngle -= range * Random.Shared.NextDouble();
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

        private void MoveParticles(double forwardMove, double Angle, double theta, int startingIndex, int finalIndex)
        {

            for (int i = startingIndex; i <= finalIndex; i++)
            {

                //Particles[i].Theta = RecalcDegree(Angle, 3);
                //Particles[i].X += (forwardMove) * Math.Cos(Particles[i].Theta * Math.PI / 180);
                //Particles[i].Y += (forwardMove) * Math.Sin(Particles[i].Theta * Math.PI / 180);




                double newTheta;
                double thetaMid;
                double thetaMidRad;
                if (firstMove)
                {
                    newTheta = RecalcDegree(Angle, 2);
                    thetaMid = RecalcDegree(Angle, 2);
                    thetaMidRad = thetaMid * Math.PI / 180.0;

                }
                else
                {
                    newTheta = NormalizeAngleDeg(NormalizeAngleDeg(Particles[i].Theta) + theta);
                    thetaMid = NormalizeAngleDeg(NormalizeAngleDeg(Particles[i].Theta) + theta / 2.0);
                    thetaMidRad =  thetaMid * Math.PI / 180.0;
                }


                Particles[i].X += forwardMove * Math.Cos(thetaMidRad);
                Particles[i].Y += forwardMove * Math.Sin(thetaMidRad);
                Particles[i].Theta = newTheta;


            }
        }
      
        public async Task StartTasksToMoveParticles(double move, double dir, double theta)
        {
            double localsavedTheta = theta;
            double localsavedDir = dir;
            double localsavedMove = move;
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
                // var usedDir = RecalcDegree(dir, 2
                Task task = new Task(() => MoveParticles(localsavedMove, localsavedDir, localsavedTheta, starIndex, endIndex));
                task.Start();
                tasks.Add(task);
            }
            await Task.WhenAll(tasks);
        }
        public async Task StartTasksToUpdateWeightsGPT(double[] observedData)
        {
            double[] tempWeights = new double[Particles.Count];
            var particlesSnapshot = Particles.ToArray();

            await Task.Run(() =>
            {
                Parallel.For(0, particlesSnapshot.Length, i =>
                {
                    double likelihood = CalculateLikelihood(particlesSnapshot[i], observedData);

                    if (double.IsNaN(likelihood) || likelihood <= 0 || likelihood < 1e-12)
                    {
                        likelihood = 1e-12;
                    }

                    tempWeights[i] = likelihood;
                });
            });

           
            double totalWeight = tempWeights.Sum();
            if (totalWeight <= 0)
            {
                //  throw new Exception("Total weight is non-positive after likelihood calculation.");
            }

            double normalizationFactor = 1.0 / totalWeight;

            for (int i = 0; i < particlesSnapshot.Length; i++)
            {
                Particles[i].Weight = tempWeights[i] * normalizationFactor;
            }

            // Optionally sanity check normalization
            double sanitySum = Particles.Sum(p => p.Weight);
            if (Math.Abs(sanitySum - 1.0) > 1e-3)
            {

                logOfTheBS.Add($"Sanity fail. Sum is  ->  {sanitySum}");

                // throw new Exception($"Normalized weights don't sum to 1.0 (got {sanitySum})");
            }

            // Update estimated position safely
            lock (_estimatedPosLock)
            {
                lastEstimatedPos = EstimatePosition();
            }

            totalWeightPublic = totalWeight;
        }

        public double GetDynamicSigma(double expectedValue, double baseFactor = 0.15)  // Still not sure if this is even helpfull, but likely it isn't. Not sure exactly how important the sigma is for Gaussian
        {
            return Math.Clamp(baseFactor * expectedValue, 4, 100);//Math.Max(baseFactor * expectedValue, 4);

        }
        public double CalculateLikelihood(Particle particle, double[] observedData)
        {
            double usedTheta = particle.Theta;
                                                     
            if (usedTheta < 0)
            {
                usedTheta += 360;
            }
            if (usedTheta > 360)
            {
                usedTheta -= 360;
            }

            double predictedFront = GetPredictedDistance(particle.X, particle.Y, usedTheta, GlobalConstants.DegreeOffsetMid, GlobalConstants.MidDegrees, GlobalConstants.MidSensorOffsets, MapMap,false,null);
            double predictedLeft = GetPredictedDistance(particle.X, particle.Y, usedTheta, GlobalConstants.DegreeOffsetLeft, GlobalConstants.LeftDegrees, GlobalConstants.LeftSensorOffsets, MapMap, false, null);
            double predictedRight = GetPredictedDistance(particle.X, particle.Y, usedTheta, GlobalConstants.DegreeOffsetRight, GlobalConstants.RightDegrees, GlobalConstants.RightSensorOffsets, MapMap, false, null);


            double frontDiff = observedData[0] > 0 ? Math.Abs(observedData[0] - predictedFront) : -1;
            double leftDiff = observedData[1] > 0 ? Math.Abs(observedData[1] - predictedLeft) : -1;
            double rightDiff = observedData[2] > 0 ? Math.Abs(observedData[2] - predictedRight) : -1;

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

            var toReturn = Math.Exp(totalLogLikelihood) / maxLikelihood;

            //comparing.Enqueue($"predicteed: {predictedLeft} | {predictedFront} | {predictedRight} with theta {usedTheta}. Real distances being {observedData[1]} | {observedData[0]} | {observedData[2]} This gives differences of {leftDiff} | {frontDiff} | {rightDiff}  And final likelihood of {toReturn}"); // some logging
            return toReturn;
        }
        public double GaussianProbability(double difference, double sigma)
        {
            double exponent = -Math.Pow(difference, 2) / (2 * Math.Pow(sigma, 2));
            double coefficient = 1.0 / (sigma * Math.Sqrt(2 * Math.PI));
            double probability = coefficient * Math.Exp(exponent);

            return Math.Max(probability, 1e-10);
        }
        public double GetPredictedDistance(double x, double y, double theta, double sensorDirectionLookingAt, double startingPointDegreeOffsets, (double, double) startingPointOffsets, Grid map, bool drawTheDistances, CustomBitmap mappp)
        {

            double thetaused = theta; 
            if (thetaused > 360) { thetaused -= 360; }
            if (thetaused < 0) { thetaused += 360; }
            double headingRadians = (thetaused-90) * Math.PI / 180.0;

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
            if (drawTheDistances)
            {
                DrawRaycast(shiftedX, shiftedY, angl, 335, map, mappp, Color.Blue);
            }
            return RaycastCone(shiftedX, shiftedY, angl, 335, GlobalConstants.SensorDispersion, map);
        }
        public static double RaycastCone(double startX, double startY, double angle, double maxRange, double dispersion, Grid map)  
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

        public void ResampleGPT()
        {
            int N = Particles.Count;

            double ess = 1.0 / Particles.Sum(p => p.Weight * p.Weight);
            logOfEss.Add((ess, totalWeightPublic));
            


            var snapshot = Particles.ToList();
            List<Particle> newParticles = new List<Particle>(N);
            double step = 1.0 / N;
            double start = Random.Shared.NextDouble() * step;

            double[] cumulative = new double[N];
            cumulative[0] = snapshot[0].Weight;
            for (int i = 1; i < N; i++)
            {
                cumulative[i] = cumulative[i - 1] + snapshot[i].Weight;
            }

            int index = 0;
            double maxWeight = snapshot.Max(p => p.Weight);
            for (int i = 0; i < N; i++)
            {
                double u = start + i * step;
                while (index < N - 1 && u > cumulative[index])
                {
                    index++;
                }

                Particle selected = snapshot[index];
                Particle p = new Particle(selected.X, selected.Y, selected.Theta);
                p.Weight = 1.0 / N;




                double weightRatio = selected.Weight / maxWeight; // ∈ [0, 1]

                double noiseScale = 1.0 - weightRatio;
                noiseScale = 0.1 + noiseScale * 1.9; 


                p.X += RandomGaussian(0, baseJitterXY * noiseScale);
                p.Y += RandomGaussian(0, baseJitterXY * noiseScale);
                p.Theta += RandomGaussian(0, baseJitterTheta * noiseScale);

                p.Theta = NormalizeAngleDeg(p.Theta);


                newParticles.Add(p);
            }
            //int numRandom = (int)(N * 0.05);
            //for (int j = 0; j < numRandom; j++)
            //{
            //    newParticles[j].Theta = RecalcDegree(newParticles[j].Theta, 45);
            //}
            Particles = newParticles;
        }
        private double RandomGaussian(double mean, double stddev)
        {
            double u1 = 1.0 - Random.Shared.NextDouble();
            double u2 = 1.0 - Random.Shared.NextDouble();
            double randStdNormal = Math.Sqrt(-2.0 * Math.Log(u1)) *
                                   Math.Sin(2.0 * Math.PI * u2);
            return mean + stddev * randStdNormal;
        }

        private double NormalizeAngleDeg(double angle)
        {
            angle %= 360;
            return (angle < 0) ? angle + 360 : angle;
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

            lock (_particlesLock)
            {
                snapshot = Particles.ToList();
            }

            double weightSum = snapshot.Sum(p => p.Weight);
            int numberToSample = snapshot.Count() / 3;
            double step = weightSum / numberToSample;
            double start = Random.Shared.NextDouble() * step;

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

                x += snapshot[selectedIndex].X;
                y += snapshot[selectedIndex].Y;
                sumCos += Math.Cos(snapshot[selectedIndex].Theta * Math.PI / 180.0);
                sumSin += Math.Sin(snapshot[selectedIndex].Theta * Math.PI / 180.0);
            }

            double avgX = x / numberToSample;
            double avgY = y / numberToSample;
            double avgTheta = Math.Atan2(sumSin, sumCos) * 180.0 / Math.PI;
            avgTheta = (avgTheta + 360) % 360;
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
