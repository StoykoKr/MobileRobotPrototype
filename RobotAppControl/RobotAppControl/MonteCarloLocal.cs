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
        private double resampleNoiseFactor = 4;
        private double weightScale = 1;
        public double totalWeightPublic = 666;
        private bool enterChaos = false;
        private (double, double, double) lastEstimatedPos = (700, 700, 120);
        // ConcurrentQueue<double> weights = new ConcurrentQueue<double>();
        public List<Particle> Particles { private set; get; }
        private readonly object _particlesLock = new object();
        private ConcurrentDictionary<int, Particle> keyValueParticles;

        public MonteCarloLocal(int particleCount, int CenterX, int CenterY, int range, int tastksCount, Grid map)
        {
            MapMap = map;
            isStarted = true;
            allParticleCount = particleCount;
            numberOfTasksToRunOn = tastksCount;
            Particles = InitializeParticles(particleCount, CenterX - range, CenterX + range, CenterY - range, CenterY + range);
            keyValueParticles = new ConcurrentDictionary<int, Particle>(-1, particleCount);
            currentEstimateWeight = 0;
        }
        private List<Particle> InitializeParticles(int particleCount, int xMin, int xMax, int yMin, int yMax)
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
        private double RecalcDegree(double angle, double range)
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
                // no changes newAngle = Angle
            }
            return newAngle;
        }
        private double MovementVariation(double range)
        {
            Random rand = new Random();
            if (rand.NextDouble() > 0.6)
            {
                return rand.NextDouble() * range;
            }
            else if (rand.NextDouble() < 0.4)
            {
                return rand.NextDouble() * -range;
            }
            else
            {
                return 0;
            }
        }
        private void MoveParticles(double forwardMove, double Angle, int startingIndex, int finalIndex)
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
                double likelihood = CalculateLikelihood(Particles[i], observedData,sigma);
                if (likelihood <= 0) {
                    likelihood = double.Epsilon;
                        }
                if( double.IsNaN(likelihood) || likelihood < 1e-5) { // just a small value 
                   // likelihood = 1e-5;              
                }
                Particles[i].Weight = Math.Max(Particles[i].Weight * likelihood, double.Epsilon);
                total += Particles[i].Weight;
            }
            if (total == 0)
            {
                total = double.Epsilon;
            }
            doubles.Enqueue(total);
        }

        public async Task StartTasksToUpdateWeights(double[] observedData, double sigma)
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
            double normalizationFactor = 1.0 / totalWeight; // gpt shenanigans
            for (int i = 0; i < Particles.Count; i++)
            {
                // Particles[i].Weight /= totalWeight;
                Particles[i].Weight *= normalizationFactor;
            }

            lastEstimatedPos = EstimatePosition();
            totalWeightPublic = totalWeight;

        }
        public double GetDynamicSigma(double expectedValue, double baseFactor = 0.35)
        {
            return Math.Max(baseFactor * expectedValue, 10); 
        }
        public double CalculateLikelihood(Particle particle, double[] observedData, double sigma)
        {
            double predictedFront = GetPredictedDistance(particle.X, particle.Y, particle.Theta, GlobalConstants.DegreeOffsetMid, GlobalConstants.MidDegrees, GlobalConstants.MidSensorOffsets, MapMap); 
            double predictedLeft = GetPredictedDistance(particle.X, particle.Y, particle.Theta, GlobalConstants.DegreeOffsetLeft, GlobalConstants.LeftDegrees, GlobalConstants.LeftSensorOffsets, MapMap); 
            double predictedRight = GetPredictedDistance(particle.X, particle.Y, particle.Theta, GlobalConstants.DegreeOffsetRight, GlobalConstants.RightDegrees, GlobalConstants.RightSensorOffsets, MapMap);


            double frontDiff = observedData[0] - predictedFront;
            double leftDiff = observedData[1] - predictedLeft;
            double rightDiff = observedData[2] - predictedRight;

            double frontLikelihood = GaussianProbability(frontDiff, GetDynamicSigma(predictedFront));  //experiment with dynamic sigma
            double leftLikelihood = GaussianProbability(leftDiff, GetDynamicSigma(predictedLeft));
            double rightLikelihood = GaussianProbability(rightDiff, GetDynamicSigma(predictedRight));

            // return frontLikelihood * leftLikelihood * rightLikelihood;
            return Math.Exp(Math.Log(frontLikelihood) + Math.Log(leftLikelihood) + Math.Log(rightLikelihood));
        }
       
        public double GaussianProbability(double difference, double sigma)
        {
            double exponent = -Math.Pow(difference, 2) / (2 * Math.Pow(sigma, 2));
            double coefficient = 1.0 / (sigma * Math.Sqrt(2 * Math.PI));
            double probability = coefficient * Math.Exp(exponent);

            return Math.Max(probability, 1e-10); // Prevent underflow
        }

        public double GetPredictedDistance(double x, double y, double theta, double sensorDirectionLookingAt,double startingPointDegreeOffsets, (double, double) startingPointOffsets, Grid map)
        {

            double angl = theta + sensorDirectionLookingAt;
            if (angl < 0)
            {
                angl += 360;
            }
            if (angl > 360)
            {
                angl -= 360;
            }
            double shiftedX = x + (int)Math.Round(Math.Sqrt(Math.Pow(startingPointOffsets.Item1, 2) + Math.Pow(startingPointOffsets.Item2, 2)) * Math.Cos((theta + startingPointDegreeOffsets) * Math.PI / 180));
            double shiftedY = y + (int)Math.Round(Math.Sqrt(Math.Pow(startingPointOffsets.Item1, 2) + Math.Pow(startingPointOffsets.Item2, 2)) * Math.Sin((theta + startingPointDegreeOffsets) * Math.PI / 180));

            return RaycastCone(shiftedX,shiftedY, angl,330,GlobalConstants.SensorDispersion,map);
          //  return Raycast(x, y, angl, 300, map);

        }

        public static double RaycastCone(double startX, double startY, double angle, double maxRange, double dispersion, Grid map)
        {
            double stepSize = 1.5;
            double distance = 0;

            double tempAngle;

           while (distance < maxRange)
            {
                for (int i = 0; i <= dispersion; i++)
                {

                    tempAngle = angle - dispersion / 2 + i;
                    if (tempAngle < 0)
                    {
                        tempAngle += 360;
                    }
                    if(tempAngle > 360)
                    {
                        tempAngle -= 360;
                    }
                    
                    double radians = tempAngle * Math.PI / 180.0;
                    int targetX = (int)Math.Round(startX + Math.Abs(distance) * Math.Cos(radians));
                    int targetY = (int)Math.Round(startY + Math.Abs(distance) * Math.Sin(radians));

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

            if (enterChaos)
            {
                for (int i = 0; i < Particles.Count; i++)
                {
                    newParticles.Add(ParticleMaker(new Particle(lastEstimatedPos.Item1, lastEstimatedPos.Item2, lastEstimatedPos.Item3), 650, 650, 90));
                }
            }
            else
            {

                var snapshot = Particles.ToList();
                snapshot.Sort((x, y) =>
                {
                    if (double.IsNaN(x.Weight)) return 1;
                    if (double.IsNaN(y.Weight)) return -1;
                    return y.Weight.CompareTo(x.Weight);
                });

                if (snapshot.Count > 0)
                    cumulativeWeights[0] = snapshot[0].Weight;

                for (int i = 1; i < snapshot.Count; i++)
                {
                    cumulativeWeights[i] = cumulativeWeights[i - 1] + snapshot[i].Weight;
                }

                int quarter = snapshot.Count / 4;
                for (int i = 0; i < snapshot.Count; i++)
                {
                    double randomValue = rand.NextDouble() * cumulativeWeights.Last();
                    int index = Array.BinarySearch(cumulativeWeights, randomValue);
                    if (index < 0) index = ~index;

                    Particle resampledParticle = snapshot[index].Clone();
                    if (index < quarter)
                    {
                        weightScale = 0.1;
                    }
                    else if (index < quarter * 2)
                    {
                        weightScale = 0.75;
                    }
                    else if (index < quarter * 3)
                    {
                        weightScale = 2.5;
                    }
                    else
                    {
                        weightScale = 4;
                    }

                    resampledParticle.X += (rand.NextDouble() * 2 - 1) * resampleNoiseFactor * weightScale;
                    resampledParticle.Y += (rand.NextDouble() * 2 - 1) * resampleNoiseFactor * weightScale;
                    if (forceTheta)
                    {
                        resampledParticle.Theta = thetaToBeForced;

                    }
                    resampledParticle.Theta += (rand.NextDouble() * 2 - 1) * (resampleNoiseFactor / 2) * weightScale;

                    newParticles.Add(resampledParticle);
                }
            }
            // Правилно ли е освобождава паметта?!?
            Particles = newParticles; // Replace old particles with resampled set
        }


        private const double TOLERANCE = 1e-7; // Adjust based on needed precision
        public (double X, double Y, double Theta) EstimatePosition()
        {
            List<Particle> snapshot;

            lock (_particlesLock)
            {
                snapshot = Particles.ToList(); 
            }
            snapshot.Sort((x, y) =>
            {
                bool xNaN = double.IsNaN(x.Weight);
                bool yNaN = double.IsNaN(y.Weight);

                if (xNaN && yNaN) return 0;  
                if (xNaN) return 1;         
                if (yNaN) return -1;

                double diff = y.Weight - x.Weight;
                if (Math.Abs(diff) < TOLERANCE) return 0; // Treat as equal
                return diff > 0 ? 1 : -1;

            });

            int parCount = snapshot.Count / 2;
            double x = 0.0, y = 0.0;
            double sumSin = 0.0, sumCos = 0.0;

            for (int i = 0; i < parCount; i++)
            {
                x += snapshot[i].X;
                y += snapshot[i].Y;

                sumCos += Math.Cos(snapshot[i].Theta * Math.PI / 180.0);
                sumSin += Math.Sin(snapshot[i].Theta * Math.PI / 180.0);
            }

            double avgX = x / parCount;
            double avgY = y / parCount;


            double avgTheta = Math.Atan2(sumSin, sumCos) * 180.0 / Math.PI;

            return (avgX, avgY, avgTheta);
        }

        private Particle ParticleMaker(Particle model, int xRange, int yRange, int thetaRange)
        {
            Random rand = new Random();
            int newX = 0;
            int newY = 0;
            double newTheta = RecalcDegree(model.Theta, thetaRange);

            newX = (int)(model.X + (rand.NextInt64(-50, 50) / 50) * xRange);

            newY = (int)(model.Y + (rand.NextInt64(-50, 50) / 50) * yRange);

            Particle part = new Particle(newX, newY, newTheta);
            part.Weight = double.Epsilon;
            return part;

        }
        public static double Raycast(double startX, double startY, double angle, double maxRange, Grid map) // WILL NOT work for the final thing but this will help with basic MCL testing
        {

            double stepSize = 1.25;
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

                if (!map.IsWalkable((int)currentX, (int)currentY) == true)
                {
                    return distance;
                }
            }
            return maxRange;
        }

    }
}
