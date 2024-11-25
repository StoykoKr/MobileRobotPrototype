using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.ComponentModel;
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
    internal class MonteCarloLocal
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
       // ConcurrentQueue<double> weights = new ConcurrentQueue<double>();
        public List<Particle> Particles { private set; get; }
        private ConcurrentDictionary<int,Particle> keyValueParticles;
        
        public MonteCarloLocal(int particleCount, int CenterX, int CenterY, int range,int tastksCount, Grid map)
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
        private void MoveParticles(double forwardMove, double Angle,int startingIndex,int finalIndex)
        {
            for (int i = startingIndex; i <= finalIndex; i++)
            {
                Particles[i].Theta = RecalcDegree(Angle, 5);
                Particles[i].X += (forwardMove + (rand.NextDouble() * 0.2)) * Math.Cos(Particles[i].Theta * Math.PI / 180);
                Particles[i].Y += (forwardMove + (rand.NextDouble() * 0.2)) * Math.Sin(Particles[i].Theta * Math.PI / 180);              
            }
        }
        public async Task StartTasksToMoveParticles(float move, float dir)
        {
            int remainingIndexes = allParticleCount;
            int starIndex = 0;
            int endIndex = 0;           
            List<Task> tasks = new List<Task>();
            for (int j = 1; j <= numberOfTasksToRunOn; j++)
            {
                endIndex = remainingIndexes - 1;
                remainingIndexes = remainingIndexes - allParticleCount / numberOfTasksToRunOn;
                if (j == numberOfTasksToRunOn)
                {
                    remainingIndexes -= allParticleCount % numberOfTasksToRunOn;
                }
                starIndex = remainingIndexes;
                Task task = new Task(() => MoveParticles(move, dir, starIndex,endIndex));
                task.Start();
                tasks.Add(task);
            }
            await Task.WhenAll(tasks);
        }
        private bool enterChaos = false;
        private (double, double, double) lastEstimatedPos = (700, 700, 120);

        public void UpdateWeightsOld(double[] observedData, double sigma)
        {
            double totalWeight = 0;

            for (int i = 0; i < Particles.Count; i++)
            {
                double likelihood = CalculateLikelihood(Particles[i], observedData, sigma);

                Particles[i].Weight *= likelihood;

                totalWeight += Particles[i].Weight;
            }
            if (totalWeight < 0.0001)
            {
               // totalWeight = 0.1;
                enterChaos = true;
            }
            else
            {
                enterChaos = false;
                for (int i = 0; i < Particles.Count; i++)
                {
                    Particles[i].Weight /= totalWeight;
                }
                lastEstimatedPos = EstimatePosition();
            }
            totalWeightPublic = totalWeight;


        }
        public void UpdateWeights(double[] observedData, double sigma,int startingIndex, int finalIndex,ConcurrentQueue<double> doubles)
        {
            double total = 0;
            
            for (int i = startingIndex; i <= finalIndex; i++)
            {
                double likelihood = CalculateLikelihood(Particles[i], observedData, sigma);
                Particles[i].Weight = Math.Max(Particles[i].Weight * likelihood, double.Epsilon);
               // Particles[i].Weight *= likelihood;
                total += Particles[i].Weight;
            }
            doubles.Enqueue(total);
        
        }
       
        public async Task StartTasksToUpdateWeights(double[] observedData, double sigma)
        {
            ConcurrentQueue<double> weights = new ConcurrentQueue<double>();
            int remainingIndexes = allParticleCount;
            List<Task> tasks = new List<Task>();
          //  int starIndex = 0;   //FCKING muthithreading bullsh*t if I make these two outside of the loop it breaks everything.. God damn funky logic
           // int endIndex = 0;
         /*   for (int i = 0; i < allParticleCount; i++)
            {
                keyValueParticles[i] = Particles[i];
            }*/
            for (int j = 1; j <= numberOfTasksToRunOn; j++)
            {
                int endIndex = remainingIndexes - 1;
                remainingIndexes = remainingIndexes - allParticleCount / numberOfTasksToRunOn;
                if (j == numberOfTasksToRunOn)
                {
                    remainingIndexes -= allParticleCount % numberOfTasksToRunOn;
                }
                int starIndex = remainingIndexes;             
                 Task task = new Task(() => UpdateWeights(observedData,sigma,starIndex,endIndex,weights));
                 tasks.Add(task);
                 task.Start();
            }
            await Task.WhenAll(tasks);
         /*   for (int i = 0; i < allParticleCount; i++)
            {
                Particles[i] = keyValueParticles[i];
            }*/
            double totalWeight = 0;
            while (weights.TryDequeue(out double result))
            {
                totalWeight += result;
            }
            if (totalWeight <= 0)
            {
                throw new Exception("wtf");
            }
            //if (totalWeight < 0.002)
           // {
               // totalWeight = 0.1;
             //   enterChaos = true;
          //  }
          //  else
          //  {
               
         //       enterChaos = false;
                for (int i = 0; i < Particles.Count; i++)
                {
                    Particles[i].Weight /= totalWeight;
                }
                lastEstimatedPos = EstimatePosition();
          //  }
            totalWeightPublic = totalWeight;

        }
        public void Resample()
        {
            List<Particle> newParticles = new List<Particle>();
            double[] cumulativeWeights = new double[allParticleCount];

            if (enterChaos)
            {
                for (int i = 0; i < Particles.Count; i++)
                {
                    newParticles.Add(ParticleMaker(new Particle(lastEstimatedPos.Item1,lastEstimatedPos.Item2,lastEstimatedPos.Item3),650,650,90));
                }
            }
            else
            {
            Particles.Sort(delegate (Particle x, Particle y)
            {
                return y.Weight.CompareTo(x.Weight);
            });
            if(Particles.Count > 0)
            cumulativeWeights[0] = Particles[0].Weight;

            for (int i = 1; i < Particles.Count; i++)
            {
                cumulativeWeights[i] = cumulativeWeights[i - 1] + Particles[i].Weight;
            }

            int quarter = Particles.Count / 4;
                for (int i = 0; i < Particles.Count; i++)
                {
                    double randomValue = rand.NextDouble() * cumulativeWeights.Last();
                    int index = Array.BinarySearch(cumulativeWeights, randomValue);
                    if (index < 0) index = ~index;

                    Particle resampledParticle = Particles[index].Clone();
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
                    resampledParticle.Theta += (rand.NextDouble() * 2 - 1) * (resampleNoiseFactor / 2) * weightScale;

                    newParticles.Add(resampledParticle);
                }
            }

            Particles = newParticles; // Replace old particles with resampled set
        }

        public double CalculateLikelihood(Particle particle, double[] observedData, double sigma)
        {
            // Example: We assume observedData contains distance measurements (front, left, right)
            double predictedFront = GetPredictedDistance(particle.X, particle.Y, particle.Theta, 0, MapMap); // Front
            double predictedLeft = GetPredictedDistance(particle.X, particle.Y, particle.Theta, -90, MapMap); // Left
            double predictedRight = GetPredictedDistance(particle.X, particle.Y, particle.Theta, 90, MapMap); // Right


            double frontDiff = observedData[0] - predictedFront;
            double leftDiff = observedData[1] - predictedLeft;
            double rightDiff = observedData[2] - predictedRight;

            // Use Gaussian probability for each sensor
            double frontLikelihood = GaussianProbability(frontDiff, sigma);
            double leftLikelihood = GaussianProbability(leftDiff, sigma);
            double rightLikelihood = GaussianProbability(rightDiff, sigma);

            // Combine likelihoods (product of independent probabilities)
            return frontLikelihood * leftLikelihood * rightLikelihood;
        }
        public double GaussianProbability(double difference, double sigma)
        {
            double exponent = -Math.Pow(difference, 2) / (2 * Math.Pow(sigma, 2));
            double coefficient = 1.0 / (sigma * Math.Sqrt(2 * Math.PI));
            return coefficient * Math.Exp(exponent);
        }
        public static double SimilarityScore(double difference, double sigma)
        {
            // Ensure sigma is positive to avoid division by zero or invalid input
            if (sigma <= 0)
                throw new ArgumentException("Sigma must be greater than 0", nameof(sigma));

            // Exponential decay formula
            double score = Math.Exp(-difference / sigma);

            // Ensure the score is never exactly 0
            return Math.Max(score, double.Epsilon); // double.Epsilon is the smallest positive value
        }
        public double GetPredictedDistance(double x, double y, double theta, double offset, Grid map)
        {
            // Raycasting logic to get the predicted distance to the nearest obstacle
            // Offset is the direction offset for left/right sensor
            // This method should simulate sensor readings based on the map's obstacle layout
            double angl = theta + offset;
            if (angl < 0)
            {
                angl += 360;
            }
            if (angl > 360)
            {
                angl -= 360;
            }

            return Raycast(x, y, angl, 300, map); // Placeholder for actual raycasting logic
        }
        public (double X, double Y, double Theta) EstimatePosition()
        {
            Particles.Sort(delegate (Particle x, Particle y)
            {
                return y.Weight.CompareTo(x.Weight);
            });
            int parCount = Particles.Count / 2;
            double x = 0.0, y = 0.0, theta = 0.0;

            for (int i = 0; i < parCount; i++)
            {
                x += Particles[i].X;
                y += Particles[i].Y;
                theta += Particles[i].Theta;
            }           
            return (x / parCount, y / parCount, theta / parCount);

        }
        private Particle ParticleMaker(Particle model, int xRange, int yRange, int thetaRange)
        {
            Random rand = new Random();
            int newX = 0;
            int newY = 0;
            double newTheta = RecalcDegree(model.Theta, thetaRange);

                newX = (int)(model.X + (rand.NextInt64(-50, 50) / 50) * xRange);
 
                newY = (int)(model.Y + (rand.NextInt64(-50, 50) / 50) * yRange);
            
            Particle part =  new Particle(newX, newY, newTheta);
            part.Weight = double.Epsilon;
            return part;

        }
        public double Raycast(double startX, double startY, double angle, double maxRange, Grid map) // WILL NOT work for the final thing but this will help with basic MCL testing
        {
            // Define step size (how much to move along the ray each iteration)
            double stepSize = 1.25; // Small step size for accuracy
            double distance = 0;

            // Calculate ray's direction (based on the angle)
            double dirX = Math.Cos(angle * Math.PI / 180);
            double dirY = Math.Sin(angle * Math.PI / 180);

            // Current position of the ray (starting at the particle's position)
            double currentX = startX;
            double currentY = startY;

            // Move along the ray until we hit an obstacle or exceed max range
            while (distance < maxRange)
            {
                // Move the ray forward by one step
                currentX += dirX * stepSize;
                currentY += dirY * stepSize;
                distance += stepSize;

                // Check if the ray has gone out of bounds
                if (!map.IsWalkable((int)currentX, (int)currentY) == true)//!map.IsValidPosition((int)currentX, (int)currentY))
                {
                    // If the ray is out of bounds or hits an obstacle, stop
                    return distance;
                }
            }

            // If no obstacle was hit, return the max range (sensor sees no obstacle)
            return maxRange;
        }

    }
}
