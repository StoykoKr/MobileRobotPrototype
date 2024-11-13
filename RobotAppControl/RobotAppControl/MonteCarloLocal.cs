using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing.Text;
using System.IO;
using System.Linq;
using System.Reflection.Metadata;
using System.Text;
using System.Threading.Tasks;
using static System.Windows.Forms.VisualStyles.VisualStyleElement.Rebar;

namespace RobotAppControl
{
    internal class MonteCarloLocal
    {

        public bool isStarted { private set; get; }
        public double currentEstimateWeight;
        private int minX, maxX;
        private int minY, maxY;
        public Random rand = new Random();
        private Grid MapMap = null;
        private int allParticleCount = 0;
        public List<Particle> Particles { private set; get; }
        public MonteCarloLocal(int particleCount, int CenterX, int CenterY, int range, Grid map)
        {
            minX = 1;
            MapMap = map;
            maxX = map.Width - 1;
            minY = 1; maxY = map.Height - 1;
            isStarted = true;
            allParticleCount = particleCount;
            Particles = InitializeParticles(particleCount, CenterX - range, CenterX + range, CenterY - range, CenterY + range);
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
        public void MoveParticles(double forwardMove, double Angle)
        {
            for (int i = 0; i < Particles.Count; i++)
            {
                Particles[i].Theta = RecalcDegree(Angle, 5);
                Particles[i].X += (forwardMove + (rand.NextDouble() * 0.2)) * Math.Cos(Particles[i].Theta * Math.PI / 180);
                Particles[i].Y += (forwardMove + (rand.NextDouble() * 0.2)) * Math.Sin(Particles[i].Theta * Math.PI / 180);
            }
        }
        private bool enterChaos = false;
        private (double, double, double) lastEstimatedPos = (700, 700, 120);
        public void UpdateWeights(double[] observedData, double sigma)
        {
            double totalWeight = 0;

            for (int i = 0; i < Particles.Count; i++)
            {
                // Compute likelihood P(D | x_i) using Gaussian probability
                double likelihood = CalculateLikelihood(Particles[i], observedData, sigma);

                // Update particle weight: P(H|D) ∝ P(D|H) * P(H) (Bayes Law)
                Particles[i].Weight *= likelihood;

                totalWeight += Particles[i].Weight;
            }
            if (totalWeight == 0)
            {
                totalWeight = 0.1;
                enterChaos = true;
            }
            else
            {
                enterChaos = false;
                lastEstimatedPos = EstimatePosition();
                for (int i = 0; i < Particles.Count; i++)
                {
                    Particles[i].Weight /= totalWeight;
                }
            }

        
        }
        private double resampleNoiseFactor = 3;
        private double weightScale = 1;
        public void Resample()
        {
            List<Particle> newParticles = new List<Particle>();
            double[] cumulativeWeights = new double[allParticleCount];
            Particles.Sort(delegate (Particle x, Particle y)
            {
                return y.Weight.CompareTo(x.Weight);
            });
            if(Particles.Count > 0)
            cumulativeWeights[0] = Particles[0].Weight;

            // Build cumulative weight distribution
            for (int i = 1; i < Particles.Count; i++)
            {
                cumulativeWeights[i] = cumulativeWeights[i - 1] + Particles[i].Weight;
            }

            int quarter = Particles.Count / 4;

            if (enterChaos)
            {
                for (int i = 0; i < Particles.Count; i++)
                {
                    newParticles.Add(ParticleMaker(new Particle(lastEstimatedPos.Item1,lastEstimatedPos.Item2,lastEstimatedPos.Item3),350,350,90));
                }
            }
            else
            {
                for (int i = 0; i < Particles.Count; i++)
                {
                    double randomValue = rand.NextDouble() * cumulativeWeights.Last();
                    int index = Array.BinarySearch(cumulativeWeights, randomValue);
                    if (index < 0) index = ~index;

                    // newParticles.Add(Particles[index].Clone()); // Add resampled particle
                    Particle resampledParticle = Particles[index].Clone();
                    if (index < quarter)
                    {
                        weightScale = 1;
                    }
                    else if (index < quarter * 2)
                    {
                        weightScale = 2;
                    }
                    else if (index < quarter * 3)
                    {
                        weightScale = 3;
                    }
                    else
                    {
                        weightScale = 4;
                    }

                    // Add random noise to the particle's state to maintain diversity
                    resampledParticle.X += (rand.NextDouble() * 2 - 1) * resampleNoiseFactor * weightScale;
                    resampledParticle.Y += (rand.NextDouble() * 2 - 1) * resampleNoiseFactor * weightScale;
                    resampledParticle.Theta += (rand.NextDouble() * 2 - 1) * resampleNoiseFactor / 2 * weightScale;

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

        public static double CalculateGaussianProbability(double predicted, double actual, double sigma)
        {

            if (predicted == actual && actual == 300)
            {
                return 0.1;
            }
            else if (predicted == 300 || actual == 300)
            {
                return 0.2;
            }
            /*
            double exponent = -Math.Pow(predicted - actual, 2) / (2 * Math.Pow(sigma, 2));
            double coefficient = 1.0 / (sigma * Math.Sqrt(2 * Math.PI));


            // To avoid returning values too close to 0
            double minProbability = 0.1;
            return Math.Max(coefficient * Math.Exp(exponent), minProbability);*/
            double difference = Math.Abs(actual - predicted);
            if (difference <= sigma)
            {
                return 1.0;
            }

            // If the difference exceeds the allowed variance, calculate a decreasing score
            double similarity = 1.0 - (difference - sigma) / (Math.Max(actual, predicted) - Math.Min(actual, predicted));

            // Clamp the similarity score to a minimum of 0
            return Math.Max(0, similarity);
        }

        public static double CalculateParticleWeight(double[] predictedDistances, double[] actualDistances, double sigma)
        {
            double weight = 0;

            // Assuming we have 3 sensors: front, left, right
            for (int i = 0; i < predictedDistances.Length; i++)
            {
                double probability = CalculateGaussianProbability(predictedDistances[i], actualDistances[i], sigma);
                weight += probability;
            }
            return weight / 3;
        }


        public double MeasureProbability(double predicted, double actual, double stdDev)
        {
            // Gaussian probability
            double error = actual - predicted;
            return Math.Exp(-(error * error) / (2 * stdDev * stdDev)) / Math.Sqrt(2 * Math.PI * stdDev * stdDev);
        }
        public void UpdateParticleWeights(double[] sensorMeasurements, Grid map, double sensorNoise)
        {
            for (int i = 0; i < Particles.Count; i++)
            {
                // Simulate sensor readings for this particle
                double predictedFront = GetPredictedDistance(Particles[i].X, Particles[i].Y, Particles[i].Theta, 0, map); // Front
                double predictedLeft = GetPredictedDistance(Particles[i].X, Particles[i].Y, Particles[i].Theta, -90, map); // Left
                double predictedRight = GetPredictedDistance(Particles[i].X, Particles[i].Y, Particles[i].Theta, 90, map); // Right


                // weight *= MeasureProbability(predictedFront, sensorMeasurements[0], sensorNoise);
                // weight *= MeasureProbability(predictedLeft, sensorMeasurements[1], sensorNoise);
                //  weight *= MeasureProbability(predictedRight, sensorMeasurements[2], sensorNoise);         
                Particles[i].Weight = CalculateParticleWeight([predictedFront, predictedLeft, predictedRight], sensorMeasurements, 1.5);

            }
            Particles = ResampleParticles();
            for (int i = 0; i < Particles.Count; i++)
            {
                // Simulate sensor readings for this particle
                double predictedFront = GetPredictedDistance(Particles[i].X, Particles[i].Y, Particles[i].Theta, 0, map); // Front
                double predictedLeft = GetPredictedDistance(Particles[i].X, Particles[i].Y, Particles[i].Theta, -90, map); // Left
                double predictedRight = GetPredictedDistance(Particles[i].X, Particles[i].Y, Particles[i].Theta, 90, map); // Right


                // weight *= MeasureProbability(predictedFront, sensorMeasurements[0], sensorNoise);
                // weight *= MeasureProbability(predictedLeft, sensorMeasurements[1], sensorNoise);
                //  weight *= MeasureProbability(predictedRight, sensorMeasurements[2], sensorNoise);         
                Particles[i].Weight = CalculateParticleWeight([predictedFront, predictedLeft, predictedRight], sensorMeasurements, 1.5);

            }
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
            });//*/
            int parCount = Particles.Count / 2;
            double x = 0.0, y = 0.0, theta = 0.0;

            for (int i = 0; i < parCount; i++)
            {
                x += Particles[i].X;
                y += Particles[i].Y;
                theta += Particles[i].Theta;
            }
            //
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
            
            return new Particle(newX, newY, newTheta);

        }
        public List<Particle> ResampleParticles()  // THIS IS BORKED
        {
            List<Particle> newParticles = new List<Particle>();
            Random rand = new Random();
            // int quarter = Particles.Count / 4;
            var estimated = EstimatePosition();
            bool WeDoTwoNow = true;
            for (int i = 0; i < Particles.Count; i++)
            {
                if (Particles[i].Weight > 0.75)
                {
                    newParticles.Add(new Particle(Particles[i].X, Particles[i].Y, RecalcDegree(Particles[i].Theta, 2)));
                }
                else if (Particles[i].Weight < 0.1)
                {
                    // newParticles.Add(new Particle(rand.NextDouble() * (maxX - 1), rand.NextDouble() * (maxY - 1), RecalcDegree(estimated.Theta, 180)));
                    //newParticles.Add(ParticleMaker(Particles[i], 5, 5, 5));
                    newParticles.Add(ParticleMaker(new Particle(estimated.X, estimated.Y, estimated.Theta), 5, 5, 5));
                }
                else
                {
                    newParticles.Add(ParticleMaker(new Particle(estimated.X, estimated.Y, estimated.Theta), 5, 5, 5));
                    //  newParticles.Add(ParticleMaker(Particles[i], 5, 5, 5));
                    // newParticles.Add(new Particle(rand.NextDouble() * (maxX - 1), rand.NextDouble() * (maxY - 1), RecalcDegree(estimated.Theta, 180)));
                    //  newParticles.Add(ParticleMaker(new Particle(estimated.X, estimated.Y, estimated.Theta), 15, 15, 25));
                }












                /*
                newParticles.Add(new Particle(Particles[i].X, Particles[i].Y, Particles[i].Theta));
                if (WeDoTwoNow)
                {
                    WeDoTwoNow = false;

                    newParticles.Add(ParticleMaker(Particles[i],5,5,15));
                    // newParticles.Add(new Particle(Particles[i].X + (rand.NextInt64(-50,50)/50) * 15, Particles[i].Y + (rand.NextInt64(-50, 50) / 50) * 15, RecalcDegree(Particles[i].Theta, 45)));

                    //   newParticles.Add(new Particle(Particles[i].X + (rand.NextInt64(-50, 50) / 50) * 5, Particles[i].Y + (rand.NextInt64(-50, 50) / 50) * 5, RecalcDegree(Particles[i].Theta, 45)));
                    // newParticles.Add(new Particle((estimated.X + (rand.NextInt64(-50, 50) / 50) * 15) < 1? 1: (estimated.X + (rand.NextInt64(-50, 50) / 50) * 15) > maxX ? maxX - 1: (estimated.X + (rand.NextInt64(-50, 50) / 50) * 15),
                    //    (estimated.Y + (rand.NextInt64(-50, 50) / 50) * 15) < 1 ? 1 : (estimated.Y + (rand.NextInt64(-50, 50) / 50) * 15) > maxY ? maxY - 1 : (estimated.Y + (rand.NextInt64(-50, 50) / 50) * 15), RecalcDegree(estimated.Theta, 180)));
                    // newParticles.Add(new Particle(estimated.X + (rand.NextInt64(-50, 50) / 50) * 150, estimated.Y + (rand.NextInt64(-50, 50) / 150) * 150, estimated.Theta));

                }
                else
                {
                    WeDoTwoNow = true;
                    newParticles.Add(ParticleMaker(Particles[i], 5, 5, 15));
                   // newParticles.Add(ParticleMaker(Particles[i], 5, 5, 45));
                    //newParticles.Add(new Particle(Particles[i].X + (rand.NextInt64(-50, 50) / 50) * 15, Particles[i].Y + (rand.NextInt64(-50, 50) / 50) * 15, RecalcDegree(Particles[i].Theta, 45)));
                   // newParticles.Add(new Particle(Particles[i].X + (rand.NextInt64(-50, 50) / 50) * 15, Particles[i].Y + (rand.NextInt64(-50, 50) / 50) * 15, RecalcDegree(Particles[i].Theta, 45)));
                //    newParticles.Add(new Particle(Particles[i].X + (rand.NextInt64(-50, 50) / 50) * 5, Particles[i].Y + (rand.NextInt64(-50, 50) / 50) * 5, RecalcDegree(Particles[i].Theta, 45)));                 
                }*/
            }
            //  while(newParticles.Count() < Particles.Count())
            //   {
            //  newParticles.Add(ParticleMaker(new Particle(estimated.X,estimated.Y,estimated.Theta), 15, 15, 25));
            //newParticles.Add(new Particle(estimated.X + (rand.NextInt64(-50, 50) / 50) * 150, estimated.Y + (rand.NextInt64(-50, 50) / 150) * 150, estimated.Theta));
            //newParticles.Add(new Particle(estimated.X + (rand.NextInt64(-50, 50) / 50) * 200, estimated.Y + (rand.NextInt64(-50, 50) / 50) * 200, RecalcDegree(estimated.Theta, 180)));
            // newParticles.Add(new Particle(rand.NextDouble() * (maxX-1), rand.NextDouble() * (maxY-1), RecalcDegree(estimated.Theta, 180)));
            // }

            return newParticles;
        }
        public double Raycast(double startX, double startY, double angle, double maxRange, Grid map) // WILL NOT work for the final thing but this will help with basic MCL testing
        {
            // Define step size (how much to move along the ray each iteration)
            double stepSize = 0.5; // Small step size for accuracy
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
