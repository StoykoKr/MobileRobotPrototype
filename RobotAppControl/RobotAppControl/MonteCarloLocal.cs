using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing.Text;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RobotAppControl
{
    internal class MonteCarloLocal
    {
        
        public bool isStarted { private set; get; }
        private int xCenter;
        private int yCenter;
        private int CenterRadius;
        private double radiusUncertainty;
        public List<Particle> Particles { private set; get; }
        public MonteCarloLocal(int particleCount, int CenterX, int CenterY, int range, double radiusUncertainty)
        {
            isStarted = true;
            xCenter = CenterX;
            yCenter = CenterY;
            CenterRadius = range;
            this.radiusUncertainty = radiusUncertainty;
            Particles = InitializeParticles(particleCount, CenterX - range, CenterX + range, CenterY - range, CenterY + range);
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
        private int counter = 0;
        public void MoveParticles(double forwardMove, double Angle)
        {
            Random rand = new Random();
            for(int i = 0;i < Particles.Count; i++)
            {              
                double newAngle = Angle;
                if (rand.NextDouble() > 0.6)
                {
                    newAngle += radiusUncertainty * rand.NextDouble();
                    if (newAngle > 360)
                    {
                        newAngle -= 360;
                    }
                }
                else if (rand.NextDouble() < 0.4)
                {
                    newAngle -= radiusUncertainty * rand.NextDouble();
                    if (newAngle < 0)
                    {
                        newAngle += 360;
                    }
                }
                else
                {
                    // no changes newAngle = Angle
                }

                Particles[i].Theta = newAngle;

                Particles[i].X += forwardMove * Math.Cos(Particles[i].Theta * Math.PI / 180);
                Particles[i].Y += forwardMove * Math.Sin(Particles[i].Theta * Math.PI / 180);
            }
            counter++;
        }


        public static double CalculateGaussianProbability(double predicted, double actual, double sigma)
        {
            // Adjust sigma if it's too small for the expected ranges
            if (sigma < 1)
            {
                sigma = 1; // Ensure sigma isn't too small to avoid quick falloff
            }

            double exponent = -Math.Pow(predicted - actual, 2) / (2 * Math.Pow(sigma, 2));
            double coefficient = 1.0 / (sigma * Math.Sqrt(2 * Math.PI));

            // Special cases for value 300
            if (predicted == 300 || actual == 300)
            {
                return 0.33;
            }
            else if (predicted == actual && actual == 300)
            {
                return 0.1;
            }

            // To avoid returning values too close to 0
            double minProbability = 0.1;
            return Math.Max(coefficient * Math.Exp(exponent), minProbability);
        }

        public static double CalculateParticleWeight(double[] predictedDistances, double[] actualDistances, double sigma)
        {
            double weight = 1.0;

            // Assuming we have 3 sensors: front, left, right
            for (int i = 0; i < predictedDistances.Length; i++)
            {
                double probability = CalculateGaussianProbability(predictedDistances[i], actualDistances[i], sigma);
                weight *= probability;
            }

            if(weight > 0.93)
            {
                return 0.93;
            }
            return weight;
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
                Particles[i].Weight = CalculateParticleWeight([predictedFront,predictedLeft,predictedRight], sensorMeasurements,7.5);
                if(Particles[i].Weight > 0.93)
                {
                    Particles[i].Weight = 0.93;
                }
            }
            //Particles = ResampleParticles();
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
            double x = 0.0, y = 0.0, theta = 0.0, totalWeight = 0.0;

            foreach (var p in Particles)
            {
                x += p.X * p.Weight;
                y += p.Y * p.Weight;
                theta += p.Theta * p.Weight;
                totalWeight += p.Weight;
            }
            if(totalWeight == 0)
            {
                return (1, 1, 1);
            }
            else
            {
            return (x / totalWeight, y / totalWeight, theta / totalWeight);
            }
        }
        public List<Particle> ResampleParticles()  // THIS IS BORKED
        {
           List<Particle> newParticles = new List<Particle>();
            Random rand = new Random();
            double[] cumulativeWeights = new double[Particles.Count];
            cumulativeWeights[0] = Particles[0].Weight;

            for (int i = 1; i < Particles.Count; i++)
                cumulativeWeights[i] = cumulativeWeights[i - 1] + Particles[i].Weight;

            for (int i = 0; i < Particles.Count; i++)
            {
                double randomWeight = rand.NextDouble() * cumulativeWeights.Last();
                int index = Array.BinarySearch(cumulativeWeights, randomWeight);

                if (index < 0)
                    index = ~index; // In case binary search returns a negative index

                newParticles.Add(new Particle(Particles[index].X, Particles[index].Y, Particles[index].Theta));
            }

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
