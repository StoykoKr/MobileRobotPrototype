using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net.Sockets;
using System.Text;
using System.Threading.Tasks;

namespace RobotAppControl
{
    internal class Listener
    {
        private int i;
        private Byte[] bytes = new Byte[1024];
        private bool stopListening = false;
        char character;
        Queue<char> rawCharsToWorkWith;
        ConcurrentQueue<string> stringsToBeInterpreted;
        StringBuilder helperStringBuilder;
        string lastQueued = "";
        public Listener(ref ConcurrentQueue<string> strings)
        {          
            rawCharsToWorkWith = new Queue<char>();
            stringsToBeInterpreted = strings;
            helperStringBuilder = new StringBuilder();
        }
        private void MergeInStrings() // Looks at the data in the char queue and makes complete strings using a StringBuilder until a chosen char appears as a delimiter. The complete strings are enqueued to another queue that is consumed by the InterpreterThread
        {

            while (rawCharsToWorkWith.Count > 0)
            {
                character = rawCharsToWorkWith.Dequeue();
                if (character == '`')
                {

                    if (helperStringBuilder.ToString().Equals(lastQueued))
                    {
                        helperStringBuilder.Clear();
                    }
                    else
                    {
                        lastQueued = helperStringBuilder.ToString();
                    stringsToBeInterpreted.Enqueue(lastQueued);
                    helperStringBuilder.Clear();
                    }
                }
                else
                {
                    helperStringBuilder.Append(character);
                }
            }

        }
        public void Stop()
        {
            stopListening = true;
        }
        public void BeginListening(NetworkStream stream)  // Starts listening to the NetworkStream. Saves the incoming chars in a queue.
        {
            while (!stopListening)
            {
                try
                {
                    if ((i = stream.Read(bytes, 0, bytes.Length)) != 0)
                    {
                        foreach (var item in System.Text.Encoding.ASCII.GetChars(bytes, 0, i))
                        {
                            rawCharsToWorkWith.Enqueue(item);
                        }
                        MergeInStrings();
                    }
                    MergeInStrings();
                }
                catch (Exception ex)
                {
                    Console.WriteLine(ex.Source);
                    Console.WriteLine(ex.Message);
                }
            }
        }
    }
}
