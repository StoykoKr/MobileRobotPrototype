using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RobotAppControl
{
    public class KalmanFilter
    {
        private float Xt, Xt_update, Xt_prev;
        private float Pt, Pt_update, Pt_prev;
        private float Kt, R, Q_low,Q_high; //Q -> variance  R-> measurement constant
        private float TresholdForQSwitch;
        public KalmanFilter(float Q_LOW,float Q_HIGH,float tresholdForQSwitch, float R, float expectedError, float someStartingValue)
        {
            Pt = 1;
            Xt = 0;
            Xt_prev = someStartingValue;
            Pt_prev = expectedError;
            TresholdForQSwitch = tresholdForQSwitch;
            Q_low = Q_LOW;
            Q_high = Q_HIGH;
            this.R = R;
        }

        public float Output(float input)
        {
            Xt_update = Xt_prev;
            if(Math.Abs(Xt_prev - input) > TresholdForQSwitch)
            {
                Pt_update = Pt_prev + Q_high;
            }
            else
            {
                Pt_update = Pt_prev + Q_low;
            }
            Kt = Pt_update / (Pt_update + R);
            Xt = Xt_update + (Kt * (input - Xt_update));
            Pt = (1 - Kt) * Pt_update;
            Xt_prev = Xt;
            Pt_prev = Pt;
            return Xt;
        }
    }
}
