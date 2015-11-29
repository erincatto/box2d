using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Windows.UI;

namespace Box2D.uwp.UWPExtensions
{
    class ColorEx
    {
        public static Color FromScRgb(float r, float g, float b)
        {
            return FromScRgb(1.0f, r, g, b);
        }
        ///<summary>
        /// FromScRgb
        ///</summary>
        public static Color FromScRgb(float a, float r, float g, float b)
        {
            Color c1 = new Color();
            if (a < 0.0f)
            {
                a = 0.0f;
            }
            else if (a > 1.0f)
            {
                a = 1.0f;
            }
            c1.A = (byte)((a * 255.0f) + 0.5f);
            c1.R = ScRgbTosRgb(r);
            c1.G = ScRgbTosRgb(g);
            c1.B = ScRgbTosRgb(b);
            return c1;
        }
        internal static byte ScRgbTosRgb(float val)
        {
            if (!(val > 0.0))
            {
                return (0);
            }
            else if (val <= 0.0031308)
            {
                return ((byte)((255.0f * val * 12.92f) + 0.5f));
            }
            else if (val < 1.0)
            {
                return ((byte)((255.0f * ((1.055f * (float)Math.Pow((double)val, (1.0 / 2.4))) - 0.055f)) + 0.5f));
            }
            else
            {
                return (255);
            }
        }
    }
}
