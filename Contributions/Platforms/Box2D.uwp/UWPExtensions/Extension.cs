using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using Windows.UI;

namespace Box2D.uwp.UWPExtensions
{
    public static class Extension
    {
        public static Vector2 Normalize(this Vector2 vec)
        {
            return Vector2.Normalize(vec);
        }
    }
}
