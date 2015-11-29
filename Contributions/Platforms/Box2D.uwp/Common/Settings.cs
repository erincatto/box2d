/* Box2D.uwp port of Box2D.Xna: Copyright (c) 2015 Nukepayload2
* Box2D.Xna port of Box2D:
* Copyright (c) 2009 Brandon Furtwangler, Nathan Furtwangler
*
* Original source Box2D:
* Copyright (c) 2006-2009 Erin Catto http://www.gphysics.com 
* 
* This software is provided 'as-is', without any express or implied 
* warranty.  In no event will the authors be held liable for any damages 
* arising from the use of this software. 
* Permission is granted to anyone to use this software for any purpose, 
* including commercial applications, and to alter it and redistribute it 
* freely, subject to the following restrictions: 
* 1. The origin of this software must not be misrepresented; you must not 
* claim that you wrote the original software. If you use this software 
* in a product, an acknowledgment in the product documentation would be 
* appreciated but is not required. 
* 2. Altered source versions must be plainly marked as such, and must not be 
* misrepresented as being the original software. 
* 3. This notice may not be removed or altered from any source distribution. 
*/

using System;
namespace Box2D.UWP
{
    public static class Settings
    {
        public static float b2_FLT_MAX = 3.402823466e+38f;
        public static float b2_FLT_EPSILON = 1.192092896e-07f;
        public static float b2_pi = 3.14159265359f;

        /// The maximum number of contact points between two convex shapes.
        public static int b2_maxManifoldPoints = 2;

        /// The maximum number of vertices on a convex polygon.
        public static int b2_maxPolygonVertices = 8;

        /// This is used to fatten AABBs in the dynamic tree. This allows proxies
        /// to move by a small amount without triggering a tree adjustment.
        /// This is in meters.
        public static float b2_aabbExtension = 0.1f;

        /// A small length used as a collision and raint tolerance. Usually it is
        /// chosen to be numerically significant, but visually insignificant.
        public static float b2_linearSlop = 0.005f;

        /// A small angle used as a collision and raint tolerance. Usually it is
        /// chosen to be numerically significant, but visually insignificant.
        public static float b2_angularSlop = (2.0f / 180.0f * b2_pi);

        /// The radius of the polygon/edge shape skin. This should not be modified. Making
        /// this smaller means polygons will have and insufficient for continuous collision.
        /// Making it larger may create artifacts for vertex collision.
        public static float b2_polygonRadius = (2.0f * b2_linearSlop);

        // Dynamics

        /// Maximum number of contacts to be handled to solve a TOI island.
        public static int b2_maxTOIContactsPerIsland = 32;

        /// Maximum number of joints to be handled to solve a TOI island.
        public static int b2_maxTOIJointsPerIsland = 32;

        /// A velocity threshold for elastic collisions. Any collision with a relative linear
        /// velocity below this threshold will be treated as inelastic.
        public static float b2_velocityThreshold = 1.0f;

        /// The maximum linear position correction used when solving raints. This helps to
        /// prevent overshoot.
        public static float b2_maxLinearCorrection = 0.2f;

        /// The maximum angular position correction used when solving raints. This helps to
        /// prevent overshoot.
        public static float b2_maxAngularCorrection = (8.0f / 180.0f * b2_pi);

        /// The maximum linear velocity of a body. This limit is very large and is used
        /// to prevent numerical problems. You shouldn't need to adjust this.
        public static float b2_maxTranslation = 2.0f;
        public static float b2_maxTranslationSquared = (b2_maxTranslation * b2_maxTranslation);

        /// The maximum angular velocity of a body. This limit is very large and is used
        /// to prevent numerical problems. You shouldn't need to adjust this.
        public static float b2_maxRotation = (0.5f * b2_pi);
        public static float b2_maxRotationSquared = (b2_maxRotation * b2_maxRotation);

        /// This scale factor controls how fast overlap is resolved. Ideally this would be 1 so
        /// that overlap is removed in one time step. However using values close to 1 often lead
        /// to overshoot.
        public static float b2_contactBaumgarte = 0.2f;

        // Sleep

        /// The time that a body must be still before it will go to sleep.
        public static float b2_timeToSleep = 0.5f;

        /// A body cannot sleep if its linear velocity is above this tolerance.
        public static float b2_linearSleepTolerance = 0.01f;

        /// A body cannot sleep if its angular velocity is above this tolerance.
        public static float b2_angularSleepTolerance = (2.0f / 180.0f * b2_pi);

        /// Friction mixing law. Feel free to customize this.
        public static float b2MixFriction(float friction1, float friction2)
        {
	        return (float)Math.Sqrt((double)(friction1 * friction2));
        }

        /// Restitution mixing law. Feel free to customize this.
        public static float b2MixRestitution(float restitution1, float restitution2)
        {
	        return restitution1 > restitution2 ? restitution1 : restitution2;
        }
    }
}
