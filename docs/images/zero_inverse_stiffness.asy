settings.outformat = "svg";
settings.render = 0;

real px = 0.75bp;

pen ink = rgb("231f20");
pen goodPen = rgb("1f5c99") + 1.6px;
pen badPen = rgb("a8322a") + 1.6px;
pen guidePen = gray(0.5) + 0.9px + linetype(new real[] {4, 3});
pen text = fontsize(12pt);
pen tickText = fontsize(10pt);

unitsize(100px);
real arrowSize = 8px;

// Same setup as zero_inverse_mass.asy
real r = 0.8;
real w = 1.25 * r;

pair BodyPivot(real angle)
{
  return r * dir(angle);
}

pair Target()
{
  return (w, 0);
}

// Inverse inertia of one, so K along the error direction is the squared lever arm
real Stiffness(real angle)
{
  pair b = BodyPivot(angle);
  pair c = Target() - b;
  real s = cross(b, c);
  return s * s / dot(c, c);
}

real Impulse(real angle)
{
  return length(Target() - BodyPivot(angle)) / Stiffness(angle);
}

// The pivots line up with the center of mass at both ends of the swing. There the error points along r,
// rotation has no lever on it, and the impulse needed to remove it blows up.
real span = 180;
real xScale = 4.0 / span;
real top = 2.2;

real kMax = 0;
real lambdaMin = infinity;
for (int i = 1; i < 180; ++i)
{
  kMax = max(kMax, Stiffness(i));
  lambdaMin = min(lambdaMin, Impulse(i));
}

real kScale = 1.3 / kMax;
real lambdaScale = 0.6 / lambdaMin;

path k;
path lambda;
bool started = false;
int count = 720;
for (int i = 0; i <= count; ++i)
{
  real angle = span * i / count;
  pair q = (xScale * angle, kScale * Stiffness(angle));
  k = i == 0 ? q : k -- q;

  if (i == 0 || i == count)
    continue;

  real y = lambdaScale * Impulse(angle);
  if (y > top)
    continue;

  pair p = (xScale * angle, y);
  lambda = started ? lambda -- p : p;
  started = true;
}

real xEnd = xScale * span;
draw((0, 0) -- (xEnd + 0.3, 0), ink + 1.0px, Arrow(arrowSize));
draw((0, 0) -- (0, top + 0.15), guidePen);
draw((xEnd, 0) -- (xEnd, top + 0.15), guidePen);
label("$\theta$", (xEnd + 0.3, 0), E, ink + text);

for (int deg = 0; deg <= 180; deg += 45)
{
  real x = xScale * deg;
  draw((x, 0) -- (x, -0.05), ink + 0.8px);
  label("$" + string(deg) + "^\circ$", (x, -0.05), S, ink + tickText);
}

draw(k, goodPen);
draw(lambda, badPen);

// Anchor the labels on the falling side of the stiffness and the rising side of the impulse, away from where
// the curves cross
real kLabelAngle = 110;
label("$k = \mathbf{n}^T K \mathbf{n}$", (xScale * kLabelAngle, kScale * Stiffness(kLabelAngle)), NE,
      goodPen + text);

real lambdaLabelAngle = 90;
while (lambdaLabelAngle > 1 && lambdaScale * Impulse(lambdaLabelAngle) > 0.75 * top)
  lambdaLabelAngle -= 0.5;
label("$|\lambda| = C / k$", (xScale * lambdaLabelAngle, lambdaScale * Impulse(lambdaLabelAngle)), E,
      badPen + text);
