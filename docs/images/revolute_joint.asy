settings.outformat = "svg";

real px = 0.75bp;
unitsize(64px);

pen ink = rgb("231f20");
pen bodyPen = ink + 0.8px + miterjoin;
pen bodyFill = rgb("d7e3f4");
pen penA = rgb("a8322a") + 1.2px;
pen penB = rgb("1f5c99") + 1.2px;
pen guidePen = gray(0.45) + 0.6px + linetype(new real[] {4, 3});
pen anglePen = ink + 0.8px;

real axisLength = 0.8;
real arrowSize = 6px;

real angleA = 12;
real angleB = 58;

pair pivot = (0, 0);

pair halfA = (1.5, 0.7);
pair halfB = (1.4, 0.7);
pair localPivotA = (1.1, 0.1);
pair localPivotB = (-1.15, -0.18);

pair originA = pivot - rotate(angleA) * localPivotA;
pair originB = pivot - rotate(angleB) * localPivotB;

path boxA = shift(originA) * rotate(angleA) * box(-halfA, halfA);
path boxB = shift(originB) * rotate(angleB) * box(-halfB, halfB);

filldraw(boxA, palegreen, bodyPen);
filldraw(boxB, lightgray, bodyPen);

void origin(pair p)
{
  real r = 0.07;
  draw((p.x - r, p.y) -- (p.x + r, p.y), ink + 0.8px);
  draw((p.x, p.y - r) -- (p.x, p.y + r), ink + 0.8px);
}

draw(originA -- pivot, guidePen);
draw(originB -- pivot, guidePen);
origin(originA);
origin(originB);

label("$A$", originA + rotate(angleA) * (-0.35, 0), ink);
label("$B$", originB + rotate(angleB) * (0.35, 0), ink);

void frame(pair p, real angle, pen axisPen, string xs, string ys)
{
  draw(p -- p + axisLength * dir(angle), axisPen, Arrow(arrowSize));
  draw(p -- p + 0.6 * axisLength * dir(angle + 90), axisPen, Arrow(arrowSize));
  label(xs, p + axisLength * dir(angle), dir(angle), axisPen);
  label(ys, p + 0.6 * axisLength * dir(angle + 90), dir(angle + 90), axisPen);
}

frame(pivot, angleA, penA, "$x_A$", "$y_A$");
frame(pivot, angleB, penB, "$x_B$", "$y_B$");

real arcRadius = 0.42;
draw(arc(pivot, arcRadius, angleA, angleB), anglePen, EndArrow(5px));
label("$\theta$", pivot + (arcRadius + 0.12) * dir((angleA + angleB) / 2), dir((angleA + angleB) / 2), ink);

dot(pivot, ink + 4px);
