settings.outformat = "svg";

real px = 0.75bp;
unitsize(64px);

pen ink = rgb("231f20");
pen bodyPen = ink + 0.8px + miterjoin;
pen penA = rgb("a8322a") + 2.6px;
pen penB = rgb("1f5c99") + 1.2px;
pen guidePen = gray(0.45) + 0.6px + linetype(new real[] {4, 3});

real axisLength = 0.8;
real arrowSize = 6px;

real angleA = -6;
real angleB = 32;
real frameAngle = 14;

pair pivot = (0, 0);

pair halfA = (1.5, 0.5);
pair halfB = (1.2, 0.75);
pair localPivotA = (1.15, 0.15);
pair localPivotB = (-0.85, -0.2);

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

pair xAxis = dir(frameAngle);
pair yAxis = dir(frameAngle + 90);

// Frame A drawn wide underneath so the coincident frame B shows on top of it
draw(pivot -- pivot + axisLength * xAxis, penA, Arrow(arrowSize + 2px));
draw(pivot -- pivot + 0.6 * axisLength * yAxis, penA, Arrow(arrowSize + 2px));
draw(pivot -- pivot + axisLength * xAxis, penB, Arrow(arrowSize));
draw(pivot -- pivot + 0.6 * axisLength * yAxis, penB, Arrow(arrowSize));

label("$x_A$", pivot + axisLength * xAxis, xAxis + 0.6 * yAxis, rgb("a8322a"));
label("$x_B$", pivot + axisLength * xAxis, xAxis - 0.6 * yAxis, rgb("1f5c99"));
label("$y_A$", pivot + 0.6 * axisLength * yAxis, -xAxis, rgb("a8322a"));
label("$y_B$", pivot + 0.6 * axisLength * yAxis, xAxis, rgb("1f5c99"));

dot(pivot, ink + 4px);
