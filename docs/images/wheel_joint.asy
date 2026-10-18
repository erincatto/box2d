settings.outformat = "svg";

real px = 0.75bp;
unitsize(64px);

pen ink = rgb("231f20");
pen bodyPen = ink + 0.8px + miterjoin;
pen penA = rgb("a8322a") + 1.2px;
pen penB = rgb("1f5c99") + 1.2px;
pen guidePen = gray(0.45) + 0.6px + linetype(new real[] {4, 3});
pen axisPen = gray(0.35) + 0.8px + linetype(new real[] {6, 4});
pen dimensionPen = ink + 0.8px;

real axisLength = 0.8;
real arrowSize = 6px;

real chassisAngle = 6;
real axisAngle = chassisAngle - 90;
pair axis = dir(axisAngle);
pair normal = dir(axisAngle + 90);

real wheelAngle = 35;
real wheelRadius = 0.85;
real translation = 1.9;

pair frameA = (0, 0);
pair frameB = frameA + translation * axis;

pair halfA = (1.7, 0.45);
pair localFrameA = (1.25, -0.2);

pair originA = frameA - rotate(chassisAngle) * localFrameA;

path boxA = shift(originA) * rotate(chassisAngle) * box(-halfA, halfA);
path wheelB = circle(frameB, wheelRadius);

filldraw(boxA, palegreen, bodyPen);
filldraw(wheelB, lightgray, bodyPen);

draw(frameA - 0.9 * axis -- frameB + 1.3 * axis, axisPen);

void origin(pair p)
{
  real r = 0.07;
  draw((p.x - r, p.y) -- (p.x + r, p.y), ink + 0.8px);
  draw((p.x, p.y - r) -- (p.x, p.y + r), ink + 0.8px);
}

draw(originA -- frameA, guidePen);
origin(originA);

label("$A$", originA + rotate(chassisAngle) * (-0.35, 0), ink);
label("$B$", frameB + wheelRadius * dir(wheelAngle + 180) + 0.15 * dir(wheelAngle), ink);

void frame(pair p, real angle, pen axisPen, string xs, string ys)
{
  draw(p -- p + axisLength * dir(angle), axisPen, Arrow(arrowSize));
  draw(p -- p + 0.6 * axisLength * dir(angle + 90), axisPen, Arrow(arrowSize));
  label(xs, p + axisLength * dir(angle), dir(angle), axisPen);
  label(ys, p + 0.6 * axisLength * dir(angle + 90), dir(angle + 90), axisPen);
}

frame(frameA, axisAngle, penA, "", "$y_A$");
label("$x_A$", frameA + axisLength * (0.9 * axis - 0.2 * normal), axis, penA);
frame(frameB, wheelAngle, penB, "$x_B$", "$y_B$");

dot(frameA, ink + 4px);
dot(frameB, ink + 4px);

real spinRadius = wheelRadius + 0.18;
draw(arc(frameB, spinRadius, axisAngle - 50, axisAngle + 25), dimensionPen, Arrows(5px));

pair offset = -0.85 * normal;
pair startDim = frameA + offset;
pair endDim = frameB + offset;
draw(frameA -- startDim, guidePen);
draw(frameB -- endDim, guidePen);
draw(startDim -- endDim, dimensionPen, Arrows(5px));
label("$d$", (startDim + endDim) / 2, -normal, ink);
